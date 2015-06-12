/*
 * QEMU loongson 1b develop board emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "hw.h"
#include "mips.h"
#include "mips_cpudevs.h"
#include "pc.h"
#include "serial.h"
#include "isa.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "boards.h"
#include "flash.h"
#include "qemu/log.h"
#include "mips-bios.h"
#include "ide.h"
#include "loader.h"
#include "elf.h"
#include "sysbus.h"
#include "synopGMAC.h"
#include "sysemu/blockdev.h"
#include "ssi.h"
#include "i2c.h"
#include "exec/address-spaces.h"
#include "qdev-addr.h"

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };

/* i8254 PIT is attached to the IRQ0 at PIC i8259 */

static struct _loaderparams {
	int ram_size;
	const char *kernel_filename;
	const char *kernel_cmdline;
	const char *initrd_filename;
} loaderparams;

static int clkreg[2];
static MemoryRegion *ddrcfg_iomem;
static int reg0420[2]={0,0x100000};

unsigned int aui_boot_code[] =
{
0x3c089000, /* lui t0,0x9000		*/
0x40887801, /* mtc0 t0,$15,1		*/
0x40806000, /* mtc0 zero,c0_status		*/
0x03e06821, /* move t5,ra		*/
0x3c0201ff, /* lui v0,0x1ff		*/
0x3442e000, /* ori v0,v0,0xe000		*/
0x40822800, /* mtc0 v0,c0_pagemask		*/
0x24080000, /* li t0,0		*/
0x3c090040, /* lui t1,0x40		*/
0x3529001f, /* ori t1,t1,0x1f		*/
0x3c0a0004, /* lui t2,0x4		*/
0x3c0b0200, /* lui t3,0x200		*/
0x240c0008, /* li t4,8		*/
0x0411000c, /* bal 0x68		*/
0x24020000, /* li v0,0		*/
0x00000000, /* nop 		*/
0x240c0010, /* li t4,16		*/
0x3c082000, /* lui t0,0x2000		*/
0x3c090040, /* lui t1,0x40		*/
0x04110006, /* bal 0x68		*/
0x35290017, /* ori t1,t1,0x17		*/
0x00000000, /* nop 		*/
0x01a0f821, /* move ra,t5		*/
0x03e00008, /* jr ra		*/
0x00000000, /* nop 		*/
0x00000000, /* nop 		*/
0x40820000, /* mtc0 v0,c0_index		*/
0x00000000, /* nop 		*/
0x40891000, /* mtc0 t1,c0_entrylo0		*/
0x012a4821, /* addu t1,t1,t2		*/
0x40891800, /* mtc0 t1,c0_entrylo1		*/
0x00000000, /* nop 		*/
0x40885000, /* mtc0 t0,c0_entryhi		*/
0x42000002, /* tlbwi 		*/
0x010b4021, /* addu t0,t0,t3		*/
0x012a4821, /* addu t1,t1,t2		*/
0x24420001, /* addiu v0,v0,1		*/
0x144cfff4, /* bne v0,t4,0x68		*/
0x00000000, /* nop 		*/
0x03e00008, /* jr ra		*/
0x00000000, /* nop 		*/

}
;
static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe78030:
		case 0x1fe78034:
			clkreg[(addr - 0x1fe78030)/4] = val;
			break;

		case 0x1fd00420:
			reg0420[0] = val;
			break;
		case 0x1fd00424:

			if(val&0x100000 && !(reg0420[1]&0x100000))
			{
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);
			}

			if(!(val&0x100000) && (reg0420[1]&0x100000))
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ffffe00&TARGET_PAGE_MASK, ddrcfg_iomem, 1);
			}

			reg0420[1] = val;

			break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe78030:
		case 0x1fe78034:
			return clkreg[(addr - 0x1fe78030)/4];
			break;
		case 0x0ffffe10:
			return 1;
			break;
		case 0x0ffffef0:
			return 0x100000;
			break;
		case 0x0ffffef2:
			return 0x10;
			break;
	}
	return 0;
}

static const MemoryRegionOps mips_qemu_ops = {
    .read = mips_qemu_readl,
    .write = mips_qemu_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


typedef struct ResetData {
	MIPSCPU *cpu;
	uint64_t vector;
} ResetData;

static uint64_t cpu_mips_kseg1_to_phys(void *opaque, uint64_t addr)
{
    if (!bios_name || strcmp(bios_name, "tlbat0"))
     return addr & 0x1fffffffll;
    else
     return (addr|0x10000000) & 0x1fffffffll;
}

static int64_t load_kernel(void)
{
	int64_t entry, kernel_low, kernel_high;
	long kernel_size;
#if 0
        int  initrd_size, params_size;
	char *params_buf;
	ram_addr_t initrd_offset;
	int ret;

	if(getenv("BOOTROM"))
	{
		initrd_size = load_image_targphys(loaderparams.kernel_filename,
				strtoul(getenv("BOOTROM"),0,0),ram_size); //qemu_get_ram_ptr
		return 0;
	}
#endif
	kernel_size = load_elf(loaderparams.kernel_filename, cpu_mips_kseg1_to_phys, NULL,
			(uint64_t *)&entry, (uint64_t *)&kernel_low,
			(uint64_t *)&kernel_high,0,ELF_MACHINE, 1);
	if (kernel_size >= 0) {
		if ((entry & ~0x7fffffffULL) == 0x80000000)
			entry = (int32_t)entry;
	} else {
		fprintf(stderr, "qemu: could not load kernel '%s'\n",
				loaderparams.kernel_filename);
		exit(1);
	}

#if 0

	/* load initrd */
	initrd_size = 0;
	initrd_offset = 0;
	if (loaderparams.initrd_filename) {
		initrd_size = get_image_size (loaderparams.initrd_filename);
		if (initrd_size > 0) {
			initrd_offset = (kernel_high + ~TARGET_REALPAGE_MASK) & TARGET_REALPAGE_MASK;
			if (initrd_offset + initrd_size > ram_size) {
				fprintf(stderr,
						"qemu: memory too small for initial ram disk '%s'\n",
						loaderparams.initrd_filename);
				exit(1);
			}
			initrd_size = load_image_targphys(loaderparams.initrd_filename,
					initrd_offset,ram_size-initrd_offset); //qemu_get_ram_ptr
		}
		if (initrd_size == (target_ulong) -1) {
			fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
					loaderparams.initrd_filename);
			exit(1);
		}
	}

	/* Store command line.  */
	params_size = 264;
	params_buf = g_malloc(params_size);



#define BOOTPARAM_PHYADDR ((64 << 20) - 264)
#define BOOTPARAM_ADDR (0x80000000+BOOTPARAM_PHYADDR)
	// should set argc,argv
	//env->gpr[REG][env->current_tc]
	{
		char memenv[32];
		int i;
		unsigned int *parg_env=(void *)params_buf;
		/*
		 * pram buf like this:
		 *argv[0] argv[1] 0 env[0] env[1] ...env[i] ,0, argv[0]'s data , argv[1]'s data ,env[0]'data,...,env[i]'s dat,0
		 */
		
		sprintf(memenv,"%d",loaderparams.ram_size>0x10000000?256:(loaderparams.ram_size>>20));
		setenv("ENV_memsize", memenv, 1);
		sprintf(memenv,"%d",loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0);
		setenv("ENV_highmemsize", memenv, 1);
		setenv("ENV_cpuclock", "200000000", 0);
		setenv("ENV_busclock", "33333333", 0);

		//*count user special env
		for(ret=0,i=0;environ[i];i++)
			if(!strncmp(environ[i],"ENV_",4))ret+=4;

		//jump over argv and env area
		ret +=(3+1)*4;
		//argv0
		*parg_env++=BOOTPARAM_ADDR+ret;
		ret +=1+snprintf(params_buf+ret,256-ret,"g");
		//argv1
		*parg_env++=BOOTPARAM_ADDR+ret;
		if (initrd_size > 0) {
			ret +=1+snprintf(params_buf+ret,256-ret, "rd_start=0x" TARGET_FMT_lx " rd_size=%li %s",
					PHYS_TO_VIRT((uint32_t)initrd_offset),
					initrd_size, loaderparams.kernel_cmdline);
		} else {
			ret +=1+snprintf(params_buf+ret, 256-ret, "%s", loaderparams.kernel_cmdline);
		}
		//argv2
		*parg_env++=0;


		for(i=0;environ[i];i++)
		{
			if(!strncmp(environ[i],"ENV_",4)){
				*parg_env++=BOOTPARAM_ADDR+ret;
				ret +=1+snprintf(params_buf+ret,256-ret,"%s",&environ[i][4]);
			}
		}
		*parg_env++=0;
		rom_add_blob_fixed("params", params_buf, params_size,
				BOOTPARAM_PHYADDR);

	}
#endif
	return entry;
}
static void main_cpu_reset(void *opaque)
{
	ResetData *s = (ResetData *)opaque;
	CPUMIPSState *env = &s->cpu->env;

	cpu_reset(CPU(s->cpu));
	env->CP0_IntCtl = 0xfc000000;
    if (!bios_name || strcmp(bios_name, "tlbat0"))
	env->active_tc.PC = s->vector;
    else
    {
	env->active_tc.PC = 0xbfc00000;
	env->active_tc.gpr[31]=s->vector;
    }
#if 0
	env->active_tc.gpr[4]=2;
	env->active_tc.gpr[5]=0x80000000+BOOTPARAM_PHYADDR;
	env->active_tc.gpr[6]=0x80000000+BOOTPARAM_PHYADDR +12;
#endif
}


void *hs1b_intctl_init(hwaddr addr,qemu_irq parent_irq);

static const int sector_len = 32 * 1024;

extern void (*mypc_callback)(target_ulong pc, uint32_t opcode);

static void mypc_callback_for_net( target_ulong pc, uint32_t opcode)
{
#if 0
	CPUMIPSState *env = cpu_single_env;
        static int32_t CP0_Compare = 0;
#define ST0_CU2			0x40000000
 if(env->CP0_Compare!=CP0_Compare)
 {
	printf("\r\n0x%08x on 0x%llx\r\n",env->CP0_Compare, (long long)pc);
        CP0_Compare = env->CP0_Compare;
//	exit(0);
 }
#endif

}


void *hisense_intctl_init(hwaddr addr,qemu_irq parent_irq);

static void mips_hs1b_init (QEMUMachineInitArgs *args)
{
	ram_addr_t ram_size = args->ram_size;
	const char *cpu_model = args->cpu_model;
	const char *kernel_filename = args->kernel_filename;
	const char *kernel_cmdline = args->kernel_cmdline;
	const char *initrd_filename = args->initrd_filename;
	char *filename;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	MemoryRegion *bios;
	int bios_size;
	MIPSCPU *cpu;
	CPUMIPSState *env;
	ResetData *reset_info;
	qemu_irq *hisense_irq;


	/* init CPUs */
	if (cpu_model == NULL) {
#ifdef TARGET_MIPS64
		cpu_model = "LS232";
#else
		cpu_model = "LS232";
#endif
	}

		cpu = cpu_mips_init(cpu_model);
		if (cpu == NULL) {
			fprintf(stderr, "Unable to find CPU definition\n");
			exit(1);
		}
		env = &cpu->env;


		reset_info = g_malloc0(sizeof(ResetData));
		reset_info->cpu = cpu;
		reset_info->vector = env->active_tc.PC;
		qemu_register_reset(main_cpu_reset, reset_info);

		/* allocate RAM */
	memory_region_init_ram(ram, "vddr", 0x0f000000);
	vmstate_register_ram_global(ram);

	memory_region_add_subregion(address_space_mem, 0x10000000, ram);


	{
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	memory_region_init_ram(ram, "mddr", 0x10000000);
	vmstate_register_ram_global(ram);
	memory_region_add_subregion(address_space_mem, 0, ram);
        }


    if (kernel_filename) {
	uint64_t vector;
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        vector = load_kernel();
	if(vector)
        reset_info->vector = vector;
    }


    if (!bios_name || strcmp(bios_name, "tlbat0"))
   {
	if(!bios_name)
        bios_name = BIOS_FILENAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = get_image_size(filename);
    } else {
        bios_size = -1;
    }

    if ((bios_size > 0) && (bios_size <= BIOS_SIZE)) {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, "mips_r4k.bios", BIOS_SIZE);
        vmstate_register_ram_global(bios);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

        load_image_targphys(filename, 0x1fc00000, BIOS_SIZE);
    }
    } else 
    {

        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, "mips_r4k.bios", BIOS_SIZE);
        vmstate_register_ram_global(bios);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

	bios_size = sizeof(aui_boot_code);
	rom_add_blob_fixed("bios",aui_boot_code,bios_size,0x1fc00000);

    }

	/* Init CPU internal devices */
	cpu_mips_irq_init_cpu(env);
	cpu_mips_clock_init(env);


	hisense_irq =hisense_intctl_init(0x1F080000,env->irq[4]);



	if (serial_hds[0])
		serial_mm_init(address_space_mem, 0x1f280000, 2,hisense_irq[9],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);

	mypc_callback =  mypc_callback_for_net;

}

QEMUMachine mips_hs1b_machine = {
	.name = "hs1b",
	.desc = "mips hs1b platform",
	.init = mips_hs1b_init,
	DEFAULT_MACHINE_OPTIONS,
};

static void mips_machine_init(void)
{
	qemu_register_machine(&mips_hs1b_machine);
}

machine_init(mips_machine_init);

