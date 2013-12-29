/*
 * QEMU loongson 3a develop board emulation
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
#include "loader.h"
#include "elf.h"
#include "sysbus.h"
#include "synopGMAC.h"
#include "exec/address-spaces.h"
#include "ide.h"
#include "mc146818rtc.h"

#ifdef TARGET_WORDS_BIGENDIAN
#define BIOS_FILENAME "mips_bios.bin"
#else
#define BIOS_FILENAME "mipsel_bios.bin"
#endif

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)
#define MAX_CPUS 2


/* i8254 PIT is attached to the IRQ0 at PIC i8259 */

static CPUMIPSState *mycpu[MAX_CPUS];

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
    target_ulong a0,a1,a2;
} loaderparams;


#include "hr_rom.h"
static int configdata = -1;
static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1f088008:
			configdata = val;
			break;
		case 0x1f078100:
		case 0x1f078200:
		break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1f088008:
			return configdata;
			break;
	}
	return 0;
}

static const MemoryRegionOps mips_qemu_ops = {
    .read = mips_qemu_readl,
    .write = mips_qemu_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define A_IMR_CPU0_BASE		    0x1f078100
#define A_IMR_CPU1_BASE     	0x1f078200
#define R_IMR_MAILBOX_CPU   	0x0
#define R_IMR_MAILBOX_ENABLE	0x8
#define R_IMR_MAILBOX_SET_CPU	0x10
#define R_IMR_MAILBOX_CLR_CPU	0x18
#define R_IMR_MAILBOX_MAILBOX0	0x20
#define R_IMR_MAILBOX_MAILBOX1	0x28
#define R_IMR_MAILBOX_MAILBOX2	0x30
#define R_IMR_MAILBOX_MAILBOX3	0x38
static unsigned long long mailboxreg[2][4];

static void mips_ipi_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	long i = (long)opaque;
	switch(addr)
	{
		case R_IMR_MAILBOX_CPU:
		case R_IMR_MAILBOX_ENABLE:
		case R_IMR_MAILBOX_MAILBOX1:
		 mailboxreg[i][1] = val;
		break;
		case R_IMR_MAILBOX_MAILBOX2:
		 mailboxreg[i][2] = val;
		break;
		case R_IMR_MAILBOX_MAILBOX3:
		 mailboxreg[i][3] = val;
		break;
		case R_IMR_MAILBOX_SET_CPU:
		 qemu_irq_raise(mycpu[i]->irq[4]);
		 break;
		case R_IMR_MAILBOX_CLR_CPU:
		 qemu_irq_lower(mycpu[i]->irq[4]);
		 break;
		case R_IMR_MAILBOX_MAILBOX0:
		 mailboxreg[i][0] = val;
		break;
	}
}

static uint64_t mips_ipi_readl (void *opaque, hwaddr addr, unsigned size)
{
	long i = (long)opaque;
	uint64_t ret = 0;
	switch(addr)
	{
		case R_IMR_MAILBOX_CPU:
		break;
		case R_IMR_MAILBOX_ENABLE:
		break;
		case R_IMR_MAILBOX_MAILBOX1:
		 ret =  mailboxreg[i][1];
		break;
		case R_IMR_MAILBOX_MAILBOX2:
		 ret =  mailboxreg[i][2];
		break;
		case R_IMR_MAILBOX_MAILBOX3:
		 ret =  mailboxreg[i][3];
		break;
		case R_IMR_MAILBOX_SET_CPU:
		break;
		case R_IMR_MAILBOX_CLR_CPU:
		break;
		case R_IMR_MAILBOX_MAILBOX0:
		 ret =  mailboxreg[i][0];
		break;
	}
	return ret;
}

static const MemoryRegionOps mips_ipi_ops = {
    .read = mips_ipi_readl,
    .write = mips_ipi_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

typedef struct ResetData {
	MIPSCPU *cpu;
	uint64_t vector;
} ResetData;


#define BOOTPARAM_PHYADDR ((64 << 20))
#define BOOTPARAM_ADDR (0x80000000+BOOTPARAM_PHYADDR)
// should set argc,argv
//env->gpr[REG][env->current_tc]
static int set_bootparam(ram_addr_t initrd_offset,long initrd_size)
{
	char memenv[32];
	char highmemenv[32];
	const char *pmonenv[]={"cpuclock=200000000",memenv,highmemenv};
	int i;
	long params_size;
	char *params_buf;
	unsigned int *parg_env;
	int ret;

	/* Store command line.  */
	params_size = 264;
	params_buf = g_malloc(params_size);

	parg_env=(void *)params_buf;

	/*
	 * pram buf like this:
	 *argv[0] argv[1] 0 env[0] env[1] ...env[i] ,0, argv[0]'s data , argv[1]'s data ,env[0]'data,...,env[i]'s dat,0
	 */

	//*count user special env
	for(ret=0,i=0;environ[i];i++)
		if(!strncmp(environ[i],"ENV_",4))ret+=4;

	//jump over argv and env area
	ret +=(3+sizeof(pmonenv)/sizeof(char *)+1)*4;
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

	//env
	sprintf(memenv,"memsize=%d",loaderparams.ram_size>0x10000000?256:(loaderparams.ram_size>>20));
	sprintf(highmemenv,"highmemsize=%d",loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0);


	for(i=0;i<sizeof(pmonenv)/sizeof(char *);i++)
	{
		*parg_env++=BOOTPARAM_ADDR+ret;
		ret +=1+snprintf(params_buf+ret,256-ret,"%s",pmonenv[i]);
	}

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
	loaderparams.a0 = 2;
	loaderparams.a1 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR;
	loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR +12;
	
return 0;
}


static int64_t load_kernel(void)
{
    int64_t entry, kernel_low, kernel_high;
    long kernel_size, initrd_size;
    ram_addr_t initrd_offset;

	if(getenv("BOOTROM"))
	{
            initrd_size = load_image_targphys(loaderparams.kernel_filename,
                                     strtoul(getenv("BOOTROM"),0,0),ram_size); //qemu_get_ram_ptr
	return 0;
	}
	kernel_size = load_elf(loaderparams.kernel_filename, cpu_mips_kseg0_to_phys, NULL,
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
	    if(getenv("INITRD_OFFSET")) initrd_offset=strtoul(getenv("INITRD_OFFSET"),0,0);
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                                     initrd_offset,ram_size-initrd_offset); //qemu_get_ram_ptr
        }
        if (initrd_size == (target_ulong) -1) {
            fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
                    loaderparams.initrd_filename);
            exit(1);
        }
    }


	if(0)
	 set_bootparam(initrd_offset, initrd_size);
	
return entry;
}
static void main_cpu_reset(void *opaque)
{
	ResetData *s = (ResetData *)opaque;
	CPUMIPSState *env = &s->cpu->env;

	cpu_reset(CPU(s->cpu));
	env->active_tc.PC = s->vector;
	env->active_tc.gpr[4]=loaderparams.a0;
	env->active_tc.gpr[5]=loaderparams.a1;
	env->active_tc.gpr[6]=loaderparams.a2;
}



static void hr_serial_set_irq(void *opaque, int irq, int level)
{
	int i;
	for(i=0;i<smp_cpus;i++)
	qemu_set_irq(mycpu[i]->irq[2],level);
}

static const int sector_len = 32 * 1024;
static void mips_hr_init (QEMUMachineInitArgs *args)
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
	ResetData *reset_info[10];
	qemu_irq *hr_serial_irq ;
	int be;
	DriveInfo *dinfo=NULL;
	int i;


    /* init CPUs */
    if (cpu_model == NULL) {
        cpu_model = "godson3";
    }
    /* init CPUs */
{

  for(i = 0; i < smp_cpus; i++) {
    printf("==== init smp_cpus=%d ====\n", i);
    cpu = cpu_mips_init(cpu_model);
    if (cpu == NULL) {
	    fprintf(stderr, "Unable to find CPU definition\n");
	    exit(1);
    }

    env = &cpu->env;
    mycpu[i] = env;

    env->CP0_EBase |= i;


    if (i != 0)
      env->halted = 0;


    env->CP0_Status |= (1 << CP0St_KX); // gx 
    //env->CP0_PRid   |= def->CP0_PRid; // gx 
    env->CP0_PRid   |= 0x6303; // gx 

    reset_info[i] = g_malloc0(sizeof(ResetData));
    reset_info[i]->cpu = cpu;
    reset_info[i]->vector = env->active_tc.PC;
    qemu_register_reset(main_cpu_reset, reset_info[i]);


    /* Init CPU internal devices */
    cpu_mips_irq_init_cpu(env);
    cpu_mips_clock_init(env);

  }
  env = mycpu[0];

}

	memory_region_init_ram(ram, "mips_hr.ram", ram_size);
	vmstate_register_ram_global(ram);
#if 0
	MemoryRegion *lowram = g_new(MemoryRegion, 1);
        //memory_region_init_alias(lowram, "mips_hr.lowram", ram, 0, MIN(ram_size,256*0x100000));
    /* allocate RAM */
	//memory_region_add_subregion(address_space_mem, 0, lowram);

	//memory_region_add_subregion(address_space_mem, 0x80000000, ram);
#else
	memory_region_add_subregion(address_space_mem, 0, ram);
#endif


    /* Try to load a BIOS image. If this fails, we continue regardless,
       but initialize the hardware ourselves. When a kernel gets
       preloaded we also initialize the hardware, since the BIOS wasn't
       run. */
    if (bios_name == NULL)
        bios_name = BIOS_FILENAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = get_image_size(filename);
    } else {
        bios_size = -1;
    }
#ifdef TARGET_WORDS_BIGENDIAN
    be = 1;
#else
    be = 0;
#endif
    if ((bios_size > 0) && (bios_size <= BIOS_SIZE)) {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, "mips_r4k.bios", BIOS_SIZE);
        vmstate_register_ram_global(bios);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

        load_image_targphys(filename, 0x1fc00000, BIOS_SIZE);
    } else if ((dinfo = drive_get(IF_PFLASH, 0, 0)) != NULL) {
        uint32_t mips_rom = 0x00400000;
        if (!pflash_cfi01_register(0x1fc00000, NULL, "mips_r4k.bios", mips_rom,
                                   dinfo->bdrv, sector_len,
                                   mips_rom / sector_len,
                                   4, 0, 0, 0, 0, be)) {
            fprintf(stderr, "qemu: Error registering flash memory.\n");
	}
    }
    else {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, "mips_r4k.bios", BIOS_SIZE);
        vmstate_register_ram_global(bios);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

	bios_size = sizeof(aui_boot_code);
	rom_add_blob_fixed("bios",aui_boot_code,bios_size,0x1fc00000);
    }

    if (filename) {
        g_free(filename);
    }

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info[0]->vector = load_kernel();
    }

	hr_serial_irq = qemu_allocate_irqs(hr_serial_set_irq, mycpu, 1);

    if (serial_hds[0])
            serial_mm_init(address_space_mem, 0xf2000000, 0,hr_serial_irq[0],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);
	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &mips_qemu_ops, (void *)0x1f088008, "0x1f088008", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1f088008, iomem);
	}
	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &mips_ipi_ops, (void *)0, "0x1f078100", 0x40);
		//env->irq[4]
                memory_region_add_subregion(address_space_mem, 0x1f078100, iomem);
	}
	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &mips_ipi_ops, (void *)1, "0x1f078200", 0x40);
                memory_region_add_subregion(address_space_mem, 0x1f078200, iomem);
	}

}

QEMUMachine mips_hr_machine = {
    .name = "hr",
    .desc = "Godson3 Multicore platform",
    .init = mips_hr_init,
    .max_cpus = 255,
    DEFAULT_MACHINE_OPTIONS,
};

static void mips_machine_init(void)
{
    qemu_register_machine(&mips_hr_machine);
}

machine_init(mips_machine_init);
