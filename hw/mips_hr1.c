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
#include "exec/address-spaces.h"
#include "ide.h"
#include "mc146818rtc.h"
#include "char/char.h"

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
#define SERIAL1(x) (0x18000000 + 0xb0000 + x)

static qemu_irq *intctl0_irqs;
static int configdata = -1;

#define FMAX 16
static char serial_rfifo[FMAX];
static char serial_xfifo[FMAX];
static int rhead,rtail,rcnt;
static int xhead,xtail,xcnt;
static int xie,rie;
static void *iie_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq);
static QEMUTimer *serial_timer;

#define S_STATUS0_TE		0
#define M_STATUS0_TE		(0x1 << S_STATUS0_TE)
#define S_STATUS0_THE		1
#define M_STATUS0_THE		(0x1 << S_STATUS0_THE)
#define S_STATUS0_TF		2
#define M_STATUS0_TF		(0x1 << S_STATUS0_TF)
#define S_STATUS0_NKD		3
#define M_STATUS0_NKD		(0x1 << S_STATUS0_NKD)

#define S_STATUS1_RBF		0
#define M_STATUS1_RBF		(0x1 << S_STATUS1_RBF)

#define S_RXIE_RBE			0
#define M_RXIE_RBE			(0x1 << S_RXIE_RBE)
#define S_RXIE_RHF			1
#define M_RXIE_RHF			(0x1 << S_RXIE_RHF)

#define S_INTENABLE1_RBE	0
#define M_INTENABLE1_RBE	(0x1 << S_INTENABLE1_RBE)
#define S_INTENABLE1_RHF	1
#define M_INTENABLE1_RHF	(0x1 << S_INTENABLE1_RHF)
#define S_INTENABLE1_PE		2
#define M_INTENABLE1_PE		(0x1 << S_INTENABLE1_PE)
#define S_INTENABLE1_FE		3
#define M_INTENABLE1_FE		(0x1 << S_INTENABLE1_FE)
#define S_INTENABLE1_OE		4
#define M_INTENABLE1_OE		(0x1 << S_INTENABLE1_OE)
#define S_INTENABLE1_TNE	5
#define M_INTENABLE1_TNE	(0x1 << S_INTENABLE1_TNE)
#define S_INTENABLE1_TOI	6
#define M_INTENABLE1_TOI	(0x1 << S_INTENABLE1_TOI)

#define S_INTENABLE0_TE		0
#define M_INTENABLE0_TE		(0x1 << S_INTENABLE0_TE)
#define S_INTENABLE0_THE	1
#define M_INTENABLE0_THE	(0x1 << S_INTENABLE0_THE)

static void serial_checkirq(void *opaque)
{

	if(xie && !xcnt)
		qemu_irq_raise(intctl0_irqs[12]);
	else if(rie && rcnt)
		qemu_irq_raise(intctl0_irqs[12]);
	else
		qemu_irq_lower(intctl0_irqs[12]);
}

static void serial_receive(void *opaque, const uint8_t *buf, int size)
{

	while(rcnt<FMAX && size)
	{
		serial_rfifo[rhead++] = *buf++;
		if(rhead==FMAX) rhead=0;
		rcnt++;
		size--;
	}
        serial_checkirq(opaque);
}

static int serial_can_receive(void *opaque)
{

    return rcnt<FMAX;
}

static void serial_event(void *opaque, int event)
{
}

static void serial_put(void *opaque, int val)
{

	if(!xcnt)
        {
          qemu_mod_timer(serial_timer, qemu_get_clock_ns(vm_clock) + get_ticks_per_sec() / 9600*8);
        }

	if(xcnt<FMAX)
	{
		serial_xfifo[xhead++] = val;
		if(xhead==FMAX) xhead=0;
		xcnt++;
	}
        serial_checkirq(opaque);

}

static int serial_get(void *opaque)
{
	char c;
	c = serial_rfifo[rtail];
	if(rcnt)
	{
		rcnt--;
		rtail++;
		if(rtail==FMAX) rtail = 0;
	}
        serial_checkirq(opaque);
	return c;
}

static void serial_send_timer_cb(void *opaque)
{
 int val;
	if(xcnt)
        {
		val = serial_xfifo[xtail++];
		if(xtail == FMAX) xtail = 0;
                xcnt--;
		qemu_chr_fe_write(serial_hds[1], (void *)&val, 1);
        }

	if(xcnt)
          qemu_mod_timer(serial_timer, qemu_get_clock_ns(vm_clock) + get_ticks_per_sec() / 9600*8);

        serial_checkirq(opaque);
}


static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case SERIAL1(0x34):
                serial_put(opaque,val);
		 break;
		case SERIAL1(0x28):
		break;
		case SERIAL1(0x14):
                  xie = val;
        serial_checkirq(opaque);
		break;
		case SERIAL1(0x18):
                  rie = val;
        serial_checkirq(opaque);
		break;
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
        int tf,te,rf;
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1f088008:
			return configdata;
			break;
		case SERIAL1(0x20):
		     return serial_get(opaque);
		case SERIAL1(0x28):
                    tf =  (xcnt==FMAX)?M_STATUS0_TF:0;
                    te = xcnt?0:M_STATUS0_TE;
		      return tf|te;
		case SERIAL1(0x2c):
                    rf = rcnt?M_STATUS1_RBF:0;
		      return rf;
		case SERIAL1(0x14):
                    return xie;
		case SERIAL1(0x18):
                     return rie;
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
		 qemu_irq_raise(mycpu[i]->irq[6]);
		 break;
		case R_IMR_MAILBOX_CLR_CPU:
		 qemu_irq_lower(mycpu[i]->irq[6]);
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


extern void (*mypc_callback)(target_ulong pc, uint32_t opcode);
#define MASK_OP_MAJOR(op)  (op & (0x3F << 26))
#define OPC_CACHE  (0x2F << 26)
static void mypc_callback_hr1(target_ulong pc, uint32_t opcode)
{
	CPUMIPSState *env = cpu_single_env;
    	//uint32_t op, op1, op2;
		switch(MASK_OP_MAJOR(opcode))
		{
			/* MIPS64 opcodes */
			case OPC_CACHE:
				printf("pc=0x%x %s, ra=0x%x %s\n",(int)pc,lookup_symbol(pc),env->active_tc.gpr[31],lookup_symbol(env->active_tc.gpr[31]));
				break;
			default:
				break;
		}

}



static const int sector_len = 32 * 1024;
static void mips_hr1_init (QEMUMachineInitArgs *args)
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
	int be;
	DriveInfo *dinfo=NULL;
	int i;


    /* init CPUs */
    if (cpu_model == NULL) {
        cpu_model = "ls232e";
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
    //env->CP0_PRid   |= 0x6303; // gx 

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

	memory_region_init_ram(ram, "mips_hr1.ram", ram_size);
	vmstate_register_ram_global(ram);
#if 0
	MemoryRegion *lowram = g_new(MemoryRegion, 1);
        //memory_region_init_alias(lowram, "mips_hr1.lowram", ram, 0, MIN(ram_size,256*0x100000));
    /* allocate RAM */
	//memory_region_add_subregion(address_space_mem, 0, lowram);

	//memory_region_add_subregion(address_space_mem, 0x80000000, ram);
#else
	memory_region_add_subregion(address_space_mem, 0, ram);
#endif

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info[0]->vector = load_kernel();
	aui_boot_code[6] = (aui_boot_code[6]&0xffff0000)|((reset_info[0]->vector&0xffff0000)>>16);
	aui_boot_code[7] = (aui_boot_code[7]&0xffff0000)|((reset_info[0]->vector&0xffff));
    }


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

        intctl0_irqs = iie_intctl_init(address_space_mem, 0x18020000, env->irq[2]);

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &mips_qemu_ops, (void *)0x1f088008, "0x1f088008", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1f088008, iomem);
	}
	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &mips_qemu_ops, (void *)(0x18000000 + 0xb0000 + 0), "serial", 0x80);
                memory_region_add_subregion(address_space_mem, (0x18000000 + 0xb0000 + 0), iomem);
	if (serial_hds[1])
        {
        qemu_chr_add_handlers(serial_hds[1], serial_can_receive, serial_receive, serial_event, serial_hds[1]);
         serial_timer = qemu_new_timer_ns(vm_clock, (QEMUTimerCB *) serial_send_timer_cb, serial_hds[1]);
         qemu_mod_timer(serial_timer, qemu_get_clock_ns(vm_clock) + get_ticks_per_sec() / 9600*8);
        }
	}
	if (serial_hds[0])
		serial_mm_init(address_space_mem, 0x1fe00000, 0,env->irq[4],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);
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

	mypc_callback =  mypc_callback_hr1;
}




//-----------------

//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define DPRINTF(fmt, args...) \
	do { printf("IRQ: " fmt , ##args); } while (0)
#else
#define DPRINTF(fmt, args...)
#endif

typedef struct GS232_INTCTLState {
	uint32_t inten[2];
	uint32_t intmask[2];
	uint32_t intforce[2];
	uint32_t rawstatus[2];
	uint32_t status[2];
	uint32_t maskstatus[2];
	uint32_t finalstatus[2];
	uint32_t irq_vector[16];
	uint32_t reserve;
	uint32_t fiq_inten;
	uint32_t fiq_intmask;
	uint32_t fiq_intforce;
	uint32_t fiq_rawstatus;
	uint32_t fiq_status;
	uint32_t fiq_finalstatus;
	uint32_t irq_plevel;
	uint32_t reserve1;
	uint32_t version;
	uint32_t reserve2;
	uint32_t irq_pr[1]; 
	uint32_t irq_vector_default; 

	qemu_irq cpu_irq;
	uint32_t intreg_pending;
	uint32_t pil_out;
} GS232_INTCTLState;

#define INTCTL_SIZE 0x100
#define INTCTLM_MAXADDR 0x13
#define INTCTLM_SIZE (INTCTLM_MAXADDR + 1)
#define INTCTLM_MASK 0x1f
#define MASTER_IRQ_MASK ~0x0fa2007f
#define MASTER_DISABLE 0x80000000
#define CPU_SOFTIRQ_MASK 0xfffe0000
#define CPU_HARDIRQ_MASK 0x0000fffe
#define CPU_IRQ_INT15_IN 0x0004000
#define CPU_IRQ_INT15_MASK 0x80000000


#define 	HI_IRQ_INTEN				(0x00)
#define 	HI_IRQ_INTMASK				(0x08)
#define 	HI_IRQ_INTFORCE				(0x10)
#define 	HI_IRQ_RAWSTATUS			(0x18)
#define 	HI_IRQ_STATUS				(0x20)
#define 	HI_IRQ_MASKSTATUS			(0x28)
#define 	HI_IRQ_FINALSTATUS			(0x30)
#define 	HI_FIQ_INTEN				(0xC0)
#define 	HI_FIQ_INTMASK				(0xC4)
#define 	HI_FIQ_INTFORCE				(0xC8)
#define 	HI_FIQ_RAWSTATUS			(0xCC)
#define 	HI_FIQ_STATUS				(0xD0)
#define 	HI_FIQ_FINALSTATUS			(0xD4)
#define 	HI_IRQ_PLEVEL				(0xD8)

static void iie_check_interrupts(void *opaque);

// per-cpu interrupt controller
static uint64_t iie_intctl_mem_readl(void *opaque, hwaddr addr, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr, ret;

	saddr = addr >> 2;
	switch (saddr) {
		case 6: //rawstatus
			ret =  s->intreg_pending;
			break;
		case 7: //rawstatus
			ret =  s->intreg_pending;
			break;
		case 8: //status
			ret = s->inten[0] & s->intreg_pending;
			break;
		case 9: //status
			ret = s->inten[1] & s->intreg_pending;
			break;
		case 10: //maskstatus
			ret = ~s->intmask[0] & s->intreg_pending;
			break;
		case 11: //maskstatus
			ret = ~s->intmask[1] & s->intreg_pending;
			break;
		case 12: //finalstatus
			ret = s->inten[0] /*& ~s->intmask[0]*/ & s->intreg_pending;
			break;
		case 13: //finalstatus
			ret = s->inten[1] /*& ~s->intmask[1]*/ & s->intreg_pending;
			break;
		default:
			ret = *(int *)s;
			break;
	}
	DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr, ret);

	return ret;
}

static void iie_intctl_mem_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr;

	saddr = addr >> 2;
	if(saddr>=6 && saddr<14) return;
	if(saddr>14) return;
	switch (saddr) {
		default:
			*(uint32_t *)((void *)s+addr) = val;
			iie_check_interrupts(s);
			break;
	}
}

static const MemoryRegionOps iie_intctl_mem_ops = {
    .read = iie_intctl_mem_readl,
    .write = iie_intctl_mem_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static void iie_check_interrupts(void *opaque)
{
	GS232_INTCTLState *s = opaque;
	uint32_t pil_pending;


	pil_pending = s->inten[0] /*& ~s->intmask[0]*/ & s->intreg_pending;


	if (pil_pending ) {
		if (!s->pil_out)
			qemu_irq_raise(s->cpu_irq);
	} else {
		if (s->pil_out)
			qemu_irq_lower(s->cpu_irq);
	}
	s->pil_out = pil_pending;
	DPRINTF("pending %x \n", pil_pending);
}

/*
 * "irq" here is the bit number in the system interrupt register to
 * separate serial and keyboard interrupts sharing a level.
 */
static void iie_set_irq(void *opaque, int irq, int level)
{
	GS232_INTCTLState *s = opaque;
	uint32_t mask = 1 << irq;

	DPRINTF("Set irq %d level %d\n", irq,
			level);
	if (level) {
	if(!s->intreg_pending)
		s->intreg_pending |= mask;
	} else {
	if(s->intreg_pending)
		s->intreg_pending &= ~mask;
	}
	iie_check_interrupts(s);
}


static void iie_intctl_save(QEMUFile *f, void *opaque)
{
	GS232_INTCTLState *s = opaque;

	qemu_put_be32s(f, &s->intreg_pending);
}

static int iie_intctl_load(QEMUFile *f, void *opaque, int version_id)
{
	GS232_INTCTLState *s = opaque;

	if (version_id != 1)
		return -EINVAL;

	qemu_get_be32s(f, &s->intreg_pending);
	iie_check_interrupts(s);
	return 0;
}

static void iie_intctl_reset(void *opaque)
{
	GS232_INTCTLState *s = opaque;

	s->intreg_pending = 0;
	iie_check_interrupts(s);
}


static void *iie_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq)
{
	qemu_irq *irqs;
	GS232_INTCTLState *s;

	s = g_malloc0(sizeof(GS232_INTCTLState));
	if (!s)
		return NULL;

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &iie_intctl_mem_ops, s, "iie_int", INTCTL_SIZE);
                memory_region_add_subregion(mr, addr, iomem);
	}

	s->cpu_irq = parent_irq;

	register_savevm(NULL, "iie_intctl", addr, 1, iie_intctl_save, iie_intctl_load, s);
	qemu_register_reset(iie_intctl_reset, s);
	irqs = qemu_allocate_irqs(iie_set_irq, s, 32);

	iie_intctl_reset(s);
	return irqs;
}

QEMUMachine mips_hr1_machine = {
    .name = "hr1",
    .desc = "Godson3 Multicore platform",
    .init = mips_hr1_init,
    .max_cpus = 255,
    DEFAULT_MACHINE_OPTIONS,
};

static void mips_machine_init(void)
{
    qemu_register_machine(&mips_hr1_machine);
}

machine_init(mips_machine_init);
