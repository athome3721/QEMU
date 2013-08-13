/*
 * QEMU loongson 1a develop board emulation
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
#include "ide/internal.h"

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

static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
	default:
			break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fda0004:
			return 1;
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

static int64_t load_kernel(void)
{
	int64_t entry, kernel_low, kernel_high;
	long kernel_size, initrd_size, params_size;
	char *params_buf;
	ram_addr_t initrd_offset;
	int ret;

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
		char highmemenv[32];
		const char *pmonenv[]={"cpuclock=200000000",memenv,highmemenv};
		int i;
		unsigned int *parg_env=(void *)params_buf;
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

	}
	return entry;
}
static void main_cpu_reset(void *opaque)
{
	ResetData *s = (ResetData *)opaque;
	CPUMIPSState *env = &s->cpu->env;

	cpu_reset(CPU(s->cpu));
	env->CP0_IntCtl = 0xfc000000;
	env->active_tc.PC = s->vector;
	env->active_tc.gpr[4]=2;
	env->active_tc.gpr[5]=0x80000000+BOOTPARAM_PHYADDR;
	env->active_tc.gpr[6]=0x80000000+BOOTPARAM_PHYADDR +12;
}


static const int sector_len = 32 * 1024;

static void *iie_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq);

static void mips_iie_init (QEMUMachineInitArgs *args)
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
	qemu_irq *iie_irq;
	DriveInfo *flash_dinfo=NULL;


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
	memory_region_init_ram(ram, "mips_r4k.ram", ram_size);
	vmstate_register_ram_global(ram);

	memory_region_add_subregion(address_space_mem, 0, ram);

	//memory_region_init_io(iomem, &mips_qemu_ops, NULL, "mips-qemu", 0x10000);
	//memory_region_add_subregion(address_space_mem, 0x1fbf0000, iomem);

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

    if ((bios_size > 0) && (bios_size <= BIOS_SIZE)) {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, "mips_r4k.bios", BIOS_SIZE);
        vmstate_register_ram_global(bios);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

        load_image_targphys(filename, 0x1fc00000, BIOS_SIZE);
    } else if (!(flash_dinfo = drive_get_next(IF_PFLASH)))
    {
	/* not fatal */
        fprintf(stderr, "qemu: Warning, could not load MIPS bios '%s'\n",
		bios_name);
    }
    if (filename) {
        g_free(filename);
    }

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info->vector = load_kernel();
    }


	/* Init CPU internal devices */
	cpu_mips_irq_init_cpu(env);
	cpu_mips_clock_init(env);

	iie_irq =iie_intctl_init(get_system_memory(), 0x1Fd20000, env->irq[2]);


	if (serial_hds[0])
		serial_mm_init(address_space_mem, 0x1fd30000, 0,iie_irq[0],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[1])
		serial_mm_init(address_space_mem, 0x1fd40000, 0,iie_irq[1],115200,serial_hds[1], DEVICE_NATIVE_ENDIAN);



	sysbus_create_simple("exynos4210-ehci-usb",0x1f920000, iie_irq[7]);
	{
		DeviceState *dev;
		dev = qdev_create(NULL, "sysbus-ohci");
		qdev_prop_set_uint32(dev, "num-ports", 4);
		qdev_prop_set_taddr(dev, "dma-offset", 0);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1f930000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, iie_irq[8]);

	}
	{
		DeviceState *dev;
		BusState *idebus[4];
		DriveInfo *hd;
		dev = qdev_create(NULL, "sysbus-ahci");
		qdev_prop_set_uint32(dev, "num-ports", 2);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1f940000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, iie_irq[14]);
		hd=drive_get_next(IF_IDE);
		if(hd)
		{
			idebus[0] = qdev_get_child_bus(dev, "ide.0");

			dev = qdev_create(idebus[0], hd->media_cd ? "ide-cd" : "ide-hd");
			qdev_prop_set_uint32(dev, "unit", 0);
			if(!qdev_prop_set_drive(dev, "drive", hd->bdrv))
				qdev_init_nofail(dev);
		}
	}

	if (nb_nics) {
		gmac_sysbus_create(&nd_table[0], 0x1fd90000, iie_irq[10]);
	}


#if 0
	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("iie_spi",0x1fd60000, iie_irq[13]);
		bus = qdev_get_child_bus(dev, "ssi");
		if(flash_dinfo)
		{
			dev1 = ssi_create_slave_no_init(bus, "spi-flash");
			if (qdev_prop_set_drive(dev1, "drive", flash_dinfo->bdrv)) {
				abort();
			}
			qdev_prop_set_uint32(dev1, "size", 0x100000);
			qdev_prop_set_uint64(dev1, "addr", 0x1fc00000);
			qdev_init_nofail(dev1);
		}
		else dev1 = ssi_create_slave(bus, "ssi-sd");
		cs_line = qdev_get_gpio_in(dev1, 0);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1 , cs_line);
	}


#endif

	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_nand");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fd70000);
		sysbus_connect_irq(s, 0, iie_irq[5]);
	}



	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &mips_qemu_ops, (void *)0x1fda0000, "ddr controler", 0x1000);
                memory_region_add_subregion(address_space_mem, 0x1fda0000, iomem);
	}


}

QEMUMachine mips_iie_machine = {
	.name = "iie",
	.desc = "mips iie platform",
	.init = mips_iie_init,
	DEFAULT_MACHINE_OPTIONS,
};

static void mips_machine_init(void)
{
	qemu_register_machine(&mips_iie_machine);
}

machine_init(mips_machine_init);

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

	uint32_t intreg_edge;
	uint32_t intreg_steer;
	uint32_t intreg_pol;
	uint32_t reserve3;

	qemu_irq cpu_irq;
	uint32_t intreg_pending;
} GS232_INTCTLState;

#define INTCTL_SIZE 0x18
#define INTCTLM_MAXADDR 0x13
#define INTCTLM_SIZE (INTCTLM_MAXADDR + 1)
#define INTCTLM_MASK 0x1f
#define MASTER_IRQ_MASK ~0x0fa2007f
#define MASTER_DISABLE 0x80000000
#define CPU_SOFTIRQ_MASK 0xfffe0000
#define CPU_HARDIRQ_MASK 0x0000fffe
#define CPU_IRQ_INT15_IN 0x0004000
#define CPU_IRQ_INT15_MASK 0x80000000

static void iie_check_interrupts(void *opaque);

// per-cpu interrupt controller
static uint64_t iie_intctl_mem_readl(void *opaque, hwaddr addr, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr, ret;

	saddr = addr >> 2;
	switch (saddr) {
		case 6: //rawstatus
			ret = s->inten[0] & ~s->intmask[0];
			break;
		case 7:
			ret = s->inten[1] & ~s->intmask[1];
			break;
		case 8: //maskstatus
			ret = s->inten[0] & ~s->intmask[0];
			break;
		case 9: //maskstatus
			ret = s->inten[1] & ~s->intmask[1];
			break;
		case 10: //forcestatus
			ret = s->inten[0] & ~s->intmask[0];
			break;
		case 11: //forcestatus
			ret = s->inten[1] & ~s->intmask[1];
			break;
		default:
			ret = 0;
			break;
	}
	DPRINTF("read cpu %d reg 0x" TARGET_FMT_plx " = %x\n", cpu, addr, ret);

	return ret;
}

static void iie_intctl_mem_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr;

	saddr = addr >> 2;
	if(saddr>14) return;
	//printf("write reg 0x" TARGET_FMT_plx " %x= %x\n", addr, saddr, (unsigned int)val);
	switch (saddr) {
		case 0: //isr
			//iie_check_interrupts(s);
			break;
		default:
			*(uint32_t *)((void *)s+addr) = val;
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


	pil_pending = s->inten[0] & ~s->intmask[0];

	if (pil_pending ) {
		qemu_irq_raise(s->cpu_irq);
	} else {
			qemu_irq_lower(s->cpu_irq);
	}
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

	DPRINTF("Set cpu %d irq %d level %d\n", s->target_cpu, irq,
			level);
	if (level) {
		s->intreg_pending |= mask;
	} else {
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
