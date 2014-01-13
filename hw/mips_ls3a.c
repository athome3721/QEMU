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
#include "loongson_bootparam.h"

#ifdef TARGET_WORDS_BIGENDIAN
#define BIOS_FILENAME "mips_bios.bin"
#else
#define BIOS_FILENAME "mipsel_bios.bin"
#endif

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };

extern FILE *logfile;
static void *boot_params_buf;
static void *boot_params_p;
#define align(x) (((x)+15)&~15)


/* i8254 PIT is attached to the IRQ0 at PIC i8259 */

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
    target_ulong a0,a1,a2;
} loaderparams;


#include "loongson3a_rom.h"

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

static int set_bootparam1(ram_addr_t initrd_offset,long initrd_size)
{
	char memenv[32];
	char highmemenv[32];
	long params_size;
	void *params_buf;
	unsigned int *parg_env;
	int ret;

	/* Store command line.  */
	params_size = 0x100000;
	params_buf = g_malloc(params_size);

	parg_env=(void *)params_buf;

	/*
	 * pram buf like this:
	 *argv[0] argv[1] 0 env[0] env[1] ...env[i] ,0, argv[0]'s data , argv[1]'s data ,env[0]'data,...,env[i]'s dat,0
	 */

	//jump over argv and env area
	ret =(3+1)*4;
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

	sprintf(memenv,"%d",loaderparams.ram_size>0x10000000?256:(loaderparams.ram_size>>20));
	sprintf(highmemenv,"%d",loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0);
	setenv("memsize", memenv, 1);
	setenv("highmemsize", highmemenv, 1);

	ret = ((ret+32)&~31);

	boot_params_buf = (void *)(params_buf+ret);
	boot_params_p = boot_params_buf + align(sizeof(struct boot_params));

	init_boot_param(boot_params_buf);
	printf("param len=%ld\n", boot_params_p-params_buf);

	rom_add_blob_fixed("params", params_buf, params_size,
			BOOTPARAM_PHYADDR);
	loaderparams.a0 = 2;
	loaderparams.a1 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR;
	loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR + ret;
        printf("env %x\n", BOOTPARAM_PHYADDR + ret);
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


	if(!getenv("LOONGSONENV"))
	 set_bootparam(initrd_offset, initrd_size);
	else
	 set_bootparam1(initrd_offset, initrd_size);
	
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

#define MAX_CPUS 4
static CPUMIPSState *mycpu[4];

#define CORE0_STATUS_OFF       0x000
#define CORE0_EN_OFF           0x004
#define CORE0_SET_OFF          0x008
#define CORE0_CLEAR_OFF        0x00c
#define CORE0_BUF_20           0x020
#define CORE0_BUF_28           0x028
#define CORE0_BUF_30           0x030
#define CORE0_BUF_38           0x038

#define CORE1_STATUS_OFF       0x100
#define CORE1_EN_OFF           0x104
#define CORE1_SET_OFF          0x108
#define CORE1_CLEAR_OFF        0x10c
#define CORE1_BUF_20           0x120
#define CORE1_BUF_28           0x128
#define CORE1_BUF_30           0x130
#define CORE1_BUF_38           0x138

#define CORE2_STATUS_OFF       0x200
#define CORE2_EN_OFF           0x204
#define CORE2_SET_OFF          0x208
#define CORE2_CLEAR_OFF        0x20c
#define CORE2_BUF_20           0x220
#define CORE2_BUF_28           0x228
#define CORE2_BUF_30           0x230
#define CORE2_BUF_38           0x238

#define CORE3_STATUS_OFF       0x300
#define CORE3_EN_OFF           0x304
#define CORE3_SET_OFF          0x308
#define CORE3_CLEAR_OFF        0x30c
#define CORE3_BUF_20           0x320
#define CORE3_BUF_28           0x328
#define CORE3_BUF_30           0x330
#define CORE3_BUF_38           0x338

#define MYID 0xa0

typedef struct gipi_single {
    uint32_t status;
    uint32_t en;
    uint32_t set;
    uint32_t clear;
    uint32_t buf[8];
    qemu_irq irq;
} gipi_single;

typedef struct gipiState  gipiState;
struct gipiState {
	gipi_single core[8];
} ;


static void gipi_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    gipiState * s = opaque;

    int node = (addr>>44)&3;
	int coreno = (addr>>8)&3;
	int no = coreno+node*4;
    

//    printf("gipi_writel addr=%llx val=%8x\n", addr, val);
    addr &= 0xff;
    switch(addr){
        case CORE0_STATUS_OFF: 
            hw_error("CORE0_SET_OFF Can't be write\n");
            break;
        case CORE0_EN_OFF:
		if((cpu_single_env->mem_io_vaddr&0xff)!=addr) break;
            s->core[no].en = val;
            break;
        case CORE0_SET_OFF:
            s->core[no].status |= val;
            qemu_irq_raise(s->core[no].irq);
            break;
        case CORE0_CLEAR_OFF:
		if((cpu_single_env->mem_io_vaddr&0xff)!=addr) break;
            s->core[no].status ^= val;
            qemu_irq_lower(s->core[no].irq);
            break;
        case 0x20 ... 0x3c:
            s->core[no].buf[(addr-0x20)/4] = val;
            break;
        default:
            break;
       }
    //printf("gipi_write: NODE#%d addr=0x%02x val=0x%02x, opaque=%p\n", node, addr, val, opaque);
}

static uint64_t gipi_readl(void *opaque, hwaddr addr, unsigned size)
{
    gipiState * s = opaque;
    uint32_t ret=0;
    int node = (addr>>44)&3;
	int coreno = (addr>>8)&3;
	int no = coreno+node*4;
    addr &= 0xff;

    switch(addr){
        case CORE0_STATUS_OFF: 
            ret =  s->core[no].status;
            break;
        case CORE0_EN_OFF:
            ret =  s->core[no].en;
            break;
        case CORE0_SET_OFF:
            hw_error("CORE0_SET_OFF Can't be Read\n");
            break;
        case CORE0_CLEAR_OFF:
            hw_error("CORE0_CLEAR_OFF Can't be Read\n");
        case 0x20 ... 0x3c:
            ret = s->core[no].buf[(addr-0x20)/4];
            break;
        default:
            break;
       }

    //if(ret!=0) printf("CPU#%d:gipi_read: NODE#%d addr=%p val=0x%02x pc=%x, opaque=%p\n",cpu_single_env->cpu_index, node, addr, ret, cpu_single_env->active_tc.PC, opaque);
    //if(ret!=0) printf("CPU#%d:gipi_read: addr=0x%02x val=0x%02x pc=%x\n",cpu_single_env->cpu_index, addr, ret, cpu_single_env->PC[0]);
    return ret;
}



static const MemoryRegionOps gipi_ops = {
    .read = gipi_readl,
    .write = gipi_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};



MemoryRegion *gipi_iomem;

static int godson_ipi_init(qemu_irq parent_irq , unsigned long index, gipiState * s)
{
  int mysize=0x1000;

      s->core[index].irq = parent_irq;

  if((index==0)||(index==4)) {
      memory_region_add_subregion(get_system_memory(), (0x3ff01000|((unsigned long long)(index/4) << 44)) + (index % 4)*mysize, gipi_iomem);
  }
return 0;
}

static int board_map_irq(int bus,int dev,int func,int pin)
{
int irq_num;
if(dev>=12)
irq_num=((dev-12)+pin)%4;
else irq_num=pin;
    return irq_num;
}

static void ls3a_serial_set_irq(void *opaque, int irq, int level)
{
	int i;
	for(i=0;i<smp_cpus;i++)
	qemu_set_irq(mycpu[i]->irq[2],level);
}

PCIBus *pci_ls3a_init(qemu_irq *pic, int (*board_map_irq)(int bus,int dev,int func,int pin));
static void *ls3a_intctl_init(ISABus *isa_bus, CPUMIPSState *env[]);
static const int sector_len = 32 * 1024;
static void mips_ls3a_init (QEMUMachineInitArgs *args)
{
	ram_addr_t ram_size = args->ram_size;
	const char *cpu_model = args->cpu_model;
	const char *kernel_filename = args->kernel_filename;
	const char *kernel_cmdline = args->kernel_cmdline;
	const char *initrd_filename = args->initrd_filename;
	char *filename;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	MemoryRegion *lowram = g_new(MemoryRegion, 1);
	MemoryRegion *bios;
	int bios_size;
	MIPSCPU *cpu;
	CPUMIPSState *env;
	ResetData *reset_info[10];
	qemu_irq *i8259,*ls3a_serial_irq ;
	DriveInfo *hd[MAX_IDE_BUS * MAX_IDE_DEVS];
	int be;
	DriveInfo *dinfo=NULL;
	int i;


    /* init CPUs */
    if (cpu_model == NULL) {
        cpu_model = "godson3";
    }
    /* init CPUs */
{
  gipiState * gipis =g_malloc0(sizeof(gipiState));
  gipi_iomem = g_new(MemoryRegion, 1);
  memory_region_init_io(gipi_iomem, &gipi_ops, (void *)gipis, "gipi", 0x1000);

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

    godson_ipi_init(env->irq[6] , i, gipis);  // by zxh&dw
  }
  env = mycpu[0];

}

	memory_region_init_ram(ram, "mips_ls3a.ram", ram_size);
        memory_region_init_alias(lowram, "mips_ls3a.lowram", ram, 0, MIN(ram_size,256*0x100000));
	vmstate_register_ram_global(ram);

    /* allocate RAM */
	memory_region_add_subregion(address_space_mem, 0, lowram);

	memory_region_add_subregion(address_space_mem, 0x80000000, ram);


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


    ISABus *isa_bus;

    isa_bus = isa_bus_new(NULL, get_system_io());
    /* Register 64 KB of ISA IO space at 0x14000000 */
    isa_mmio_init(0xefdfc000000LL, 0x00010000);
    isa_mem_base = 0x10000000;

	i8259 = ls3a_intctl_init(isa_bus, mycpu);
	pci_ls3a_init(&i8259[3],board_map_irq);

    /* The PIC is attached to the MIPS CPU INT0 pin */
    isa_bus_irqs(isa_bus, i8259);
    rtc_init(isa_bus, 2000, NULL);

	ls3a_serial_irq = qemu_allocate_irqs(ls3a_serial_set_irq, mycpu, 1);

    if (serial_hds[0])
            serial_mm_init(address_space_mem, 0x1fe001e0, 0,ls3a_serial_irq[0],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);
    if (serial_hds[1])
		serial_isa_init(isa_bus, 0, serial_hds[1]);


	if (nb_nics) {
    for(i = 0; i < nb_nics; i++) {
        NICInfo *nd = &nd_table[i];
	char devaddr[10];

        if (i == 0 && (!nd->model || strcmp(nd->model, "pcnet") == 0))
            /* The malta board has a PCNet card using PCI SLOT 11 */
	sprintf(devaddr,"%x",12+i);

        pci_nic_init_nofail(nd, "e1000", devaddr);
    }
	}


    if (drive_get_max_bus(IF_IDE) >= MAX_IDE_BUS) {
        fprintf(stderr, "qemu: too many IDE bus\n");
        exit(1);
    }

    for(i = 0; i < MAX_IDE_BUS * MAX_IDE_DEVS; i++) {
         hd[i] = drive_get(IF_IDE, i / MAX_IDE_DEVS, i % MAX_IDE_DEVS);
    }


    for(i = 0; i < MAX_IDE_BUS; i++)
        isa_ide_init(isa_bus, ide_iobase[i], ide_iobase2[i], ide_irq[i],
                     hd[MAX_IDE_DEVS * i],
		     hd[MAX_IDE_DEVS * i + 1]);

    isa_create_simple(isa_bus, "i8042");
{
char *p;
MemoryRegion *iomem;
        iomem = g_new(MemoryRegion, 1);
	memory_region_init_ram(iomem, "mips_r4k.ram", 0x1000);
	memory_region_add_subregion(address_space_mem, 0x3ff00000, iomem);
  	p = memory_region_get_ram_ptr(iomem);

	memset(p,0,0x200);
	*(long long *)(p+0x8) = 0x10000000;
	*(long long *)(p+0x40) = 0xfffffffff0000000;
	*(long long *)(p+0x48) = 0xfffffffff0000000;
	*(long long *)(p+0x80) = 0xf0;
	*(long long *)(p+0x88) = 0x100000f2;
	*(long long *)(p+0x100) = 0x80000000;
	*(long long *)(p+0x140) = 0xffffffff80000000;
	*(long long *)(p+0x180) = 0xf0;

        iomem = g_new(MemoryRegion, 1);
	memory_region_init_ram(iomem, "mips_r4k.ram", 0x1000);
	memory_region_add_subregion(address_space_mem, 0x3ff02000, iomem);
  	p = memory_region_get_ram_ptr(iomem);
	memset(p,0,0x800);
}
}

QEMUMachine mips_godson_machine = {
    .name = "ls3a",
    .desc = "Godson3 Multicore platform",
    .init = mips_ls3a_init,
    .max_cpus = 255,
    DEFAULT_MACHINE_OPTIONS,
};

static void mips_machine_init(void)
{
    qemu_register_machine(&mips_godson_machine);
}

machine_init(mips_machine_init);
//-------------------------

#include "hw.h"
#include "pci/pci.h"
#include "pci/pci_host.h"

//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define DPRINTF(fmt, args...) \
do { printf("IRQ: " fmt , ##args); } while (0)
#else
#define DPRINTF(fmt, args...)
#endif


#define MAX_PILS 16


#define HT_CONTROL_REGS_BASE   0xefdfb000000LL
#define HT1LO_PCICFG_BASE      0xefdfe000000LL
#define HT1LO_PCICFG_BASE_TP1  0xefdff000000LL
#define HT1LO_PCICFG_BASE_ALIAS      0x1a000000
#define INT_ROUTER_REGS_BASE   0x3ff01400


#define INT_ROUTER_REGS_SYS_INT0	0x00
#define INT_ROUTER_REGS_SYS_INT1	0x01
#define INT_ROUTER_REGS_SYS_INT2	0x02
#define INT_ROUTER_REGS_SYS_INT3	0x03
#define INT_ROUTER_REGS_PCI_INT0	0x04
#define INT_ROUTER_REGS_PCI_INT1	0x05
#define INT_ROUTER_REGS_PCI_INT2	0x06
#define INT_ROUTER_REGS_PCI_INT3	0x07
#define INT_ROUTER_REGS_MATRIX_INT0	0x08
#define INT_ROUTER_REGS_MATRIX_INT1	0x09
#define INT_ROUTER_REGS_LPC_INT		0x0a
#define INT_ROUTER_REGS_MC0		0x0B
#define INT_ROUTER_REGS_MC1		0x0C
#define INT_ROUTER_REGS_BARRIER		0x0d
#define INT_ROUTER_REGS_RESERVE		0x0e
#define INT_ROUTER_REGS_PCI_PERR	0x0f

#define INT_ROUTER_REGS_HT0_INT0	0x10
#define INT_ROUTER_REGS_HT0_INT1	0x11
#define INT_ROUTER_REGS_HT0_INT2	0x12
#define INT_ROUTER_REGS_HT0_INT3	0x13
#define INT_ROUTER_REGS_HT0_INT4	0x14
#define INT_ROUTER_REGS_HT0_INT5	0x15
#define INT_ROUTER_REGS_HT0_INT6	0x16
#define INT_ROUTER_REGS_HT0_INT7	0x17
#define INT_ROUTER_REGS_HT1_INT0	0x18
#define INT_ROUTER_REGS_HT1_INT1	0x19
#define INT_ROUTER_REGS_HT1_INT2	0x1a
#define INT_ROUTER_REGS_HT1_INT3	0x1b
#define INT_ROUTER_REGS_HT1_INT4	0x1c
#define INT_ROUTER_REGS_HT1_INT5	0x1d
#define INT_ROUTER_REGS_HT1_INT6	0x1e
#define INT_ROUTER_REGS_HT1_INT7	0x1f
#define IO_CONTROL_REGS_INTISR  	0x20
#define IO_CONTROL_REGS_INTEN		0x24	
#define IO_CONTROL_REGS_INTENSET	0x28	
#define IO_CONTROL_REGS_INTENCLR	0x2c	
#define IO_CONTROL_REGS_INTEDGE		0x38	
#define IO_CONTROL_REGS_CORE0_INTISR	0x40	
#define IO_CONTROL_REGS_CORE1_INTISR	0x48	
#define IO_CONTROL_REGS_CORE2_INTISR	0x50	
#define IO_CONTROL_REGS_CORE3_INTISR	0x58	





#define HT_LINK_CONFIG_REG  0x44
#define HT_IRQ_VECTOR_REG0	0x80	
#define HT_IRQ_VECTOR_REG1	0x84	
#define HT_IRQ_VECTOR_REG2	0x88	
#define HT_IRQ_VECTOR_REG3	0x8C	
#define HT_IRQ_VECTOR_REG4	0x90	
#define HT_IRQ_VECTOR_REG5	0x94	
#define HT_IRQ_VECTOR_REG6	0x98	
#define HT_IRQ_VECTOR_REG7	0x9C	

#define HT_IRQ_ENABLE_REG0	0xA0	
#define HT_IRQ_ENABLE_REG1	0xA4	
#define HT_IRQ_ENABLE_REG2	0xA8	
#define HT_IRQ_ENABLE_REG3	0xAC	
#define HT_IRQ_ENABLE_REG4	0xB0	
#define HT_IRQ_ENABLE_REG5	0xB4	
#define HT_IRQ_ENABLE_REG6	0xB8	
#define HT_IRQ_ENABLE_REG7	0xBC	

#define HT_UNCACHE_ENABLE_REG0	0xF0
#define HT_UNCACHE_BASE_REG0	0xF4
#define HT_UNCACHE_ENABLE_REG1	0xF8
#define HT_UNCACHE_BASE_REG1	0xFC



typedef struct LS3a_INTCTLState {
	unsigned char int_route_reg[0x100];
	unsigned char ht_irq_reg[0x100];
 	CPUMIPSState **env;
#ifdef DEBUG_IRQ_COUNT
    uint64_t irq_count[32];
#endif
    uint32_t pil_out[MAX_CPUS];
} LS3a_INTCTLState;

typedef struct LS3a_func_args {
 LS3a_INTCTLState *state;
 uint64_t base;
 uint32_t mask;
 uint8_t *mem;
} LS3a_func_args;


static void ht_update_irq(void *opaque,int disable);

// per-cpu interrupt controller
static uint32_t ls3a_intctl_mem_readb(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
    uint32_t ret;

	addr &= a->mask;

	ret=*(uint8_t *)(a->mem+addr);

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static uint32_t ls3a_intctl_mem_readw(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
    uint32_t ret;

	addr &= a->mask;

	ret=*(uint16_t *)(a->mem+addr);

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static uint32_t ls3a_intctl_mem_readl(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
    uint32_t ret;
	static int linkcfg=0;

	addr &= a->mask;

	switch(a->base+addr)
	{
	case HT_CONTROL_REGS_BASE+HT_LINK_CONFIG_REG:
		ret = linkcfg;
		linkcfg =random();
		//printf("ret=%x\n",ret);
		break;
	case HT_CONTROL_REGS_BASE+HT_IRQ_VECTOR_REG0:
	ret = (cpu_inb(0x20)&~cpu_inb(0x21)) | ((cpu_inb(0xa0)&~cpu_inb(0xa1)) << 8);
	break;
	default:
	ret=*(uint32_t *)(a->mem+addr);
	}

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static void ls3a_intctl_mem_writeb(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;
	LS3a_INTCTLState *s = a->state;

	addr &= a->mask;

	switch(a->base+addr)
	{
	case INT_ROUTER_REGS_BASE+INT_ROUTER_REGS_HT1_INT0 ... INT_ROUTER_REGS_BASE+INT_ROUTER_REGS_HT1_INT7:
	{
		uint32_t old;
		old = *(uint8_t *)(a->mem + addr);
		if (old != val)	
		 ht_update_irq(s,1);
		*(uint8_t *)(a->mem + addr) = val;
		 ht_update_irq(s,0);
	}
	break;
	case HT_CONTROL_REGS_BASE+HT_IRQ_VECTOR_REG0:
	*(uint8_t *)(a->mem + addr) &= ~val;
	ht_update_irq(s,0);
	break;

	case HT_CONTROL_REGS_BASE+HT_IRQ_ENABLE_REG0 ... HT_CONTROL_REGS_BASE+HT_IRQ_ENABLE_REG7:
	*(uint8_t *)(a->mem + addr) = val;
	ht_update_irq(s,0);
	break;
		
	default:
	*(uint8_t *)(a->mem + addr) = val;
	break;
	}
}

static void ls3a_intctl_mem_writew(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;

	addr &= a->mask;
	*(uint16_t *)(a->mem + addr) = val;
}

static void ls3a_intctl_mem_writel(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;
	LS3a_INTCTLState *s = a->state;

	addr &= a->mask;
//	printf("base=%llx,addr=%llx,mask=%x,val=%08x\n",a->base,addr,a->mask,val);
	switch(a->base+addr)
	{
	case INT_ROUTER_REGS_BASE+INT_ROUTER_REGS_HT1_INT0:
	{
		uint32_t old;
		old = *(uint32_t *)(a->mem + addr);
		if (old != val)	
		 ht_update_irq(s,1);
		*(uint32_t *)(a->mem + addr) = val;
		 ht_update_irq(s,0);
		
	}
	break;
	case HT_CONTROL_REGS_BASE+HT_IRQ_VECTOR_REG0:
	*(uint32_t *)(a->mem + addr) &= ~val;
	ht_update_irq(s,0);
	break;

	case HT_CONTROL_REGS_BASE+HT_IRQ_ENABLE_REG0:
	*(uint32_t *)(a->mem + addr) = val;
	ht_update_irq(s,0);
	break;
		
	default:
	*(uint32_t *)(a->mem + addr) = val;
	break;
	}
}

static const MemoryRegionOps ls3a_intctl_ops = {
    .old_mmio = {
	    .read = {
		    ls3a_intctl_mem_readb,
		    ls3a_intctl_mem_readw,
		    ls3a_intctl_mem_readl,
	    },
	    .write = {
		    ls3a_intctl_mem_writeb,
		    ls3a_intctl_mem_writew,
		    ls3a_intctl_mem_writel,
	    }
	}
	,
		.endianness = DEVICE_NATIVE_ENDIAN,
};


static void ht_update_irq(void *opaque,int disable)
{
	LS3a_INTCTLState *s = opaque;
	uint32_t isr,ier,irtr,core,irq_nr;
	isr = *(uint32_t *)(s->ht_irq_reg+HT_IRQ_VECTOR_REG0);
	ier = *(uint32_t *)(s->ht_irq_reg+HT_IRQ_ENABLE_REG0);

	irtr = *(uint8_t *)(s->int_route_reg+INT_ROUTER_REGS_HT1_INT0);
	
	core = irtr&0xf;
	irq_nr = ((irtr>>4)&0xf);

	if(core>0 && irq_nr>0)
	{
	if(isr&ier && !disable)
		qemu_irq_raise(s->env[ffs(core)-1]->irq[ffs(irq_nr)+1]);
	else
		qemu_irq_lower(s->env[ffs(core)-1]->irq[ffs(irq_nr)+1]);
	}
}

static void ht_set_irq(void *opaque, int irq, int level)
{
	LS3a_INTCTLState *s = opaque;
	uint32_t isr;
	if(irq == 0)
	{
	if (level)
	 isr = (cpu_inb(0x20)&~cpu_inb(0x21)) | ((cpu_inb(0xa0)&~cpu_inb(0xa1)) << 8);
    else 
	 isr = 0;
	*(uint32_t *)(s->ht_irq_reg+HT_IRQ_VECTOR_REG0) = isr; 
	}

	ht_update_irq(opaque,0);
}

static void *ls3a_intctl_init(ISABus *isa_bus,CPUMIPSState *env[])
{
	qemu_irq *i8259,*ht_irq;
	LS3a_INTCTLState *s;
	LS3a_func_args *a_irqrouter,*a_htirq;

	s = g_malloc0(sizeof(LS3a_INTCTLState));
	if (!s)
		return NULL;

	a_irqrouter=g_malloc0(sizeof(LS3a_func_args));
	a_htirq=g_malloc0(sizeof(LS3a_func_args));
	a_irqrouter->state = s;
	a_irqrouter->base = INT_ROUTER_REGS_BASE;
	a_irqrouter->mem = s->int_route_reg;
	a_irqrouter->mask = 0xff;
	a_htirq->state = s;
	a_htirq->base = HT_CONTROL_REGS_BASE;
	a_htirq->mem = s->ht_irq_reg;
	a_htirq->mask = 0xff;

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &ls3a_intctl_ops, a_irqrouter, "ls3a_intctl", 256);
                memory_region_add_subregion(get_system_memory(), INT_ROUTER_REGS_BASE, iomem);
	}

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &ls3a_intctl_ops, a_htirq, "ls3a_intctl", 256);
                memory_region_add_subregion(get_system_memory(), HT_CONTROL_REGS_BASE, iomem);
	}


	s->env = env;

	ht_irq = qemu_allocate_irqs(ht_set_irq, s, 8);
	i8259 = i8259_init(isa_bus, ht_irq[0]); 
	cpu_outb(0x4d0,0xff);
	cpu_outb(0x4d1,0xff);



    return i8259;
}
//-------------------------
// pci bridge
//-----------------



static qemu_irq *pci_ls3a_pic;
static int (*local_board_map_irq)(int bus,int dev,int func,int pin);

static int pci_ls3a_map_irq(PCIDevice *d, int irq_num)
{
int dev=(d->devfn>>3)&0x1f;
int func=d->devfn& 7;

return local_board_map_irq(0,dev,func,irq_num);
}

static void pci_ls3a_set_irq(void *opaque, int irq_num, int level)
{
qemu_irq *pic = opaque;
 qemu_set_irq(pic[irq_num],level);
}


static void pci_ls3a_config_writeb (void *opaque, hwaddr addr,
                                   uint32_t val)
{

	if(addr&0x1000000)addr &= 0xffff;
	else addr &= 0xffffff;

    pci_data_write(opaque, addr , val, 1);
}

static void pci_ls3a_config_writew (void *opaque, hwaddr addr,
                                   uint32_t val)
{
	if(addr&0x1000000)addr &= 0xffff;
	else addr &= 0xffffff;

    pci_data_write(opaque, addr , val, 2);
}

static void pci_ls3a_config_writel (void *opaque, hwaddr addr,
                                   uint32_t val)
{
	if(addr&0x1000000)addr &= 0xffff;
	else addr &= 0xffffff;

    pci_data_write(opaque, addr, val, 4);
}

static uint32_t pci_ls3a_config_readb (void *opaque, hwaddr addr)
{
	uint32_t val;

	if(addr&0x1000000)addr &= 0xffff;
	else addr &= 0xffffff;

    val = pci_data_read(opaque, addr , 1);
    return val;
}

static uint32_t pci_ls3a_config_readw (void *opaque, hwaddr addr)
{
	uint32_t val;

	if(addr&0x1000000)addr &= 0xffff;
	else addr &= 0xffffff;

    val = pci_data_read(opaque, addr, 2);
    return val;
}

static uint32_t pci_ls3a_config_readl (void *opaque, hwaddr addr)
{
	uint32_t val;

	if(addr&0x1000000)addr &= 0xffff;
	else addr &= 0xffffff;

    val = pci_data_read(opaque, addr, 4);

    return val;
}

static const MemoryRegionOps pci_ls3a_config_ops = {
	.old_mmio = {
		.read = {
			&pci_ls3a_config_readb,
			&pci_ls3a_config_readw,
			&pci_ls3a_config_readl,
		},
		.write = {
			&pci_ls3a_config_writeb,
			&pci_ls3a_config_writew,
			&pci_ls3a_config_writel,
		}
	}
		,
			.endianness = DEVICE_NATIVE_ENDIAN,
};

PCIBus *pci_ls3a_init(qemu_irq *pic, int (*board_map_irq)(int bus,int dev,int func,int pin))
{
	PCIBus *s;
	MemoryRegion *iomem = g_new(MemoryRegion, 1);
	MemoryRegion *iomem1 = g_new(MemoryRegion, 1);

	pci_ls3a_pic = pic;
	local_board_map_irq = board_map_irq;

	s = pci_register_bus(NULL,"pci",pci_ls3a_set_irq, pci_ls3a_map_irq, pic, get_system_memory(), get_system_io(), 1<<3, 4);

	memory_region_init_io(iomem, &pci_ls3a_config_ops, s, "ls3a_pci_conf", 0x2000000);
        memory_region_init_alias(iomem1, "ls3a_pci_conf", iomem, 0, 0x2000000);
	memory_region_add_subregion(get_system_memory(), HT1LO_PCICFG_BASE, iomem);
	memory_region_add_subregion(get_system_memory(), HT1LO_PCICFG_BASE_ALIAS, iomem1);


	return s;
}

#define LOONGSON_3ASINGLE
#include "loongson_bootparam.c"
