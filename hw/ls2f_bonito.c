/*
 * ARM Versatile/PB PCI host controller
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licenced under the LGPL.
 */
#include "hw.h"
#include "pci/pci.h"
#include "pc.h"
#include "mips.h"
#include "pci/pci_host.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"

#define gdb_printf(...) //printf

struct bonito_data {
	int	irqnr;
	int	pciirq;
	int 	cur_bus;
	int	cur_dev;
	int	cur_func;

	/* Based at 1fe00000, bonito pci configuration space */
	struct pcicnf {
		uint32_t pcidid;		/* 0 */
		uint32_t pcicmd;		/* 4 */
		uint32_t pciclass;		/* 8 */
		uint32_t pciltimer;		/* c */
		uint32_t pcibase0;		/* 10 */
		uint32_t pcibase1;		/* 14 */
		uint32_t pcibase2;		/* 18 */
		uint32_t resv[5];		/* 1c~2c */
		uint32_t pciexprbase;	/* 30 */
		uint32_t resv1[2];		/* 34,38 */
		uint32_t pciint;		/* 3c */
	}pcicnf;
	/* Based at 1fe00100, bonito local register space */
	struct bonlocal {
		uint32_t bonponcfg;		/* 0 */
		uint32_t bongencfg;		/* 4 */
		uint32_t iodevcfg;		/* 8 */
		uint32_t sdcfg;		/* c */
		uint32_t pcimap;		/* 10 */
		uint32_t pcimembasecfg;	/* 14 */
		uint32_t pcimap_cfg;	/* 18 */
		uint32_t gpiodata;		/* 1c */
		uint32_t gpioie;		/* 20 */
		uint32_t intedge;		/* 24 */
		uint32_t intsteer;		/* 28 */
		uint32_t intpol;		/* 2c */
		uint32_t intenset;		/* 30 */
		uint32_t intenclr;		/* 34 */
		uint32_t inten;		/* 38 */
		uint32_t intisr;		/* 3c */
		uint32_t reg40[(0x100-0x40)/4];		/* 40 */
	}bonlocal;
};

struct bonito_data mybonitodata;
static void bonito_update_irq(void);
CPUMIPSState *bonito_cpu_env;
//------------------------------
static uint64_t ddrcfg_dummy_readl(void *opaque, hwaddr addr, unsigned size)
{
	return -1;
}

static void ddrcfg_dummy_writel(void *opaque, hwaddr addr,
		uint64_t val,unsigned size)
{
}

static const MemoryRegionOps ddrcfg_dummy_ops = {
	.read = ddrcfg_dummy_readl,
	.write = ddrcfg_dummy_writel,
};


//-----------------
static inline uint32_t bonito_pci_config_addr(hwaddr addr)
{
	int bus = 0, dev = -1, func = 0, reg = 0;
	uint32_t tmp, tmp1 ,tmp2;
	int found_idsel = 0;
	struct bonito_data *d=&mybonitodata;

	tmp = addr & 0xfffc;
	tmp1 = ((d->bonlocal.pcimap_cfg & 0xffff) << 16) | tmp;
	if (d->bonlocal.pcimap_cfg & 0x10000) {
		tmp2 = tmp1 >> 24;
		bus = (tmp1 >> 16) & 0xff;
	}
	else {
		tmp2 = tmp1 >> 11;
	}
	if (tmp2 == 0) {
		return 1;
		printf("[ bonpci: PCI configration cannot access without chip selects! ]\n");
	}
	tmp = tmp2;
	while (tmp2)
	{
		if (tmp2 & 0x1) {
			if (found_idsel) {
				return 1;
				printf("[ bonpci: Multi access idsel %x! ]\n", tmp);
			}
			found_idsel = 1;
		}
		tmp2 >>= 1;
		dev++;
	}

	func = (tmp1 >> 8) & 0x7;
	reg = (tmp1) & 0xfc;

	d->cur_bus = bus;
	d->cur_dev = dev;
	d->cur_func = func;

	return bus<<16|dev<<11|func<<8|reg;
}


static void pci_bonito_config_writeb (void *opaque, hwaddr addr,
		uint32_t val)
{
	pci_data_write(opaque, bonito_pci_config_addr (addr), val, 1);
}

static void pci_bonito_config_writew (void *opaque, hwaddr addr,
		uint32_t val)
{
#ifdef TARGET_WORDS_BIGENDIAN
	val = bswap16(val);
#endif
	pci_data_write(opaque, bonito_pci_config_addr (addr), val, 2);
}

static void pci_bonito_config_writel (void *opaque, hwaddr addr,
		uint32_t val)
{
#ifdef TARGET_WORDS_BIGENDIAN
	val = bswap32(val);
#endif
	pci_data_write(opaque, bonito_pci_config_addr (addr), val, 4);
}

static uint32_t pci_bonito_config_readb (void *opaque, hwaddr addr)
{
	uint32_t val;
	val = pci_data_read(opaque, bonito_pci_config_addr (addr), 1);
	return val;
}

static uint32_t pci_bonito_config_readw (void *opaque, hwaddr addr)
{
	uint32_t val;
	val = pci_data_read(opaque, bonito_pci_config_addr (addr), 2);
#ifdef TARGET_WORDS_BIGENDIAN
	val = bswap16(val);
#endif
	return val;
}

static uint32_t pci_bonito_config_readl (void *opaque, hwaddr addr)
{
	uint32_t val;
	val = pci_data_read(opaque, bonito_pci_config_addr (addr), 4);
#ifdef TARGET_WORDS_BIGENDIAN
	val = bswap32(val);
#endif
	return val;
}

static const MemoryRegionOps bonito_pciconf_ops = {
	.old_mmio = {
		.read = {
			&pci_bonito_config_readb,
			&pci_bonito_config_readw,
			&pci_bonito_config_readl,
		},
		.write = {
			&pci_bonito_config_writeb,
			&pci_bonito_config_writew,
			&pci_bonito_config_writel,
		},
	},
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static qemu_irq *pci_bonito_pic;
static int pci_bonito_irq_offset;
static int (*local_board_map_irq)(int bus,int dev,int func,int pin);

static int pci_bonito_map_irq(PCIDevice *d, int irq_num)
{
	int dev=(d->devfn>>3)&0x1f;
	int func=d->devfn& 7;

	return local_board_map_irq(0,dev,func,irq_num);
}

static void bonito_update_irq(void)
{
	struct bonito_data *d=&mybonitodata;
	if(d->bonlocal.intisr&d->bonlocal.inten)
		qemu_set_irq(pci_bonito_pic[0],1);
	else
		qemu_set_irq(pci_bonito_pic[0],0);

}


static void pci_bonito_set_irq(void *opaque, int irq_num, int level)
{
	struct bonito_data *d=&mybonitodata;
	if(level)
		d->bonlocal.intisr |=1<<(pci_bonito_irq_offset +irq_num);
	else
		d->bonlocal.intisr &=~(1<<(pci_bonito_irq_offset+irq_num));
	bonito_update_irq();
	//    qemu_set_irq(pic[pci_bonito_irq + irq_num], level);
}

MemoryRegion *ddrcfg_iomem;

static uint32_t pci_bonito_local_readb (void *opaque, hwaddr addr)
{
	uint32_t val;
	uint32_t relative_addr=addr&0xff;
	struct bonito_data *d=&mybonitodata;

	val = ((uint32_t *)(&d->bonlocal))[relative_addr/sizeof(uint32_t)];
	switch (relative_addr) {
		break;
		case 0x3c:
		val |= ((bonito_cpu_env->CP0_Cause<<1)&0x7800);
		break;
		default:
		break;
	}
	return (val>>(addr&3))&0xff;
}

static uint32_t pci_bonito_local_readw (void *opaque, hwaddr addr)
{
	uint32_t val;
	uint32_t relative_addr=addr&0xff;
	struct bonito_data *d=&mybonitodata;

	val = ((uint32_t *)(&d->bonlocal))[relative_addr/sizeof(uint32_t)];
	switch (relative_addr) {
		case 0x3c: 
			val |= ((bonito_cpu_env->CP0_Cause<<1)&0x7800);
			break;
		default:
			break;
	}
	return (val>>(addr&3))&0xffff;
}

static uint32_t pci_bonito_local_readl (void *opaque, hwaddr addr)
{
	uint32_t val;
	uint32_t relative_addr=addr&0xff;
	struct bonito_data *d=&mybonitodata;

	val = ((uint32_t *)(&d->bonlocal))[relative_addr/sizeof(uint32_t)];
	switch (relative_addr) {
		case 0x3c: 
			val |= ((bonito_cpu_env->CP0_Cause<<1)&0x7800);
			break;
		default:
			break;
	}
	return val;
}

static void pci_bonito_local_writeb (void *opaque, hwaddr addr,
		uint32_t val)
{
	//	uint32_t relative_addr=addr&0xff;
}

static void pci_bonito_local_writew (void *opaque, hwaddr addr,
		uint32_t val)
{
	//	uint32_t relative_addr=addr&0xff;
}

#define 	WAITSTART    	0
#define 	START		1000
#define 	LOWCLK		20
#define	 	HIBEGCLK 	30
#define 	HINEXTCLK 	40
#define 	WAITSEND	50
#define 	INITSEND	55
#define		SENDDATA	60
#define 	ERROR     	100
#define		PPDELAY		10         //parallel port delay
static int gpio_serial(int val)
{
	static int STAT = WAITSTART;
	static int clkp=0,datap=0;
	static int cnt = 8;
	static unsigned char clk = 0;              //bit0 for clk
	static unsigned char data = 0;	//bit1 for data
	static unsigned char cget = 0;			//reorderd data
	static int sflg = 0;
	//static char chin;
	//static int ch;

	{	

		clk = (val>>2)&1;
		data = (val>>3)&1;

		switch (STAT)
		{
			case WAITSTART:
				if ((clkp==1)&&(datap==1)&&(clk==1)&&(data == 0))
				{
					STAT = START;
					if((cnt > 0)&&(cnt < 8) )
					{
						STAT = WAITSTART;
						sflg = 0;
						cnt  = 8;
					}
				}
				break;

			case START:
				cnt = 0;
				cget = 0;
				if (clk == 0)
				{		
					STAT = LOWCLK;
					gdb_printf("stat = lowclk\n");          //debug		
				}		
				break;

			case LOWCLK:
				STAT = HIBEGCLK;
				break;

			case HIBEGCLK:
				if (clk == 0 )
					break;
				data = data << cnt;
				data = data & 0xff;
				cget = cget | data;
				data = data >> cnt;
				data = data & 1;
				cnt++;

				if (cnt == 8)
				{
					if(/*isascii(cget)*/1||cget == 0x80)
					{			
						if(cget == 0x80)
						{

							//outb(0x4,BASEADDR);
							STAT = SENDDATA;
							break;
						}	
						else{
							if(sflg == 1 )
							{	
							}
							if(isascii(cget))
								printf("%c",cget);
							cget = 0;
						}
					}
					else
					{
						STAT = ERROR;
						printf(" ERROR ERROR ERROR char = (%x)H,",cget);
						break;
					}
				}
				STAT = HINEXTCLK;
				break;

			case HINEXTCLK:
				if (clk == 0 )
				{
					STAT = LOWCLK;
					if( cnt == 8 )
						STAT = 	WAITSTART;
				}
				if ((clk == 1) && (data != datap))
				{
					gdb_printf("warning at HINEXTCLK\n"); 		
				}		
				break;

			case WAITSEND:
				//outb(0x4,BASEADDR);

				STAT = WAITSTART;
				break;

			case SENDDATA:
				sflg = 0;
				//palputdata(chin);
				//outb(0,BASEADDR);
				STAT = WAITSTART;
				break;

			case ERROR:
				printf("error\n");
				break;

			default:
				break;
		}
	}

	clkp = clk;
	datap = data;
	return 0;
}



static void pci_bonito_local_writel (void *opaque, hwaddr addr,
		uint32_t val)
{
	uint32_t relative_addr=addr&0xff;
	struct bonito_data *d=&mybonitodata;

	switch (relative_addr) {
		case 0x1c:
			d->bonlocal.gpiodata= (d->bonlocal.gpiodata&0xffff0000)|(val&0xffff);
			gpio_serial(val);
			break;
			break;
		case 0x38:
		case 0x3c:
			printf("[ bonlocal: Access to non-writeable register %08x! ]\n", relative_addr);
			break;
		case 0x30:
			d->bonlocal.inten |= val;
			break;
		case 0x34:
			d->bonlocal.inten &= (~val)|0x7800;
			break;
		case 0x80:
			//ddr config register
			
			if(val&0x100 && !(d->bonlocal.reg40[(0x80-0x40)/4]&0x100))
			{
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);
			}

			if(!(val&0x100) && (d->bonlocal.reg40[(0x80-0x40)/4]&0x100))
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ffffe00&TARGET_PAGE_MASK, ddrcfg_iomem, 1);
			}

			d->bonlocal.reg40[(0x80-0x40)/4] = val;
			break;
		case 0x84:
			break;
		default:
			((uint32_t *)(&d->bonlocal))[relative_addr/sizeof(uint32_t)] = val & 0xffffffff;
			break;
	}
	bonito_update_irq();
}


static const MemoryRegionOps bonito_local_ops = {
	.old_mmio = {
		.read = {
			&pci_bonito_local_readb,
			&pci_bonito_local_readw,
			&pci_bonito_local_readl,
		},
		.write = {
			&pci_bonito_local_writeb,
			&pci_bonito_local_writew,
			&pci_bonito_local_writel,
		},
	},
	.endianness = DEVICE_NATIVE_ENDIAN,
};


PCIBus *pci_bonito_init(CPUMIPSState *env,qemu_irq *pic, int irq,int (*board_map_irq)(int bus,int dev,int func,int pin));
PCIBus *pci_bonito_init(CPUMIPSState *env,qemu_irq *pic, int irq,int (*board_map_irq)(int bus,int dev,int func,int pin))
{
	PCIBus *s;

	bonito_cpu_env = env;
	pci_bonito_irq_offset = irq;
	pci_bonito_pic = pic;
	local_board_map_irq = board_map_irq;

	s = pci_register_bus(NULL, "pci",pci_bonito_set_irq, pci_bonito_map_irq, pic,
			get_system_memory(), get_system_io(), 4<<3, 4);

	{
		MemoryRegion *iomem = g_new(MemoryRegion, 1);
		memory_region_init_io(iomem, &bonito_pciconf_ops, s, "ls2f_pci_conf", 512);
		memory_region_add_subregion(get_system_memory(), 0x1fe80000, iomem);
	}

	{
		MemoryRegion *iomem = g_new(MemoryRegion, 1);
		memory_region_init_io(iomem, &bonito_local_ops, s, "ls1a_pci_conf", 0x100);
		memory_region_add_subregion(get_system_memory(), 0x1fe00100, iomem);
	}

	{
		ddrcfg_iomem = g_new(MemoryRegion, 1);
		memory_region_init_io(ddrcfg_iomem, &ddrcfg_dummy_ops, NULL, "ls1a_pci_conf", TARGET_PAGE_SIZE);
	}

	mybonitodata.bonlocal.intisr=0;
	mybonitodata.bonlocal.inten=0xf<<11;

	/* Register 64 KB of ISA IO space at 0x1fd00000 */
	isa_mmio_init(0x1fd00000, 0x00010000);
	isa_mem_base = 0x10000000;
	return s;
}

