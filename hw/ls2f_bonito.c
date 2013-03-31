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

struct bonito_regs {
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
};


#define TYPE_BONITO_PCI_HOST_BRIDGE "ls2f-pcihost"
typedef struct BonitoState BonitoState;

#define BONITO_PCI_HOST_BRIDGE(obj) \
    OBJECT_CHECK(BonitoState, (obj), TYPE_BONITO_PCI_HOST_BRIDGE)

typedef struct PCIBonitoState
{
	PCIDevice dev;
	BonitoState *pcihost;
	MemoryRegion iomem;
} PCIBonitoState;

struct BonitoState {
    PCIHostState parent_obj;
    qemu_irq *pic;
    PCIBonitoState *pci_dev;
    struct bonito_regs regs;
    CPUMIPSState *cpu_env;
    int irq_offset;
};


static void bonito_update_irq(BonitoState *pcihost);


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

static void bonito_pciconf_writel(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    PCIBonitoState *s = opaque;
    PCIDevice *d = PCI_DEVICE(s);

    d->config_write(d, addr, val, 4);
}

static uint64_t bonito_pciconf_readl(void *opaque, hwaddr addr,
                                     unsigned size)
{

    PCIBonitoState *s = opaque;
    PCIDevice *d = PCI_DEVICE(s);

    return d->config_read(d, addr, 4);
}

/* north bridge PCI configure space. 0x1fe0 0000 - 0x1fe0 00ff */

static const MemoryRegionOps bonito_pciconf_ops = {
    .read = bonito_pciconf_readl,
    .write = bonito_pciconf_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static int (*local_board_map_irq)(int bus,int dev,int func,int pin);

static int pci_bonito_map_irq(PCIDevice *d, int irq_num)
{
	int dev=(d->devfn>>3)&0x1f;
	int func=d->devfn& 7;

	return local_board_map_irq(0,dev,func,irq_num);
}

static void bonito_update_irq(BonitoState *pcihost)
{
	struct bonito_regs *d=&pcihost->regs;
	if(d->intisr&d->inten)
		qemu_set_irq(pcihost->pic[0],1);
	else
		qemu_set_irq(pcihost->pic[0],0);

}


static void pci_bonito_set_irq(void *opaque, int irq_num, int level)
{
    	BonitoState *pcihost = opaque;
	struct bonito_regs *d=&pcihost->regs;
	if(level)
		d->intisr |=1<<(pcihost->irq_offset +irq_num);
	else
		d->intisr &=~(1<<(pcihost->irq_offset+irq_num));
	bonito_update_irq(pcihost);
	//    qemu_set_irq(pic[pci_bonito_irq + irq_num], level);
}

MemoryRegion *ddrcfg_iomem;

static uint32_t pci_bonito_local_readb (void *opaque, hwaddr addr)
{
	uint32_t val;
	uint32_t relative_addr=addr&0xff;
	BonitoState *pcihost = opaque;
	struct bonito_regs *d=&pcihost->regs;

	val = ((uint32_t *)d)[relative_addr/sizeof(uint32_t)];
	switch (relative_addr) {
		break;
		case 0x3c:
		val |= ((pcihost->cpu_env->CP0_Cause<<1)&0x7800);
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
	BonitoState *pcihost = opaque;
	struct bonito_regs *d=&pcihost->regs;

	val = ((uint32_t *)d)[relative_addr/sizeof(uint32_t)];
	switch (relative_addr) {
		case 0x3c: 
			val |= ((pcihost->cpu_env->CP0_Cause<<1)&0x7800);
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
	BonitoState *pcihost = opaque;
	struct bonito_regs *d=&pcihost->regs;

	val = ((uint32_t *)d)[relative_addr/sizeof(uint32_t)];
	switch (relative_addr) {
		case 0x3c: 
			val |= ((pcihost->cpu_env->CP0_Cause<<1)&0x7800);
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
	BonitoState *pcihost = opaque;
	struct bonito_regs *d=&pcihost->regs;

	switch (relative_addr) {
		case 0x1c:
			d->gpiodata= (d->gpiodata&0xffff0000)|(val&0xffff);
			gpio_serial(val);
			break;
			break;
		case 0x38:
		case 0x3c:
			printf("[ bonlocal: Access to non-writeable register %08x! ]\n", relative_addr);
			break;
		case 0x30:
			d->inten |= val;
			break;
		case 0x34:
			d->inten &= (~val)|0x7800;
			break;
		case 0x80:
			//ddr config register
			
			if(val&0x100 && !(d->reg40[(0x80-0x40)/4]&0x100))
			{
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);
			}

			if(!(val&0x100) && (d->reg40[(0x80-0x40)/4]&0x100))
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ffffe00&TARGET_PAGE_MASK, ddrcfg_iomem, 1);
			}

			d->reg40[(0x80-0x40)/4] = val;
			break;
		case 0x84:
			break;
		default:
			((uint32_t *)d)[relative_addr/sizeof(uint32_t)] = val & 0xffffffff;
			break;
	}
	bonito_update_irq(pcihost);
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

//--------------------------------------------
PCIBus *pci_bonito_init(CPUMIPSState *env,qemu_irq *pic, int irq,int (*board_map_irq)(int bus,int dev,int func,int pin))
{
    DeviceState *dev;
    BonitoState *pcihost;
    PCIHostState *phb;
    PCIBonitoState *s;
    PCIDevice *d;

    dev = qdev_create(NULL, TYPE_BONITO_PCI_HOST_BRIDGE);
    phb = PCI_HOST_BRIDGE(dev);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);
    pcihost->pic = pic;
    pcihost->cpu_env = env;
    pcihost->irq_offset = irq;
    local_board_map_irq = board_map_irq;
    qdev_init_nofail(dev);

    /* set the pcihost pointer before bonito_initfn is called */
    d = pci_create(phb->bus, PCI_DEVFN(0, 0), "LS2F_Bonito");
    s = DO_UPCAST(PCIBonitoState, dev, d);
    s->pcihost = pcihost;
    pcihost->pci_dev = s;
    qdev_init_nofail(DEVICE(d));


    return phb->bus;
}

static inline uint32_t bonito_pci_config_addr(PCIBonitoState *s,hwaddr addr)
{
	int bus = 0, dev = -1, func = 0, reg = 0;
	uint32_t busaddr;
	uint32_t pcimap_cfg = s->pcihost->regs.pcimap_cfg;
	busaddr = ((pcimap_cfg & 0xffff) << 16) | (addr & 0xfffc);

	if (pcimap_cfg & 0x10000) {
	bus = busaddr >> 16;
	dev = (busaddr >> 11) & 0x1f;
	func = (busaddr >> 8) & 7;
	reg = busaddr & 0xfc;
	}
	else {
		bus = 0;
		dev = ffs(busaddr>>11)-1;
		func = (busaddr >> 8) & 7;
		reg = busaddr & 0xfc;
	}

	return bus<<16|dev<<11|func<<8|reg;
}

static void pci_ls2f_config_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	PCIBonitoState *s = opaque;
	//   PCIDevice *d = PCI_DEVICE(s);
	PCIHostState *phb = PCI_HOST_BRIDGE(s->pcihost);
	pci_data_write(phb->bus, bonito_pci_config_addr(s, addr), val, size);
}

static uint64_t pci_ls2f_config_readl (void *opaque, hwaddr addr, unsigned size)
{
	PCIBonitoState *s = opaque;
	//  PCIDevice *d = PCI_DEVICE(s);
	PCIHostState *phb = PCI_HOST_BRIDGE(s->pcihost);
	uint32_t val;
	val = pci_data_read(phb->bus, bonito_pci_config_addr(s, addr), size);
	return val;
}


static const MemoryRegionOps pci_ls2f_config_ops = {
    .read = pci_ls2f_config_readl,
    .write = pci_ls2f_config_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static int bonito_initfn(PCIDevice *dev)
{
    PCIBonitoState *s = DO_UPCAST(PCIBonitoState, dev, dev);
    SysBusDevice *sysbus = SYS_BUS_DEVICE(s->pcihost);
    PCIHostState *phb = PCI_HOST_BRIDGE(s->pcihost);

    /* Bonito North Bridge, built on FPGA, VENDOR_ID/DEVICE_ID are "undefined" */
    pci_config_set_prog_interface(dev->config, 0x00);


    memory_region_init_io(&s->iomem, &bonito_local_ops, s->pcihost, "ls2f_pci_local", 0x100);
    sysbus_init_mmio(sysbus, &s->iomem);
    sysbus_mmio_map(sysbus, 0, 0x1fe00100);

    /* set the north bridge pci configure  mapping */
    memory_region_init_io(&phb->conf_mem, &bonito_pciconf_ops, s,
                          "north-bridge-pci-config", 0x100);
    sysbus_init_mmio(sysbus, &phb->conf_mem);
    sysbus_mmio_map(sysbus, 1, 0x1fe00000);

    /* set the south bridge pci configure  mapping */
    memory_region_init_io(&phb->data_mem, &pci_ls2f_config_ops, s,
                          "south-bridge-pci-config", 0x100);
    sysbus_init_mmio(sysbus, &phb->data_mem);
    sysbus_mmio_map(sysbus, 2, 0x1fe80000);

	{
		ddrcfg_iomem = g_new(MemoryRegion, 1);
		memory_region_init_io(ddrcfg_iomem, &ddrcfg_dummy_ops, NULL, "ls2f_pci_conf", TARGET_PAGE_SIZE);
	}

	{
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	memory_region_init_ram(ram, "ls2f_addrconf", 0x4000);
	memory_region_add_subregion(get_system_memory(), 0x3ff00000, ram);
	}

	s->pcihost->regs.intisr=0;
	s->pcihost->regs.inten=0xf<<11;

	/* Register 64 KB of ISA IO space at 0x1fd00000 */
	isa_mmio_init(0x1fd00000, 0x00010000);
	isa_mem_base = 0x10000000;

    /* set the default value of north bridge pci config */
    pci_set_word(dev->config + PCI_COMMAND, 0x0000);
    pci_set_word(dev->config + PCI_STATUS, 0x0000);
    pci_set_word(dev->config + PCI_SUBSYSTEM_VENDOR_ID, 0x0000);
    pci_set_word(dev->config + PCI_SUBSYSTEM_ID, 0x0000);

    pci_set_byte(dev->config + PCI_INTERRUPT_LINE, 0x00);
    pci_set_byte(dev->config + PCI_INTERRUPT_PIN, 0x01);
    pci_set_byte(dev->config + PCI_MIN_GNT, 0x3c);
    pci_set_byte(dev->config + PCI_MAX_LAT, 0x00);


    return 0;
}


static void bonito_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = bonito_initfn;
    k->vendor_id = 0xdf53;
    k->device_id = 0x00d5;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    dc->desc = "Host bridge";
    dc->no_user = 1;
}

static const TypeInfo bonito_info = {
    .name          = "LS2F_Bonito",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCIBonitoState),
    .class_init    = bonito_class_init,
};

static int bonito_pcihost_initfn(SysBusDevice *dev)
{
    BonitoState *pcihost;
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);

    phb->bus = pci_register_bus(DEVICE(dev), "pci",
                                pci_bonito_set_irq, pci_bonito_map_irq, pcihost,
                                get_system_memory(), get_system_io(),
                                12<<3, 4);

    return 0;
}

static void bonito_pcihost_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = bonito_pcihost_initfn;
    dc->no_user = 1;
}

static const TypeInfo bonito_pcihost_info = {
    .name          = TYPE_BONITO_PCI_HOST_BRIDGE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(BonitoState),
    .class_init    = bonito_pcihost_class_init,
};

static void bonito_register_types(void)
{
    type_register_static(&bonito_pcihost_info);
    type_register_static(&bonito_info);
}

type_init(bonito_register_types)

