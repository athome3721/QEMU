#include "hw.h"
#include "sysbus.h"
#include "pci/pci.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "qemu/range.h"
#include "qdev-addr.h"
#include "serial.h"
#include "net/net.h"
#include "pc.h"
#include "sysemu/blockdev.h"
#include "ssi.h"
#include "i2c.h"
#include "exec/address-spaces.h"

#define PCI_VENDOR_ID_LS1A 0x104a
#define PCI_DEVICE_ID_LS1A 0x0

typedef struct pcilocal{
	unsigned int pcimap;
	unsigned int pcix_rgagte;
	unsigned int pcix_noused;
	unsigned int pcimap_cfg;
} pcilocal;


typedef struct LS1APciState {
	PCIDevice card;
	DMAContext dma_ehci;
	DMAContext dma_ohci;
	DMAContext dma_ahci;
	DMAContext dma_gmac;
	DMAContext dma_nand;
	DMAContext dma_ac97;
	AddressSpace as;
	void *ls1a;
	union{
		CharDriverState **serial_hds;
		void *serial_ptr;
	};
	union{
		DriveInfo *hd;
		void *hd_ptr;
	};
	union{
		DriveInfo *flash;
		void *flash_ptr;
	};
	union{
		NICInfo *nd;
		void *nd_ptr;
	};
	qemu_irq *pin0_irqs, *pin1_irqs;
	uint8_t pin0_irqstat, pin1_irqstat;
	MemoryRegion iomem_root;
	MemoryRegion iomem_dc;
	MemoryRegion iomem_axi;
	MemoryRegion iomem_ddr;
	pcilocal pcilocalreg;
	MemoryRegion iomem_dc0;
	MemoryRegion iomem_axi0;
	MemoryRegion iomem_ddr0;
} LS1APciState;


int pci_ls1a_init(PCIBus *bus, int devfn, CharDriverState **serial, DriveInfo *hd, NICInfo *nd, DriveInfo *flash);
int pci_ls1a_init(PCIBus *bus, int devfn, CharDriverState **serial, DriveInfo *hd, NICInfo *nd, DriveInfo *flash)
{
    PCIDevice *dev;

    dev = pci_create(bus, devfn, "ls1a");
    qdev_prop_set_ptr(&dev->qdev, "serial", serial);
    qdev_prop_set_ptr(&dev->qdev, "hd", hd);
    qdev_prop_set_ptr(&dev->qdev, "nd", nd);
    qdev_prop_set_ptr(&dev->qdev, "flash", flash);
    qdev_init_nofail(&dev->qdev);

    return 0;
}

static void pin0_set_irq(void *opaque, int irq, int level)
{
	struct LS1APciState *d = opaque;
	uint32_t mask = 1 << irq;
	if(level)
	 d->pin0_irqstat |= mask;
	else
	 d->pin0_irqstat &= ~mask;

	if(d->pin0_irqstat)
	  qemu_irq_raise(d->card.irq[0]);
	else
	  qemu_irq_lower(d->card.irq[0]);
}

static void pin1_set_irq(void *opaque, int irq, int level)
{
	struct LS1APciState *d = opaque;
	uint32_t mask = 1 << irq;
	if(level)
	 d->pin1_irqstat |= mask;
	else
	 d->pin1_irqstat &= ~mask;

	if(d->pin1_irqstat)
	  qemu_irq_raise(d->card.irq[1]);
	else
	  qemu_irq_lower(d->card.irq[1]);
}

#include "ls1a_int.c"

static int clkreg[2];
static MemoryRegion *ddrcfg_iomem;
static int reg0420[1]={0x2};
static struct LS1APciState *sbstat;

static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x00e78030:
		case 0x00e78034:
			clkreg[(addr - 0x00e78030)/4] = val;
			break;

		case 0x00d00420:
			memory_region_transaction_begin();
			if(ddrcfg_iomem->parent)
				memory_region_del_subregion(&sbstat->iomem_ddr, ddrcfg_iomem);

			if((val&0x2) == 0)
			{
				memory_region_add_subregion_overlap(&sbstat->iomem_ddr, 0x0ffffe00&TARGET_PAGE_MASK, ddrcfg_iomem, 1);
			}

			reg0420[0] = val;
			memory_region_transaction_commit();

			break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x00e78030:
		case 0x00e78034:
			return clkreg[(addr - 0x00e78030)/4];
			break;
		case 0x00d00420:
			return reg0420[0];
			break;
		case 0x00fffe10:
			return 1;
			break;
		case 0x00fffef0:
			return 0x100000;
			break;
		case 0x00fffef2:
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

static uint64_t pci_bonito_local_readl (void *opaque, hwaddr addr, unsigned size)
{
    LS1APciState *s = opaque;
	uint32_t val;
	uint32_t relative_addr=addr;
	val = ((uint32_t *)&s->pcilocalreg)[relative_addr/sizeof(uint32_t)];
	switch(size)
	{
	case 1:
	val = (val>>(addr&3))&0xff;
	break;
	case 2:
	val = (val>>(addr&3))&0xffff;
	break;
	}
	return val;
}

static void pci_bonito_local_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
    LS1APciState *s = opaque;
	uint32_t relative_addr=addr;
	((uint32_t *)&s->pcilocalreg)[relative_addr/sizeof(uint32_t)]=val;
}

static const MemoryRegionOps pci_bonito_local_ops = {
    .read = pci_bonito_local_readl,
    .write = pci_bonito_local_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static int ls1adma_translate(LS1APciState *s, DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    uint32_t pcimap = s->pcilocalreg.pcimap;
    hwaddr plen = *len;
	//dma_memory_map
    	//dma_memory_rw
        //address_space_rw(dma->as, paddr, buf, plen, dir == DMA_DIRECTION_FROM_DEVICE);
	if(addr >= 0x10000000 && addr < 0x14000000 )
	{
	s->card.dma->translate(s->card.dma, (addr&0x03ffffff)|((pcimap&0x3f)<<26), paddr, len, dir);
	dma->as = s->card.dma->as;
	}
	else if(addr >= 0x14000000 && addr < 0x18000000 )
	{
	s->card.dma->translate(s->card.dma, (addr&0x03ffffff)|(((pcimap>>6)&0x3f)<<26), paddr, len, dir);
	dma->as = s->card.dma->as;
	}
	else if(addr >= 0x18000000 && addr < 0x1c000000 )
	{
	s->card.dma->translate(s->card.dma, (addr&0x03ffffff)|(((pcimap>>12)&0x3f)<<26), paddr, len, dir);
	dma->as = s->card.dma->as;
	}
	else
	{
	dma->as = &s->as;
	*paddr = addr;
	*len = 0x10000000;
	}

	if(plen && *len > plen)
	{
       *len = plen;
	}


    return 0;
}

static int ls1adma_ehci_translate(DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    LS1APciState *s = container_of (dma, LS1APciState, dma_ehci);
    return ls1adma_translate(s, dma, addr, paddr, len, dir);
}

static int ls1adma_ohci_translate(DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    LS1APciState *s = container_of (dma, LS1APciState, dma_ohci);
    return ls1adma_translate(s, dma, addr, paddr, len, dir);
}


static int ls1adma_ahci_translate(DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    LS1APciState *s = container_of (dma, LS1APciState, dma_ahci);
    return ls1adma_translate(s, dma, addr, paddr, len, dir);
}

static int ls1adma_gmac_translate(DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    LS1APciState *s = container_of (dma, LS1APciState, dma_gmac);
    return ls1adma_translate(s, dma, addr, paddr, len, dir);
}


static int ls1adma_nand_translate(DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    LS1APciState *s = container_of (dma, LS1APciState, dma_nand);
    return ls1adma_translate(s, dma, addr, paddr, len, dir);
}

static int ls1adma_ac97_translate(DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    LS1APciState *s = container_of (dma, LS1APciState, dma_ac97);
    return ls1adma_translate(s, dma, addr, paddr, len, dir);
}


static int ls1a_initfn(PCIDevice *dev)
{
    struct LS1APciState *d;
    qemu_irq *ls1a_irq,*ls1a_irq1;
    sbstat = d = DO_UPCAST(struct LS1APciState, card, dev);
    pci_config_set_vendor_id(d->card.config, PCI_VENDOR_ID_LS1A);
    pci_config_set_device_id(d->card.config, PCI_DEVICE_ID_LS1A);
    d->card.config[PCI_COMMAND] = PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
                                  PCI_COMMAND_MASTER;

    pci_config_set_class(d->card.config,PCI_CLASS_DISPLAY_OTHER);
//    pci_set_byte(&d->card.config[PCI_CLASS_PROG], SM502_PROGMODE_MAJOR_REV_1);

    d->card.config[PCI_CACHE_LINE_SIZE] = 0x08;  /* Cache line size */
    d->card.config[PCI_LATENCY_TIMER]   = 0x00;  /* Latency timer */
    d->card.config[PCI_HEADER_TYPE]     = PCI_HEADER_TYPE_NORMAL;
    d->card.config[PCI_INTERRUPT_PIN]   = 1;     /* interrupt pin 0 */


    memory_region_init(&d->iomem_dc, "ls1a_dc", 0x200000);
    memory_region_init(&d->iomem_axi, "ls1a_axi", 0x1000000);
    memory_region_init(&d->iomem_ddr, "ls1a_ddr", 0x40000000);
    {
	    MemoryRegion *ram = g_new(MemoryRegion, 1);
	    memory_region_init_ram(ram, "mips_r4k.ram", 0x10000000);
	    vmstate_register_ram_global(ram);
	    memory_region_add_subregion(&d->iomem_ddr, 0, ram);
    }

    memory_region_init_alias(&d->iomem_dc0, "ls1a_dc0", &d->iomem_dc, 0, 0x00200000);
    memory_region_init_alias(&d->iomem_axi0, "ls1a_axi0", &d->iomem_axi, 0, 0x00e80000);
    memory_region_init_alias(&d->iomem_ddr0, "ls1a_ddr0", &d->iomem_ddr, 0, 0x10000000);


    memory_region_init(&d->iomem_root, "system", INT32_MAX);
    address_space_init(&d->as, &d->iomem_root);
    address_space_memory.name = "ls1a memory";
    dma_context_init(&d->dma_ehci, &d->as, ls1adma_ehci_translate, NULL, NULL);
    dma_context_init(&d->dma_ohci, &d->as, ls1adma_ohci_translate, NULL, NULL);
    dma_context_init(&d->dma_ahci, &d->as, ls1adma_ahci_translate, NULL, NULL);
    dma_context_init(&d->dma_gmac, &d->as, ls1adma_gmac_translate, NULL, NULL);
    dma_context_init(&d->dma_nand, &d->as, ls1adma_nand_translate, NULL, NULL);
    dma_context_init(&d->dma_ac97, &d->as, ls1adma_ac97_translate, NULL, NULL);

    memory_region_add_subregion(&d->iomem_root, 0x1c200000, &d->iomem_dc0);
    memory_region_add_subregion(&d->iomem_root, 0x1f000000, &d->iomem_axi0);
    memory_region_add_subregion(&d->iomem_root, 0x00000000, &d->iomem_ddr0);

    d->pin0_irqs = qemu_allocate_irqs(pin0_set_irq, d, 2);
    d->pin1_irqs = qemu_allocate_irqs(pin1_set_irq, d, 3);

    ls1a_irq =ls1a_intctl_init(&d->iomem_axi, 0x00d01040, d->pin0_irqs[0]);
    ls1a_irq1=ls1a_intctl_init(&d->iomem_axi, 0x00d01058, d->pin0_irqs[1]);
    ls1a_intctl_init(&d->iomem_axi, 0x00d01070, d->pin1_irqs[0]);
    ls1a_intctl_init(&d->iomem_axi, 0x00d01088, d->pin1_irqs[1]);

    printf("serial_hds[0]=%p\n", d->serial_hds[0]);

    if (d->serial_hds[0])
	    serial_mm_init(&d->iomem_axi, 0x00e40000, 2,ls1a_irq[2],115200,d->serial_hds[0], DEVICE_NATIVE_ENDIAN);

    if (d->serial_hds[1])
	    serial_mm_init(&d->iomem_axi, 0x00e44000, 2,ls1a_irq[3],115200,d->serial_hds[1], DEVICE_NATIVE_ENDIAN);

    if (d->serial_hds[2])
	    serial_mm_init(&d->iomem_axi, 0x00e48000, 2,ls1a_irq[4],115200,d->serial_hds[2], DEVICE_NATIVE_ENDIAN);

    if (d->serial_hds[3])
	    serial_mm_init(&d->iomem_axi, 0x00e4c000, 2,ls1a_irq[5],115200,d->serial_hds[3], DEVICE_NATIVE_ENDIAN);

    {
	    MemoryRegion *i8042 = g_new(MemoryRegion, 1);
	    i8042_mm_init(ls1a_irq[12], ls1a_irq[11], i8042, 0x20, 0x10);
	    memory_region_add_subregion(&d->iomem_axi, 0x00e60000, i8042);
    }
	{
		DeviceState *dev;
		SysBusDevice *sysbusdev;
		hwaddr devaddr =  0x00101240;
		dev = sysbus_create_simple("ls1a_fb", -1, NULL);
		sysbusdev =  SYS_BUS_DEVICE(dev);
		sysbusdev->mmio[0].addr = devaddr;
		qdev_prop_set_ptr(dev, "root", &d->iomem_root);
		memory_region_add_subregion(&d->iomem_dc, devaddr, sysbusdev->mmio[0].memory);
	}

	{
		DeviceState *dev;
		SysBusDevice *sysbusdev;
		hwaddr devaddr =  0x00e00000;
		dev = qdev_create(NULL, "exynos4210-ehci-usb");
		qdev_prop_set_ptr(dev, "dma", &d->dma_ehci);
		qdev_init_nofail(dev);

		sysbusdev =  SYS_BUS_DEVICE(dev);
		sysbusdev->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, sysbusdev->mmio[0].memory);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a_irq1[0]);
	}

	{
		DeviceState *dev;
		SysBusDevice *sysbusdev;
		hwaddr devaddr =  0x00e08000;
		dev = qdev_create(NULL, "sysbus-ohci");
		qdev_prop_set_uint32(dev, "num-ports", 4);
		qdev_prop_set_taddr(dev, "dma-offset", 0);
		qdev_prop_set_ptr(dev, "dma", &d->dma_ohci);
		qdev_init_nofail(dev);

		sysbusdev =  SYS_BUS_DEVICE(dev);
		sysbusdev->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, sysbusdev->mmio[0].memory);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a_irq1[1]);
	}

	{
		DeviceState *dev;
		SysBusDevice *sysbusdev;
		hwaddr devaddr =  0x00e30000;
		BusState *idebus[4];
		dev = qdev_create(NULL, "sysbus-ahci");
		qdev_prop_set_uint32(dev, "num-ports", 2);
		qdev_prop_set_ptr(dev, "dma", &d->dma_ahci);
		qdev_init_nofail(dev);

		sysbusdev =  SYS_BUS_DEVICE(dev);
		sysbusdev->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, sysbusdev->mmio[0].memory);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a_irq1[4]);

		//hd=drive_get_next(IF_IDE);
		if(d->hd)
		{
			idebus[0] = qdev_get_child_bus(dev, "ide.0");

			dev = qdev_create(idebus[0], d->hd->media_cd ? "ide-cd" : "ide-hd");
			qdev_prop_set_uint32(dev, "unit", 0);
			if(!qdev_prop_set_drive(dev, "drive", d->hd->bdrv))
				qdev_init_nofail(dev);
		}
	}

	{
		DeviceState *dev;
		SysBusDevice *sysbusdev;
		hwaddr devaddr =  0x00e7c000;
		dev = qdev_create(NULL, "ls1a_acpi");
		qdev_init_nofail(dev);

		sysbusdev =  SYS_BUS_DEVICE(dev);
		sysbusdev->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, sysbusdev->mmio[0].memory);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a_irq[0]);
	}


	if (d->nd) {
		DeviceState *dev;
		SysBusDevice *sysbusdev;
		hwaddr devaddr =  0x00e10000;
		dev = qdev_create(NULL, "sysbus-synopgmac");
		qdev_set_nic_properties(dev, d->nd);
		qdev_prop_set_ptr(dev, "dma", &d->dma_gmac);
		qdev_init_nofail(dev);

		sysbusdev =  SYS_BUS_DEVICE(dev);
		sysbusdev->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, sysbusdev->mmio[0].memory);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a_irq1[2]);
	}

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x00e7c060;
		dev=sysbus_create_simple("ls1a_wdt", -1, NULL);
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
	}
#endif

	{
		DeviceState *dev,*dev1;
		SysBusDevice *sysbusdev;
		hwaddr devaddr =  0x00e80000;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi", -1, ls1a_irq[8]);
		sysbusdev =  SYS_BUS_DEVICE(dev);
		sysbusdev->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, sysbusdev->mmio[0].memory);

		bus = qdev_get_child_bus(dev, "ssi");
		if(d->flash)
		{
			dev1 = ssi_create_slave_no_init(bus, "spi-flash");
			if (qdev_prop_set_drive(dev1, "drive", d->flash->bdrv)) {
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

	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x00e74000;
		dev=sysbus_create_varargs("ls1a_ac97", -1, ls1a_irq[14], ls1a_irq[15], NULL);
		qdev_prop_set_ptr(dev, "dma", &d->dma_ac97);
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
	}

	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x00d01160;
		dev=sysbus_create_simple("ls1a_dma", -1, NULL);
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
	}

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x00e78000;
		dev = qdev_create(NULL, "ls1a_nand");
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		qdev_prop_set_ptr(dev, "dma", &d->dma_nand);
		qdev_init_nofail(dev);
		sysbus_connect_irq(s, 0, ls1a_irq[13]);
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
	}
#endif

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x00e64020;
		dev=sysbus_create_varargs("ls1a_rtc", -1, ls1a_irq[26], ls1a_irq[27], ls1a_irq[28], NULL);
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
	}
#endif

	{
		DeviceState *dev;
		SysBusDevice *s;
		void *bus;
		hwaddr devaddr =  0x00e58000;
		dev = qdev_create(NULL, "ls1a_i2c");
		qdev_prop_set_int32(dev, "shift", 2);
		s = SYS_BUS_DEVICE(dev);
		qdev_init_nofail(dev);
		sysbus_connect_irq(s, 0, ls1a_irq[17]);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x00e50000;
		dev=sysbus_create_simple("ls1a_can", -1, ls1a_irq[6]);
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
	}

	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x00e54000;
		dev=sysbus_create_simple("ls1a_can", -1, ls1a_irq[7]);
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(&d->iomem_axi, devaddr, s->mmio[0].memory);
	}
#endif
	{
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, &mips_qemu_ops, (void *)(0x0ffffe00&TARGET_PAGE_MASK), "ddr", TARGET_PAGE_SIZE);
		mips_qemu_writel((void *)0x00d00420, 0, 0x0, 4);
	}

	{
	 MemoryRegion *pcilocal_iomem = g_new(MemoryRegion, 1);
	 memory_region_init_io(pcilocal_iomem, &pci_bonito_local_ops, d, "ls1a_pci_conf", 16);
	 memory_region_add_subregion(&d->iomem_axi, 0x00d01114, pcilocal_iomem);
	}

    pci_register_bar(&d->card, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->iomem_dc);
    pci_register_bar(&d->card, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->iomem_axi);
    pci_register_bar(&d->card, 4, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->iomem_ddr);
    return 0;
}


static Property ls1a_properties[] = {
    DEFINE_PROP_PTR("serial", LS1APciState, serial_ptr),
    DEFINE_PROP_PTR("hd", LS1APciState, hd_ptr),
    DEFINE_PROP_PTR("nd", LS1APciState, nd_ptr),
    DEFINE_PROP_PTR("flash", LS1APciState, flash_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls1a_write_config(PCIDevice *dev, uint32_t addr,
                                      uint32_t val, int l)
{
    //struct LS1APciState *d;
    //d = DO_UPCAST(struct LS1APciState, card, dev);

    pci_default_write_config(dev, addr, val, l);

}

static void ls1a_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = ls1a_initfn;
    k->config_write = ls1a_write_config;
    k->vendor_id = PCI_VENDOR_ID_LS1A;
    k->device_id = PCI_DEVICE_ID_LS1A;
    k->class_id = 0x3800;
    k->revision = 0xc0;
    dc->desc = "ls1a pci card";
    dc->no_user = 1;
    dc->props = ls1a_properties;
}

static const TypeInfo ls1a_info = {
    .name          = "ls1a",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(LS1APciState),
    .class_init    = ls1a_class_init,
};

static void ls1a_register_types(void)
{
    type_register_static(&ls1a_info);
}

type_init(ls1a_register_types)
