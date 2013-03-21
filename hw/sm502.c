#include "hw.h"
#include "monitor.h"
#include "sysbus.h"
#include "pci.h"
#include "dma.h"
#include "cpu-common.h"

#define PCI_VENDOR_ID_SM502 0x126f
#define PCI_DEVICE_ID_SM502 0x501

void *sm502_new(uint32_t local_mem_bytes, qemu_irq irq, CharDriverState *chr);
void sm502_update_membase(void *opaque,target_phys_addr_t membase);
void sm502_update_iobase(void *opaque,target_phys_addr_t membase);

typedef struct SM502State{
    int mem;
    int ohci_mem;
    target_phys_addr_t *localmem_base;
    qemu_irq irq;
} SM502State;


typedef struct SM502PciState {
    PCIDevice card;
    void *sm502;
    void *serial;
} SM502PciState;



static void sm502_pci_map(PCIDevice *pci_dev, int region_num,
        pcibus_t addr, pcibus_t size, int type)
{
    struct SM502PciState *d = (struct SM502PciState *)pci_dev;

    switch(region_num)
    {
     case 0:
    sm502_update_membase(d->sm502,addr);
    break;
     case 1:
    /*usb ohci*/
    sm502_update_iobase(d->sm502,addr);
    break;
    }
}

int pci_sm502_init(PCIBus *bus,
                 int devfn, CharDriverState *chr)
{
    PCIDevice *dev;

    dev = pci_create(bus, devfn, "sm502");
    qdev_prop_set_ptr(&dev->qdev, "serial", chr);
    qdev_init_nofail(&dev->qdev);

    return 0;
}

static int pci_sm502_initfn(PCIDevice *dev)
{
    struct SM502PciState *d;
    d = DO_UPCAST(struct SM502PciState, card, dev);
    pci_config_set_vendor_id(d->card.config,0x126f);
    pci_config_set_device_id(d->card.config,0x0501);
    d->card.config[PCI_COMMAND] = PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
                                  PCI_COMMAND_MASTER;

    pci_config_set_class(d->card.config,PCI_CLASS_DISPLAY_OTHER);
//    pci_set_byte(&d->card.config[PCI_CLASS_PROG], SM502_PROGMODE_MAJOR_REV_1);

    d->card.config[PCI_CACHE_LINE_SIZE] = 0x08;  /* Cache line size */
    d->card.config[PCI_LATENCY_TIMER]   = 0x00;  /* Latency timer */
    d->card.config[PCI_HEADER_TYPE]     = PCI_HEADER_TYPE_NORMAL;
    d->card.config[PCI_INTERRUPT_PIN]   = 1;     /* interrupt pin 0 */

    pci_register_bar(&d->card, 0, 0x2000000, PCI_BASE_ADDRESS_SPACE_MEMORY,
                     sm502_pci_map);
    pci_register_bar(&d->card, 1, 0x200000,
                           PCI_BASE_ADDRESS_SPACE_MEMORY, sm502_pci_map);
    d->sm502 = sm502_new(0x2000000,d->card.irq[0],d->serial);
    return 0;
}

static PCIDeviceInfo sm502_info = {
    .qdev.name  = "sm502",
    .qdev.size  = sizeof(SM502PciState),
    .init       = pci_sm502_initfn,
    .qdev.props   = (Property[]) {
        DEFINE_PROP_PTR("serial", SM502PciState, serial),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static Property sm502_properties[] = {
    DEFINE_PROP_PTR("serial", SM502PciState, serial),
    DEFINE_PROP_END_OF_LIST(),
};

static void sm502_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = sm502_initfn;
//    k->config_write = sm502_write_config;
    k->vendor_id = PCI_VENDOR_ID_SM502;
    k->device_id = PCI_DEVICE_ID_SM502;
    k->class_id = 0x3800;
    k->revision = 0xc0;
    dc->desc = "sm502 pci card";
    dc->no_user = 1;
    dc->props = sm502_properties;
}

static const TypeInfo sm502_info = {
    .name          = "sm502",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(SM502PciState),
    .class_init    = sm502_class_init,
};

static void sm502_register_types(void)
{
    type_register_static(&sm502_info);
}

type_init(sm502_register_types)
