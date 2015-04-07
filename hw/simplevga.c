/*
 * QEMU VMware-SVGA "chipset".
 *
 * Copyright (c) 2007 Andrzej Zaborowski  <balrog@zabor.org>
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
#include "ui/pixel_ops.h"
#include "ui/console.h"
#include "pci/pci.h"
#include "framebuffer.h"


#undef DIRECT_VRAM


typedef struct simplevga_state_s {

    int width;
    int height;
    int invalidated;
    int depth;
    int bypp;
    int enable;
    int config;
    int need_update;
    void *type;
    DisplayState *ds;
    int vram_size;
    ram_addr_t vram_offset;
    uint8_t *vram_ptr;


    int index;
    int new_width;
    int new_height;
    int syncing;
    int invalidate;

    AddressSpace as;
    MemoryRegion vram;
    MemoryRegion pci_bar0;
    MemoryRegion *root;

} simplevga_state;

struct pci_simplevga_state_s {
    PCIDevice card;
    struct simplevga_state_s chip;

};

#define ls1a_fb_state simplevga_state
#define BITS 8
#include "ls1a_fb_template.h"
#define BITS 15
#include "ls1a_fb_template.h"
#define BITS 16
#include "ls1a_fb_template.h"
#define BITS 24
#include "ls1a_fb_template.h"
#define BITS 32
#include "ls1a_fb_template.h"



static inline void simplevga_size(simplevga_state *s)
{
    		s->invalidate = 1;
		qemu_console_resize(s->ds, s->width, s->height);
}

static void simplevga_invalidate_screen(void *opaque)
{
	simplevga_state *s = opaque;
	s->invalidate = 1;
	qemu_console_resize(s->ds, s->width, s->height);
}

static void simplevga_update_screen(void *opaque)
{
    simplevga_state *s = opaque;
    int first = 0;
    int last = 0;
    drawfn fn;


    int dest_width = s->width;

    switch (ds_get_bits_per_pixel(s->ds)) {
    case 0:
        return;
    case 8:
        fn = draw_line_8;
        break;
    case 15:
        fn = draw_line_15;
        dest_width *= 2;
        break;
    case 16:
        fn = draw_line_16;
        dest_width *= 2;
        break;
    case 24:
        fn = draw_line_24;
        dest_width *= 3;
        break;
    case 32:
        fn = draw_line_32;
        dest_width *= 4;
        break;
    default:
        hw_error("milkymist_vgafb: bad color depth\n");
        break;
    }

    framebuffer_update_display(s->ds, s->root,
                               s->vram_offset,
                               s->width,
                               s->height,
                               s->width * s->bypp,
                               dest_width,
                               0,
                               s->invalidate,
                               fn,
                               s,
                               &first, &last);

    if (first >= 0) {
        dpy_gfx_update(s->ds, 0, first, s->width, last - first + 1);
    }
    s->invalidate = 0;
}

static void simplevga_reset(simplevga_state *s)
{
	int width=800,height=600,depth=16;
	unsigned int fb_offset=0;

	if(getenv("SIMPLEVGA"))
	{
		if(sscanf(getenv("SIMPLEVGA"),"%dx%d-%d:0x%x",&width,&height,&depth,&fb_offset)<3)
		{
			printf("usage: SIMPLEVGA=800x600-16:0xc3000\n");
			exit(1);
		}
	}
	s->index = 0;
	s->need_update = 1;
	s->enable = 0;
	s->config = 0;
	s->width = width;
	s->height = height;
	s->depth = depth;

	s->vram_offset = fb_offset;
	s->bypp = (s->depth + 7) >> 3;
	s->syncing = 0;
	simplevga_size(s);
}




#define PCI_CLASS_BASE_DISPLAY		0x03
#define PCI_CLASS_SUB_VGA		0x00
#define PCI_CLASS_HEADERTYPE_00h	0x00



void pci_simplevga_init(PCIBus *bus,int width,int height,int depth,const char *type);
void pci_simplevga_init(PCIBus *bus,int width,int height,int depth,const char *type)
{
    PCIDevice *dev;

    dev = pci_create(bus, -1, "simplevga");
    qdev_prop_set_int32(&dev->qdev, "width", width);
    qdev_prop_set_int32(&dev->qdev, "height", height);
    qdev_prop_set_int32(&dev->qdev, "depth", depth);
    qdev_prop_set_ptr(&dev->qdev, "type", (void *)type);
    qdev_init_nofail(&dev->qdev);

}





static int pci_simplevga_initfn(PCIDevice *dev)
{
	struct pci_simplevga_state_s *s=DO_UPCAST(struct pci_simplevga_state_s, card, dev);
	int PCI_VENDOR_ID_VGACARD;
	int PCI_DEVICE_ID_VGACARD;
	if(!strcmp(s->chip.type,"sis")){
		PCI_VENDOR_ID_VGACARD=0x1039;
		PCI_DEVICE_ID_VGACARD=0x0315;
	}
	else if(!strcmp(s->chip.type,"sm712"))
	{
		PCI_VENDOR_ID_VGACARD=0x126f;
		PCI_DEVICE_ID_VGACARD=0x0712;
	}
	else if(!strncmp(s->chip.type,"pci:",4))
	{
		unsigned long x=strtoul(s->chip.type+4,0,0);
		PCI_VENDOR_ID_VGACARD=x&0xffff;
		PCI_DEVICE_ID_VGACARD=x>>16;
	}
	else
	{
		PCI_VENDOR_ID_VGACARD=0x126f;
		PCI_DEVICE_ID_VGACARD=0x0712;
	}

	pci_config_set_vendor_id(s->card.config, PCI_VENDOR_ID_VGACARD);
	pci_config_set_device_id(s->card.config, PCI_DEVICE_ID_VGACARD);
	s->card.config[PCI_COMMAND]		= 0x07;		/* I/O + Memory */
	pci_config_set_class(s->card.config, PCI_CLASS_DISPLAY_VGA);
	s->card.config[0x0c]		= 0x08;		/* Cache line size */
	s->card.config[0x0d]		= 0x40;		/* Latency timer */
	s->card.config[PCI_HEADER_TYPE]	= PCI_HEADER_TYPE_NORMAL;
	/* XXX: vga_ram_size must be a power of two */

	memory_region_init_ram(&s->chip.vram, "vga.vram", 0x100000);
    	address_space_init(&s->chip.as, &s->chip.vram);
    	memory_region_init_alias(&s->chip.pci_bar0, "pci bar0", &s->chip.vram, 0, 0x100000);
	s->chip.vram_ptr = memory_region_get_ram_ptr(&s->chip.pci_bar0);
	s->chip.root = &s->chip.vram;


	pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->chip.pci_bar0);

	s->chip.ds = graphic_console_init(simplevga_update_screen, simplevga_invalidate_screen, 0, 0, &s->chip);
	simplevga_reset(&s->chip);
	return 0;
}


static Property simplevga_properties[] = {
        DEFINE_PROP_INT32("width",struct pci_simplevga_state_s , chip.new_width, 0),
        DEFINE_PROP_INT32("height",struct pci_simplevga_state_s, chip.new_height, 0),
        DEFINE_PROP_INT32("depth",struct pci_simplevga_state_s, chip.depth, 0),
	DEFINE_PROP_PTR("type",struct pci_simplevga_state_s,chip.type),
    DEFINE_PROP_END_OF_LIST(),
};

static void simplevga_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->no_hotplug = 1;
    k->init = pci_simplevga_initfn;
    k->vendor_id = 0x1039;
    k->device_id = 0x0315;
    k->class_id = PCI_CLASS_DISPLAY_VGA;
    dc->props = simplevga_properties;
}

static const TypeInfo simplevga_info = {
    .name          = "simplevga",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(struct pci_simplevga_state_s),
    .class_init    = simplevga_class_init,
};

static void simplevga_register_types(void)
{
    type_register_static(&simplevga_info);
}

type_init(simplevga_register_types)

//memory_region_init_ram_ptr(&s->ram_vram, "vram",  PCI6254_MEM_SIZE, s->mapaddr+0x1000);
