/*
 * QEMU loongson 1a framebuffer emulation
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
#include "sysbus.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "framebuffer.h"
#include "sysemu/dma.h"

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;

    int width;
    int height;
    int invalidate;
    int depth;
    int bypp;
    int enable;
    int config;
    int need_update;
    void *type;

    DisplayState *ds;
    int vram_size;
    ram_addr_t vram_offset;

    union{
	    MemoryRegion *root;
	    void *root_ptr;
    };

    int index;
    int syncing;
} ls1a_fb_state;


static const VMStateDescription vmstate_ls2h_fb = {
    .name = "ls2h_fb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(width, ls1a_fb_state),
        VMSTATE_END_OF_LIST()
    }
};


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


static inline void ls2h_fb_size(ls1a_fb_state *s)
{
    		s->invalidate = 1;
		qemu_console_resize(s->ds, s->width, s->height);
}

static void ls2h_fb_invalidate_screen(void *opaque)
{
	ls1a_fb_state *s = opaque;
	s->invalidate = 1;
	qemu_console_resize(s->ds, s->width, s->height);
}

static void ls2h_fb_update_screen(void *opaque)
{
    ls1a_fb_state *s = opaque;
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

static void ls2h_fb_reset(ls1a_fb_state *s)
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
	ls2h_fb_size(s);
}


#define LS2H_DC_REG_BASE				0

#define LS2H_FB_CFG_DVO_REG				(LS2H_DC_REG_BASE + 0x1240)
#define LS2H_FB_CFG_VGA_REG				(LS2H_DC_REG_BASE + 0x1250)
#define LS2H_FB_ADDR0_DVO_REG				(LS2H_DC_REG_BASE + 0x1260)
#define LS2H_FB_ADDR0_VGA_REG				(LS2H_DC_REG_BASE + 0x1270)
#define LS2H_FB_STRI_DVO_REG				(LS2H_DC_REG_BASE + 0x1280)
#define LS2H_FB_STRI_VGA_REG				(LS2H_DC_REG_BASE + 0x1290)
#define LS2H_FB_HDISPLAY_DVO_REG			(LS2H_DC_REG_BASE + 0x1400)
#define LS2H_FB_HDISPLAY_VGA_REG			(LS2H_DC_REG_BASE + 0x1410)
#define LS2H_FB_VDISPLAY_VGA_REG			(LS2H_DC_REG_BASE + 0x1490)

static void ls2h_fb_writel (void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
 ls1a_fb_state *s = opaque;

 switch(addr)
 {
  case LS2H_FB_CFG_VGA_REG:
	if(val&0x100)
	{
	  switch(val&7)
	  {
		  case 4:
			  s->depth = 24;
			  break;
		  case 3:
			  s->depth = 16;
			  break;
		  case 2:
			  s->depth = 15;
			  break;
		  default:
			  s->depth = 16;
			  break;
	  }
	  s->need_update = 1;
	ls2h_fb_size(s);
      }
  break;
  case LS2H_FB_HDISPLAY_VGA_REG:
	s->width = val&0xffff;
	s->need_update = 1;
	ls2h_fb_size(s);
	break;
  case LS2H_FB_VDISPLAY_VGA_REG:
	s->height = val&0xffff;
	s->need_update = 1;
	ls2h_fb_size(s);
	break;

  case LS2H_FB_ADDR0_DVO_REG:
  case LS2H_FB_ADDR0_VGA_REG:
  s->vram_offset = val;
  s->need_update = 1;
	ls2h_fb_size(s);
  break;
 }
}

static uint64_t ls2h_fb_readl (void *opaque, hwaddr addr, unsigned size)
{
// ls1a_fb_state *s = opaque;
    return 0;
}

static const MemoryRegionOps ls2h_fb_ops = {
    .read = ls2h_fb_readl,
    .write = ls2h_fb_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int ls2h_fb_sysbus_init(SysBusDevice *dev)
{
    ls1a_fb_state *d = FROM_SYSBUS(ls1a_fb_state, dev);

    memory_region_init_io(&d->iomem, &ls2h_fb_ops, (void *)d, "ls2h fb", 0x10000);

    sysbus_init_mmio(dev, &d->iomem);
    d->vram_size = 0;
    d->vram_offset = 0;
    d->root = sysbus_address_space(dev);
    
    d->ds = graphic_console_init(ls2h_fb_update_screen,
                                 ls2h_fb_invalidate_screen,
                                 NULL, NULL, d);
    ls2h_fb_reset(d);

    return 0;
}

static Property ls2h_fb_properties[] = {
    DEFINE_PROP_PTR("root", ls1a_fb_state, root_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls2h_fb_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2h_fb_sysbus_init;
    dc->desc = "ls2h fb";
    dc->props = ls2h_fb_properties;
}

static const TypeInfo ls2h_fb_sysbus_info = {
    .name          = "ls2h_fb",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls1a_fb_state),
    .class_init    = ls2h_fb_sysbus_class_init,
};


static void ls2h_fb_sysbus_register_types(void)
{
    type_register_static(&ls2h_fb_sysbus_info);
}

type_init(ls2h_fb_sysbus_register_types)
