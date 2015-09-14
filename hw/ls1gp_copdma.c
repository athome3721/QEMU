#include "hw.h"
#include "sysbus.h"
#define DPRINTF(a...) printf(a)
struct ls1gp_cop_global_regs
{
	volatile unsigned int control;
	volatile unsigned int rd_inten;
	volatile unsigned int wr_inten;
	volatile unsigned int rd_intisr;		/* offset 0x10*/
	volatile unsigned int wr_intisr;
	unsigned int unused[11];
} __attribute__((packed)) ; 

struct ls1gp_cop_channel_regs
{
	volatile unsigned int rd_control;
	volatile unsigned int rd_src;
	volatile unsigned int rd_cnt;
	volatile unsigned int rd_status;		/* offset 0x10*/
	volatile unsigned int wr_control;
	volatile unsigned int wr_src;
	volatile unsigned int wr_cnt;
	volatile unsigned int wr_status;		/* offset 0x10*/
} __attribute__((packed)); 

struct ls1gp_cop_regs
{
	struct ls1gp_cop_global_regs global;
	struct ls1gp_cop_channel_regs chan[8][2];
} __attribute__((packed));

typedef struct DmaState{
	int dma;
	int irq;
	struct ls1gp_cop_regs regs;
	int dma_ptr[8];
} DmaState;

typedef struct dma_sysbus_state {
	SysBusDevice busdev;
	DmaState dma;
} dma_sysbus_state;


enum {
GLOBAL_CONTROL = 0,
GLOBAL_RD_INTEN = 4 ,
GLOBAL_WR_INTEN = 8,
GLOBAL_RD_INTISR = 12,
GLOBAL_WR_INTISR = 16
};

enum {
CHANNEL_RD_CONTROL = 0,
CHANNEL_RD_SRC = 4,
CHANNEL_RD_CNT = 8,
CHANNEL_RD_STATUS = 12,		
CHANNEL_WR_CONTROL = 16,
CHANNEL_WR_SRC = 20,
CHANNEL_WR_CNT = 24,
CHANNEL_WR_STATUS= 28,		
};

void update_irq(DmaState *s)
{
	if(s->regs.global.rd_inten & s->regs.global.rd_intisr || s->regs.global.wr_inten & s->regs.global.wr_intisr)
	  qemu_irq_raise(s->irq);
	else qemu_irq_lower(s->irq);
}

static void check_dma(DmaState *s,int channel)
{
	int ptr;
	ptr = s->dma_ptr[channel];
	if(s->regs.chan[channel][ptr].wr_control & s->regs.chan[channel][ptr].rd_control & 1)
	{
	int i;
	int count = (s->regs.chan[channel][ptr].rd_cnt + 1)*16;
	char *buf;
	buf = malloc(count);
	cpu_physical_memory_read(s->regs.chan[channel][ptr].rd_src,buf,count);
	cpu_physical_memory_write(s->regs.chan[channel][ptr].wr_src,buf,count);
	free(buf);

	s->regs.global.rd_intisr |= (1<<channel);
	s->regs.global.wr_intisr |= (1<<channel);
	s->regs.chan[channel][ptr].wr_control = 0;
	s->regs.chan[channel][ptr].rd_control = 0;
	update_irq(s);
	s->dma_ptr[channel] = ptr ^ 1;
	check_dma(s,channel);
	}
}

static uint32_t dma_dma_readl(void *ptr, hwaddr addr, unsigned size)
{
	DmaState *s = ptr;
	uint32_t val = 0;
	int channel;
	int slot;
	int reg;
	addr &= (TARGET_PAGE_SIZE - 1);
	channel = (addr>>6) & 7;
	slot = (addr >> 5) & 1;
	reg = addr & 0x1f;
	if(channel !=0)
	{
		channel--;

		switch(reg)
		{
			case CHANNEL_RD_STATUS:
			case CHANNEL_WR_STATUS:
				return s->dma_ptr[channel]<<16;
			default:
				val = *(uint32_t *)((void *)&s->regs + addr);
				break;
		}

	}
	else
		val = *(uint32_t *)((void *)&s->regs + addr);

	DPRINTF("dma_dma_readl:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);
	return val;
}



static void dma_dma_writel(void *ptr, hwaddr addr, uint64_t val, unsigned size)
{
	DmaState *s = ptr;
	addr &= (TARGET_PAGE_SIZE - 1);
	int channel;
	int slot;
	int reg;
	channel = (addr>>6) & 7;
	slot = (addr >> 5) & 1;
	reg = addr & 0x1f;
	
	if(channel == 0) /* global register */
	{

		switch(addr)
		{
			case GLOBAL_CONTROL:
			 memset(&s->regs,0,sizeof(s->regs));
			 break;	
			case GLOBAL_RD_INTISR:
			 s->regs.global.rd_intisr &= ~val;
			 update_irq(s);
			 break;
			case GLOBAL_WR_INTISR:
			 s->regs.global.wr_intisr &= ~val;
			 update_irq(s);
			 break;
			default :
				*(uint32_t *)((void *)&s->regs + addr) = val;
				break;
		}
	}
	else
	{
		channel = channel -1;
		switch(reg)
		{
			case CHANNEL_RD_CONTROL:
				if(val & 1)
				{
					if(channel < 3)
					{
						s->regs.chan[channel][slot].rd_control = 1;
						check_dma(s,channel);
					}
					else
					{

						s->regs.global.rd_intisr |= (1<<channel);
						update_irq(s);
					}
				}
				break;
			case CHANNEL_WR_CONTROL: 
				if(val & 1)
				{
					if(channel < 3)
					{
					s->regs.chan[channel][slot].wr_control = 1;
					check_dma(s,channel);
					}
					else
					{
						s->regs.global.wr_intisr |= (1<<channel);
						update_irq(s);
					}
				}
				break;
			default :
				*(uint32_t *)((void *)&s->regs.chan[channel][slot] + reg) = val;
			break;
		}

	}


	DPRINTF("dma_dma_writel:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);
}


static const MemoryRegionOps dma_ops = {
    .read = dma_dma_readl,
    .write = dma_dma_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int dma_sysbus_init(SysBusDevice *dev)
{
    dma_sysbus_state *d = FROM_SYSBUS(dma_sysbus_state, dev);
    memory_region_init_io(&d->dma, &dma_ops, (void *)d, "dma", 0x40*9);
    sysbus_init_irq(dev, &d->dma.irq);

    sysbus_init_mmio(dev, &d->dma);

    return 0;
}

static void dma_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = dma_sysbus_init;
    dc->desc = "ls1gp copdma";
}

static const TypeInfo dma_sysbus_info = {
    .name          = "ls1gp_copdma",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(dma_sysbus_state),
    .class_init    = dma_sysbus_class_init,
};


static void dma_sysbus_register_types(void)
{
    type_register_static(&dma_sysbus_info);
}

type_init(dma_sysbus_register_types)

