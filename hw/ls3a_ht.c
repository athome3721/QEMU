#include "hw.h"
#include "pci/pci.h"
#include "pc.h"
#include "mips.h"
#include "pci/pci_host.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"
#include "qemu/range.h"
#define MAX_CPUS 4

#define gdb_printf(...) //printf
#define DPRINTF(...)

#define HT_CONTROL_REGS_BASE   0x0fdfb000000LL
#define HT1LO_PCICFG_BASE      0x0fdfe000000LL
#define HT1LO_PCICFG_BASE_TP1  0x0fdff000000LL
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

struct ls3bonito_regs {
		uint32_t bonponcfg;		/* 0 */
		uint32_t bongencfg;		/* 4 */
		uint32_t iodevcfg;		/* 8 */
		uint32_t sdcfg;		/* c */
		uint32_t pcimap;		/* 10 */
		uint32_t PCIX_Bridge_Cfg;	/* 14 */
		uint32_t pcimap_cfg;	/* 18 */
		uint32_t gpiodata;		/* 1c */
		uint32_t gpioie;		/* 20 */
		uint32_t reg40[(0x100-0x40)/4];		/* 40 */
};

#define TYPE_BONITO_PCI_HOST_BRIDGE "ls3aht-pcihost"
typedef struct LS3BonitoState LS3BonitoState;

#define BONITO_PCI_HOST_BRIDGE(obj) \
    OBJECT_CHECK(LS3BonitoState, (obj), TYPE_BONITO_PCI_HOST_BRIDGE)

typedef struct PCILS3BonitoState
{
	PCIDevice dev;
	LS3BonitoState *pcihost;
	MemoryRegion iomem;
} PCILS3BonitoState;

struct LS3BonitoState {
    PCIHostState pcie;
    PCIHostState pci;
    qemu_irq *pic;
    PCILS3BonitoState *pci_dev;
    struct ls3bonito_regs regs;
    CPUMIPSState *cpu_env;
    MemoryRegion iomem_pciemem;
    MemoryRegion iomem_pcimem;
    MemoryRegion iomem_pcimap[3];
    MemoryRegion iomem_pcieio;
    MemoryRegion iomem_pciio;
    int irq_offset;
    union{
    uint8_t xbar2cfg_mem[0x200];
    struct {
    uint64_t cpubase[8]; 
    uint64_t cpumask[8]; 
    uint64_t cpumap[8]; 
    uint64_t dummy[8]; 
    uint64_t pcidmabase[8]; 
    uint64_t pcidmamask[8]; 
    uint64_t pcidmamap[8]; 
    uint64_t dummy1[8]; 
    } xbar2cfg_reg;
    };

    union{
    uint8_t xbar1cfg_mem[0x800];
    struct {
    uint64_t base[8]; 
    uint64_t mask[8]; 
    uint64_t map[8]; 
    uint64_t dummy[8]; 
	   }  xbar1cfg_reg[8];
    };
    union {
    struct{
    uint8_t sys_int[4];
    uint8_t pci_int[4];
    uint8_t matrix_int[2];
    uint8_t lpc_int;
    uint8_t mc[2];
    uint8_t barrier;
    uint8_t rsv;
    uint8_t pci_perr;
    uint8_t ht0_int[8];
    uint8_t ht1_int[8];
    uint32_t intisr;
     };

    unsigned char int_route_reg[0x100];
    };
    unsigned char ht_irq_reg[0x100];
    CPUMIPSState **env;

    DMAContext dma;
    MemoryRegion *ram;
    MemoryRegion iomem_xbar2[8];
    MemoryRegion iomem_xbar1[8];
    MemoryRegion iomem_lowio;
    int pci_introut[3];
};


static void ls3bonito_update_irq(LS3BonitoState *pcihost);


/* dummy ddr config ops */
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

/*address space config ops*/
static uint64_t ls3a_xbar1_readl(void *opaque, hwaddr addr, unsigned size)
{
    	LS3BonitoState *pcihost = opaque;
        switch(size)
	{	
	case 1:
	return *(uint8_t *)(pcihost->xbar1cfg_mem+addr);
	case 2:
	return *(uint16_t *)(pcihost->xbar1cfg_mem+addr);
	case 4:
	return *(uint32_t *)(pcihost->xbar1cfg_mem+addr);
	default:
	return *(uint64_t *)(pcihost->xbar1cfg_mem+addr);
	}
	
}

void update_xbar1map(LS3BonitoState *pcihost);

static void ls3a_xbar1_writel(void *opaque, hwaddr addr,
		uint64_t val,unsigned size)
{
    	LS3BonitoState *pcihost = opaque;

	/*no 0x3ff02400~0x3ff025ff reg*/
	if(addr>=0x400 && addr<=0x5ff)  return;

        switch(size)
	{	
	case 1:
	*(uint8_t *)(pcihost->xbar1cfg_mem+addr) = val;
	break;
	case 2:
	 *(uint16_t *)(pcihost->xbar1cfg_mem+addr) = val;
	break;
	case 4:
	*(uint32_t *)(pcihost->xbar1cfg_mem+addr) = val;
	break;
	case 8:
	*(uint64_t *)(pcihost->xbar1cfg_mem+addr) = val;
	break;
	}

	update_xbar1map(pcihost);
}

static uint64_t ls3a_xbar2_readl(void *opaque, hwaddr addr, unsigned size)
{
    	LS3BonitoState *pcihost = opaque;
        switch(size)
	{	
	case 1:
	return *(uint8_t *)(pcihost->xbar2cfg_mem+addr);
	case 2:
	return *(uint16_t *)(pcihost->xbar2cfg_mem+addr);
	case 4:
	return *(uint32_t *)(pcihost->xbar2cfg_mem+addr);
	default:
	return *(uint64_t *)(pcihost->xbar2cfg_mem+addr);
	}
	
}

void update_xbar2map(LS3BonitoState *pcihost);

static void ls3a_xbar2_writel(void *opaque, hwaddr addr,
		uint64_t val,unsigned size)
{
    	LS3BonitoState *pcihost = opaque;
        switch(size)
	{	
	case 1:
	*(uint8_t *)(pcihost->xbar2cfg_mem+addr) = val;
	break;
	case 2:
	 *(uint16_t *)(pcihost->xbar2cfg_mem+addr) = val;
	break;
	case 4:
	*(uint32_t *)(pcihost->xbar2cfg_mem+addr) = val;
	break;
	case 8:
	*(uint64_t *)(pcihost->xbar2cfg_mem+addr) = val;
	break;
	}

	update_xbar2map(pcihost);
}

static const MemoryRegionOps ls3a_xbar1_ops = {
	.read = ls3a_xbar1_readl,
	.write = ls3a_xbar1_writel,
};

static const MemoryRegionOps ls3a_xbar2_ops = {
	.read = ls3a_xbar2_readl,
	.write = ls3a_xbar2_writel,
};

//-------------------------------------
static void ls3bonito_pciheader_writel(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    LS3BonitoState *s = opaque;
    PCIDevice *d = &s->pci_dev->dev;

    d->config_write(d, addr, val, 4);
}

static uint64_t ls3bonito_pciheader_readl(void *opaque, hwaddr addr,
                                     unsigned size)
{
    LS3BonitoState *s = opaque;
    PCIDevice *d = &s->pci_dev->dev;

    return d->config_read(d, addr, 4);
}

/* north bridge PCI configure space. 0x1fe0 0000 - 0x1fe0 00ff */

static const MemoryRegionOps ls3bonito_pciheader_ops = {
    .read = ls3bonito_pciheader_readl,
    .write = ls3bonito_pciheader_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static int (*local_board_map_irq)(int bus,int dev,int func,int pin);

static int pci_ls3bonito_map_irq(PCIDevice *d, int irq_num)
{
	int dev=(d->devfn>>3)&0x1f;
	int func=d->devfn& 7;

	return local_board_map_irq(0,dev,func,irq_num);
}

static void ls3bonito_update_irq(LS3BonitoState *pcihost)
{
	struct ls3bonito_regs *d=&pcihost->regs;
	if(d->intisr&d->inten)
		qemu_set_irq(pcihost->pic[0],1);
	else
		qemu_set_irq(pcihost->pic[0],0);

}


static void pci_ls3bonito_set_irq(void *opaque, int irq_num, int level)
{
    	LS3BonitoState *pcihost = opaque;
	struct ls3bonito_regs *d=&pcihost->regs;
	int_route_reg;
	if(level)
		d->intisr |=1<<(pcihost->irq_offset +irq_num);
	else
		d->intisr &=~(1<<(pcihost->irq_offset+irq_num));

	env[i]->irq
	ls3bonito_update_irq(pcihost);
	//    qemu_set_irq(pic[pci_ls3bonito_irq + irq_num], level);
}

MemoryRegion *ddrcfg_iomem;

static uint32_t pci_ls3bonito_local_readb (void *opaque, hwaddr addr)
{
	uint32_t val;
	uint32_t relative_addr=addr&0xff;
	LS3BonitoState *pcihost = opaque;
	struct ls3bonito_regs *d=&pcihost->regs;

	val = ((uint32_t *)d)[relative_addr/sizeof(uint32_t)];
	return (val>>(addr&3))&0xff;
}

static uint32_t pci_ls3bonito_local_readw (void *opaque, hwaddr addr)
{
	uint32_t val;
	uint32_t relative_addr=addr&0xff;
	LS3BonitoState *pcihost = opaque;
	struct ls3bonito_regs *d=&pcihost->regs;

	val = ((uint32_t *)d)[relative_addr/sizeof(uint32_t)];
	return (val>>(addr&3))&0xffff;
}

static uint32_t pci_ls3bonito_local_readl (void *opaque, hwaddr addr)
{
	uint32_t val;
	uint32_t relative_addr=addr&0xff;
	LS3BonitoState *pcihost = opaque;
	struct ls3bonito_regs *d=&pcihost->regs;

	val = ((uint32_t *)d)[relative_addr/sizeof(uint32_t)];
	return val;
}

static void pci_ls3bonito_local_writeb (void *opaque, hwaddr addr,
		uint32_t val)
{
	//	uint32_t relative_addr=addr&0xff;
}

static void pci_ls3bonito_local_writew (void *opaque, hwaddr addr,
		uint32_t val)
{
	//	uint32_t relative_addr=addr&0xff;
}

static void update_pcimap(LS3BonitoState *pcihost)
{
	struct ls3bonito_regs *d=&pcihost->regs;
	uint32_t pcimap = d->pcimap;

	memory_region_transaction_begin();
	if(pcihost->iomem_pcimap[0].parent)
	memory_region_del_subregion(get_system_memory(), &pcihost->iomem_pcimap[0]);
	if(pcihost->iomem_pcimap[1].parent)
	memory_region_del_subregion(get_system_memory(), &pcihost->iomem_pcimap[1]);
	if(pcihost->iomem_pcimap[2].parent)
	memory_region_del_subregion(get_system_memory(), &pcihost->iomem_pcimap[2]);

        memory_region_init_alias(&pcihost->iomem_pcimap[0], "pcimem0", &pcihost->iomem_pcimem, (pcimap&0x3f)<<26, 0x4000000);
        memory_region_init_alias(&pcihost->iomem_pcimap[1], "pcimem1", &pcihost->iomem_pcimem, ((pcimap>>6)&0x3f)<<26, 0x4000000);
        memory_region_init_alias(&pcihost->iomem_pcimap[2], "pcimem2", &pcihost->iomem_pcimem, ((pcimap>>12)&0x3f)<<26, 0x4000000);

	memory_region_add_subregion(get_system_memory(), 0x10000000, &pcihost->iomem_pcimap[0]);
	memory_region_add_subregion(get_system_memory(), 0x14000000, &pcihost->iomem_pcimap[1]);
	memory_region_add_subregion(get_system_memory(), 0x18000000, &pcihost->iomem_pcimap[2]);
	memory_region_transaction_commit();

}

/*
loongson3a has fix address map and xbar1, xbar2
xbar1 map system memory space to system memory space
*/

void update_xbar1map(LS3BonitoState *pcihost)
{
	int i;
	
memory_region_transaction_begin();
	for(i=0;i<8;i++)
	{
		int device;
		int bit;
		uint64_t mapaddr, mapbase, mapmask;
	
		if(!(pcihost->xbar1cfg_reg[0].base[i]&0x80)) continue;

		if((pcihost->xbar1cfg_reg[0].mask[i] & pcihost->xbar1cfg_reg[0].base[i]) != pcihost->xbar1cfg_reg[0].base[i])
			continue;
		bit = ffs(pcihost->xbar1cfg_reg[0].mask[i]);
		if(!bit) continue;
		bit--;
		device = pcihost->xbar1cfg_reg[0].map[i]&7;

		switch(device)
		{
		case 0 ... 3:
			mapbase = 0;
			mapmask = 0x7ffffffff00ULL;
			break;
		case 4:
			mapbase = 0x80000000000ULL;
			mapmask = 0x1ffffffff00ULL;
			break;
		case 5:
			mapbase = 0xa0000000000ULL;
			mapmask = 0x1ffffffff00ULL;
			break;
		case 6:
			mapbase = 0xc0000000000ULL;
			mapmask = 0x1ffffffff00ULL;
			break;
		case 7:
			mapbase = 0xe0000000000ULL;
			mapmask = 0x1ffffffff00ULL;
			break;
		}

		mapaddr = (pcihost->xbar1cfg_reg[0].map[i] & mapmask) | mapbase;
		if(mapaddr == pcihost->xbar1cfg_reg[0].base[i]) continue;

		if(pcihost->iomem_xbar1[i].parent)
			memory_region_del_subregion(get_system_memory(), &pcihost->iomem_xbar1[i]);

			memory_region_init_alias(&pcihost->iomem_xbar1[i], "cpumemi", get_system_memory(), mapaddr, 1ULL<<bit);
			memory_region_add_subregion(get_system_memory(), pcihost->xbar1cfg_reg[0].base[i], &pcihost->iomem_xbar1[i]);
	}
memory_region_transaction_commit();

}

/*xbar2 map ddr and pci*/


void update_xbar2map(LS3BonitoState *pcihost)
{
	int i;
	int maptype;
	
memory_region_transaction_begin();
	for(i=0;i<8;i++)
	{
		int bit;
		/*mmap[7]:en*/
		if(!(pcihost->xbar2cfg_reg.cpumap[i]&0x80)) continue;
		if((pcihost->xbar2cfg_reg.cpumask[i] & pcihost->xbar2cfg_reg.cpubase[i]) != pcihost->xbar2cfg_reg.cpubase[i]) continue;

		maptype = pcihost->xbar2cfg_reg.cpumap[i]&0x3;
 
		bit = ffs(pcihost->xbar2cfg_reg.cpumask[i]);
		if(!bit) continue;
		bit--;

		if(pcihost->iomem_xbar2[i].parent)
			memory_region_del_subregion(get_system_memory(), &pcihost->iomem_xbar2[i]);

		if(maptype == 2)
		{
			if(pcihost->xbar2cfg_reg.cpubase[i] == 0x10000000) continue;
			memory_region_init_alias(&pcihost->iomem_xbar2[i], "cpumemi", &pcihost->iomem_lowio, pcihost->xbar2cfg_reg.cpumap[i]&0x0ffffff0ULL, 1ULL<<bit);
			memory_region_add_subregion(get_system_memory(), pcihost->xbar2cfg_reg.cpubase[i], &pcihost->iomem_xbar2[i]);
		}

		if(maptype == 0 || maptype == 1)
		{

			memory_region_init_alias(&pcihost->iomem_xbar2[i], "cpumemi", pcihost->ram, pcihost->xbar2cfg_reg.cpumap[i]&~0xfULL, 1ULL<<bit);
			memory_region_add_subregion(get_system_memory(), pcihost->xbar2cfg_reg.cpubase[i], &pcihost->iomem_xbar2[i]);
		}
	}
memory_region_transaction_commit();
}



static void pci_ls3bonito_local_writel (void *opaque, hwaddr addr,
		uint32_t val)
{
	uint32_t relative_addr=addr&0xff;
	LS3BonitoState *pcihost = opaque;
	struct ls3bonito_regs *d=&pcihost->regs;

	switch (relative_addr) {
		case 0x10:
		 d->pcimap = val;
		update_pcimap(pcihost);
		break;
		case 0x1c:
			d->gpiodata= (d->gpiodata&0xffff0000)|(val&0xffff);
			break;
		case 0x80:
			//ddr config register
			memory_region_transaction_begin();
			if(ddrcfg_iomem->parent)
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);
			

			if(!(val&0x100))
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ffffe00&TARGET_PAGE_MASK, ddrcfg_iomem, 1);
			}

			d->reg40[(0x80-0x40)/4] = val;
			memory_region_transaction_commit();
			break;
		case 0x84:
			break;
		default:
			((uint32_t *)d)[relative_addr/sizeof(uint32_t)] = val & 0xffffffff;
			break;
	}
	ls3bonito_update_irq(pcihost);
}


static const MemoryRegionOps ls3bonito_local_ops = {
	.old_mmio = {
		.read = {
			&pci_ls3bonito_local_readb,
			&pci_ls3bonito_local_readw,
			&pci_ls3bonito_local_readl,
		},
		.write = {
			&pci_ls3bonito_local_writeb,
			&pci_ls3bonito_local_writew,
			&pci_ls3bonito_local_writel,
		},
	},
	.endianness = DEVICE_NATIVE_ENDIAN,
};

//-------------------------------------------------------------

static void ht_update_irq(void *opaque,int disable);

// per-cpu interrupt controller
static uint32_t ls3a_intctl_mem_readb(void *opaque, hwaddr addr)
{
    LS3BonitoState *s = opaque;
    uint32_t ret;

	ret=*(uint8_t *)(s->int_route_reg+addr);

    return ret;
}

static uint32_t ls3a_intctl_mem_readw(void *opaque, hwaddr addr)
{
    LS3BonitoState *s = opaque;
    uint32_t ret;


	ret=*(uint16_t *)(a->int_route_reg+addr);

    return ret;
}

static uint32_t ls3a_intctl_mem_readl(void *opaque, hwaddr addr)
{
    LS3BonitoState *s = opaque;
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
	case IO_CONTROL_REGS_CORE0_INTISR ... IO_CONTROL_REGS_CORE3_INTISR:
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
	LS3BonitoState *s = a->state;

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
	
	case INT_ROUTER_REGS_PCI_INT0 ... INT_ROUTER_REGS_PCI_INT3:
	*((uint8_t *)s->pci_introut + (a->base+addr-INT_ROUTER_REGS_PCI_INT0)) = val;
	break;
	case IO_CONTROL_REGS_INTISR ... IO_CONTROL_REGS_INTEDGE+3:
	{
		uint32_t old;
		old = *(uint8_t *)(a->mem + addr);
		if (old != val)	
		 ht_update_irq(s,1);
		*(uint8_t *)(a->mem + addr) = val;
		 ht_update_irq(s,0);
	}
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
	LS3BonitoState *s = a->state;

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
	case INT_ROUTER_REGS_PCI_INT0 ... INT_ROUTER_REGS_PCI_INT3:
	*(uint32_t *)((char *)s->pci_introut + (a->base+addr-INT_ROUTER_REGS_PCI_INT0)) = val;
	break;
	case IO_CONTROL_REGS_INTISR ... IO_CONTROL_REGS_INTEDGE+3:
	{
	*(uint32_t *)(a->mem + addr) = val;
	ht_update_irq(s,0);
	}
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
//-----------
// per-cpu interrupt controller
static uint32_t ls3a_htctl_mem_readb(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
    uint32_t ret;

	addr &= a->mask;

	ret=*(uint8_t *)(a->mem+addr);

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static uint32_t ls3a_htctl_mem_readw(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
    uint32_t ret;

	addr &= a->mask;

	ret=*(uint16_t *)(a->mem+addr);

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static uint32_t ls3a_htctl_mem_readl(void *opaque, hwaddr addr)
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
	case IO_CONTROL_REGS_CORE0_INTISR ... IO_CONTROL_REGS_CORE3_INTISR:
	break;
	default:
	ret=*(uint32_t *)(a->mem+addr);
	}

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static void ls3a_htctl_mem_writeb(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;
	LS3BonitoState *s = a->state;

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
	
	case INT_ROUTER_REGS_PCI_INT0 ... INT_ROUTER_REGS_PCI_INT3:
	break;
	case IO_CONTROL_REGS_INTISR ... IO_CONTROL_REGS_INTEDGE+3:
	{
		uint32_t old;
		old = *(uint8_t *)(a->mem + addr);
		if (old != val)	
		 ht_update_irq(s,1);
		*(uint8_t *)(a->mem + addr) = val;
		 ht_update_irq(s,0);
	}
	break;
		
	default:
	*(uint8_t *)(a->mem + addr) = val;
	break;
	}
}

static void ls3a_htctl_mem_writew(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;

	addr &= a->mask;
	*(uint16_t *)(a->mem + addr) = val;
}

static void ls3a_htctl_mem_writel(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;
	LS3BonitoState *s = a->state;

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
	case INT_ROUTER_REGS_PCI_INT0 ... INT_ROUTER_REGS_PCI_INT3:
	break;
	case IO_CONTROL_REGS_INTISR ... IO_CONTROL_REGS_INTEDGE+3:
	{
	*(uint32_t *)(a->mem + addr) = val;
	ht_update_irq(s,0);
	}
	break;
		
	default:
	*(uint32_t *)(a->mem + addr) = val;
	break;
	}
}

static const MemoryRegionOps ls3a_htctl_ops = {
    .old_mmio = {
	    .read = {
		    ls3a_htctl_mem_readb,
		    ls3a_htctl_mem_readw,
		    ls3a_htctl_mem_readl,
	    },
	    .write = {
		    ls3a_htctl_mem_writeb,
		    ls3a_htctl_mem_writew,
		    ls3a_htctl_mem_writel,
	    }
	}
	,
		.endianness = DEVICE_NATIVE_ENDIAN,
};


static void ht_update_irq(void *opaque,int disable)
{
	LS3BonitoState *s = opaque;
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
	LS3BonitoState *s = opaque;
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

//-------------------------------------------------------------
/*
loongson3 ipi interrupt
*/

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
//-------------------------------------------------------------

PCIBus *pci_ls3bonito_init(CPUMIPSState *env,qemu_irq *pic, int irq,int (*board_map_irq)(int bus,int dev,int func,int pin),MemoryRegion *ram);

//--------------------------------------------
PCIBus *pci_ls3bonito_init(CPUMIPSState *env,qemu_irq *pic, int irq,int (*board_map_irq)(int bus,int dev,int func,int pin),MemoryRegion *ram)
{
    DeviceState *dev;
    LS3BonitoState *pcihost;
    PCIHostState *phb;
    PCILS3BonitoState *s;
    PCIDevice *d;

    dev = qdev_create(NULL, TYPE_BONITO_PCI_HOST_BRIDGE);
    phb = PCI_HOST_BRIDGE(dev);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);
    pcihost->pic = pic;
    pcihost->cpu_env = env;
    pcihost->irq_offset = irq;
    pcihost->ram = ram;

    pcihost->env = env;
    local_board_map_irq = board_map_irq;
    qdev_init_nofail(dev);

    /* set the pcihost pointer before ls3bonito_initfn is called */
    d = pci_create(phb->bus, PCI_DEVFN(0, 0), "LS3A_LS3Bonito");
    s = DO_UPCAST(PCILS3BonitoState, dev, d);
    s->pcihost = pcihost;
    pcihost->pci_dev = s;
    qdev_init_nofail(DEVICE(d));


    return phb->bus;
}

//-------------------------
// pci bridge
//-----------------



static qemu_irq *pci_ls3a_pic;

static int pci_ls3a_map_irq(PCIDevice *d, int irq_num)
{
int dev=(d->devfn>>3)&0x1f;
int func=d->devfn& 7;

return local_board_map_irq(0,dev,func,irq_num);
}

static void pci_ls3a_set_irq(void *opaque, int irq_num, int level)
{
 qemu_set_irq(pci_ls3a_pic[irq_num],level);
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

static inline uint32_t bonito_pci_config_addr(LS3BonitoState *s,hwaddr addr)
{
	int bus = 0, dev = -1, func = 0, reg = 0;
	uint32_t busaddr;
	uint32_t pcimap_cfg = s->regs.pcimap_cfg;
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
	LS3BonitoState *s = opaque;
	PCIHostState *phb = PCI_HOST_BRIDGE(s);
	pci_data_write((phb+1)->bus, bonito_pci_config_addr(s, addr), val, size);
}

static uint64_t pci_ls2f_config_readl (void *opaque, hwaddr addr, unsigned size)
{
	LS3BonitoState *s = opaque;
	PCIHostState *phb = PCI_HOST_BRIDGE(s);
	uint32_t val;
	val = pci_data_read((phb+1)->bus, bonito_pci_config_addr(s, addr), size);
	return val;
}


static const MemoryRegionOps pci_ls2f_config_ops = {
    .read = pci_ls2f_config_readl,
    .write = pci_ls2f_config_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static int ls3bonito_initfn(PCIDevice *dev)
{
    /* LS3Bonito North Bridge, built on FPGA, VENDOR_ID/DEVICE_ID are "undefined" */
    pci_config_set_prog_interface(dev->config, 0x00);



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


static void ls3bonito_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = ls3bonito_initfn;
    k->vendor_id = 0xdf53;
    k->device_id = 0x00d5;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    dc->desc = "Host bridge";
    dc->no_user = 1;
}

static const TypeInfo ls3bonito_info = {
    .name          = "LS3A_LS3Bonito",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCILS3BonitoState),
    .class_init    = ls3bonito_class_init,
};

static int pcidma_translate(DMAContext *dma,
                               dma_addr_t addr,
                               hwaddr *paddr,
                               hwaddr *len,
                               DMADirection dir)
{
    LS3BonitoState *pcihost = container_of (dma, LS3BonitoState, dma);
    int i;
    hwaddr offset;
    for(i=0;i<4;i++)
    {
	    if((uint32_t)(addr & pcihost->xbar2cfg_reg.pcidmamask[i]) == (uint32_t)pcihost->xbar2cfg_reg.pcidmabase[i])
		    break;
    }
    if(i == 4) return -1;

     offset = addr & ~pcihost->xbar2cfg_reg.pcidmamask[i];

	*paddr = (pcihost->xbar2cfg_reg.pcidmamap[i]&~3ULL)|offset;
	*len = (~pcihost->xbar2cfg_reg.pcidmamask[i] - offset);

    return 0;
}

static DMAContext *pci_dma_context_fn(PCIBus *bus, void *opaque,
                                            int devfn)
{
    LS3BonitoState *pcihost = opaque;

    return &pcihost->dma;
}

static int ls3bonito_pcihost_initfn(SysBusDevice *dev)
{
    LS3BonitoState *pcihost;
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);

    memory_region_init(&pcihost->iomem_pcimem, "pcimem", 0x100000000);
    memory_region_init(&pcihost->iomem_pciemem, "pciemem", 0x100000000);
    memory_region_init(&pcihost->iomem_pcieio "pcieio", 0x10000);
    memory_region_init(&pcihost->iomem_pciio "pciio", 0x10000);
    memory_region_init(&pcihost->iomem_lowio, "pcimem", 0x10000000);

    phb->bus = pci_register_bus(DEVICE(dev), "pcie",
                                pci_ls3a_set_irq, pci_ls3a_map_irq, pcihost,
                                &pcihost->iomem_pciemem, &pcihost->pcieio,
                                12<<3, 4);

    (phb+1)->bus = pci_register_bus(DEVICE(dev), "pci",
                                pci_ls3bonito_set_irq, pci_ls3bonito_map_irq, pcihost,
                                &pcihost->iomem_pcimem, &pcihost->pciio,
                                12<<3, 4);

    dma_context_init(&pcihost->dma, &address_space_memory, pcidma_translate, NULL, NULL);
    pci_setup_iommu(phb->bus, pci_dma_context_fn, pcihost);

    memory_region_add_subregion(get_system_memory(), 0xe0000000000ULL, &pcihost->iomem_pciemem);
    memory_region_add_subregion(&pcihost->iomem_pciemem, 0x0fdfc000000ULL, &pcihost->iomem_pcieio);

   /* ls3a pcie config ops */
   {
    MemoryRegion *iomem = g_new(MemoryRegion, 1);
    memory_region_init_io(iomem, &pci_ls3a_config_ops, phb->bus, "ls3a_pci_conf", 0x2000000);
    memory_region_add_subregion(&pcihost->iomem_pciemem, HT1LO_PCICFG_BASE, iomem);
   }

   /* pcie isa ops */
   {
    ISABus *isa_bus;
	qemu_irq *i8259,*ht_irq;
	LS3a_func_args *a_irqrouter,*a_htirq;

    MemoryRegion *iomem = g_new(MemoryRegion, 1);
    isa_mmio_setup(iomem, 0x10000);
    memory_region_add_subregion(&pcihost->iomem_pciemem, 0x0fdfc000000LL, iomem);
    isa_mem_base = 0;

    isa_bus = isa_bus_new(NULL, get_system_io());


	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &ls3a_intctl_ops, pcihost, "ls3a_intctl", 256);
                memory_region_add_subregion(get_system_memory(), INT_ROUTER_REGS_BASE, iomem);
	}

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, &ls3a_htctl_ops, pcihost, "ls3a_intctl", 256);
                memory_region_add_subregion(&pcihost->iomem_pciemem, HT_CONTROL_REGS_BASE, iomem);
	}


	ht_irq = qemu_allocate_irqs(ht_set_irq, pcihost, 8);
	i8259 = i8259_init(isa_bus, ht_irq[0]); 
	cpu_outb(0x4d0,0xff);
	cpu_outb(0x4d1,0xff);

	pci_ls3a_pic = &i8259[3];

	/* The PIC is attached to the MIPS CPU INT0 pin */
	isa_bus_irqs(isa_bus, i8259);
    }

//---
   /*iomem lowio 0x1000000-0x1fffffff*/
   {
    MemoryRegion *iomem = g_new(MemoryRegion, 1);
    memory_region_init_io(iomem, &ls3bonito_local_ops, pcihost, "ls3a_pci_local", 0x100);
    memory_region_add_subregion(&pcihost->iomem_lowio, 0x0fe00100, iomem);
   }

   { 
    MemoryRegion *iomem = g_new(MemoryRegion, 1);
    /* set the north bridge pci configure  mapping */
    memory_region_init_io(iomem, &ls3bonito_pciheader_ops, pcihost,
                          "north-bridge-pci-config", 0x100);
    memory_region_add_subregion(&pcihost->iomem_lowio, 0x0fe00000, iomem);
   }

    /* set the south bridge pci configure  mapping */
    {
    MemoryRegion *iomem = g_new(MemoryRegion, 1);
    memory_region_init_io(iomem, &pci_ls2f_config_ops, pcihost,
                          "pci cfg addrmap", 0x100);
    memory_region_add_subregion(&pcihost->iomem_lowio, 0x0fe80000, iomem);
    }

	{
		ddrcfg_iomem = g_new(MemoryRegion, 1);
		memory_region_init_io(ddrcfg_iomem, &ddrcfg_dummy_ops, NULL, "ls3a ddr conf", TARGET_PAGE_SIZE);
	}

	{
	MemoryRegion *addrconf_iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(addrconf_iomem, &ls3a_xbar2_ops, pcihost, "ls3a xbar", 0x1000);
	memory_region_add_subregion(get_system_memory(), 0x3ff00000, addrconf_iomem);
	}

	{
	MemoryRegion *addrconf_iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(addrconf_iomem, &ls3a_xbar1_ops, pcihost, "ls3a xbar", 0x1000);
	memory_region_add_subregion(get_system_memory(), 0x3ff02000, addrconf_iomem);
	}

	pcihost->regs.intisr=0;
	pcihost->regs.inten=0xf<<11;

//----



    pcihost->xbar2cfg_reg.cpubase[0] = 0;
    pcihost->xbar2cfg_reg.cpubase[1] = 0x10000000ULL;
    pcihost->xbar2cfg_reg.cpubase[2] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.cpubase[3] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.cpumask[0] = 0xfffffffff0000000ULL;
    pcihost->xbar2cfg_reg.cpumask[1] = 0xfffffffff0000000ULL;
    pcihost->xbar2cfg_reg.cpumask[2] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.cpumask[3] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.cpumap[0] = 0;
    pcihost->xbar2cfg_reg.cpumap[1] = 0x10000001ULL;
    pcihost->xbar2cfg_reg.cpumap[2] = 0;
    pcihost->xbar2cfg_reg.cpumap[3] = 0;
    pcihost->xbar2cfg_reg.pcidmabase[0] = 0x80000000ULL;
    pcihost->xbar2cfg_reg.pcidmabase[1] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmabase[2] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmabase[3] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmamask[0] = 0xffffffff80000000ULL;
    pcihost->xbar2cfg_reg.pcidmamask[1] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmamask[2] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmamask[3] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmamap[0] = 0;
    pcihost->xbar2cfg_reg.pcidmamap[1] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmamap[2] = 0xfffffffffff00000ULL;
    pcihost->xbar2cfg_reg.pcidmamap[3] = 0xfffffffffff00000ULL;

    pcihost->regs.pcimap = 0;
    update_xbar1map(pcihost);

    update_xbar2map(pcihost);

    return 0;
}

static void ls3bonito_pcihost_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls3bonito_pcihost_initfn;
    dc->no_user = 1;
}

static const TypeInfo ls3bonito_pcihost_info = {
    .name          = TYPE_BONITO_PCI_HOST_BRIDGE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(LS3BonitoState),
    .class_init    = ls3bonito_pcihost_class_init,
};

static void ls3bonito_register_types(void)
{
    type_register_static(&ls3bonito_pcihost_info);
    type_register_static(&ls3bonito_info);
}

type_init(ls3bonito_register_types)


#if 0


/*
 * PHB PCI device
 */
hw/pci/pci.c
    if (bus->dma_context_fn) {
        pci_dev->dma = bus->dma_context_fn(bus, bus->dma_context_opaque, devfn);
    } else {
        /* FIXME: Make dma_context_fn use MemoryRegions instead, so this path is
         * taken unconditionally */
        /* FIXME: inherit memory region from bus creator */
        memory_region_init_alias(&pci_dev->bus_master_enable_region, "bus master",
                                 get_system_memory(), 0,
                                 memory_region_size(get_system_memory()));
        memory_region_set_enabled(&pci_dev->bus_master_enable_region, false);
        address_space_init(&pci_dev->bus_master_as, &pci_dev->bus_master_enable_region);
        pci_dev->dma = g_new(DMAContext, 1);
        dma_context_init(pci_dev->dma, &pci_dev->bus_master_as, NULL, NULL, NULL);
    }
cpu0 -d4 0x90000EFDFB000060 10
90000efdfb000060: c0000000 0080fff0 c0000000 00008000 ................
90000efdfb000070: 00000000 00000000 00000000 00000000 ................
90000efdfb000080: 00000000 00000000 
loongson3_HT_init.S

#if 1//OPEN RX SPACE in HOST
	TTYDBG("HT RX DMA address ENABLE\r\n")
	dli	    t2, 0x90000efdfb000060
	li	    t0, 0xc0000000
	sw	    t0, 0x0(t2)
	li	    t0, 0x0080fff0
	sw	    t0, 0x4(t2)
	TTYDBG("HT RX DMA address ENABLE done 1\r\n")


	li	t0, 0xc0000000
	sw	t0, 0x8(t2)
	li	t0, 0x00008000
	sw	t0, 0xc(t2)
	TTYDBG("HT RX DMA address ENABLE done 2\r\n")

#if 0
	li	t0, 0xc0000000
	sw	t0, 0x10(t2)
	li	t0, 0x0040ffc0
	sw	t0, 0x14(t2)
	TTYDBG("HT RX DMA address ENABLE done 3\r\n")
#endif

#endif
#endif
