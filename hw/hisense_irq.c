/*
 * QEMU Sparc GS232 interrupt controller emulation
 *
 * Copyright (c) 2003-2005 Fabrice Bellard
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
#include "sun4m.h"
//#include "console.h"
#include "exec/address-spaces.h"
#include "pci/pci.h"

//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define DPRINTF(fmt, args...) \
do { printf("IRQ: " fmt , ##args); } while (0)
#else
#define DPRINTF(fmt, args...)
#endif

/*
 * Registers of interrupt controller in sun4m.
 *
 * This is the interrupt controller part of chip STP2001 (Slave I/O), also
 * produced as NCR89C105. See
 * http://www.ibiblio.org/pub/historic-linux/early-ports/Sparc/NCR/NCR89C105.txt
 *
 * There is a system master controller and one for each cpu.
 *
 */

#define MAX_CPUS 1
#define MAX_PILS 16

typedef struct GS232_INTCTLState {
	uint32_t baseaddr;
    uint32_t intreg_edge;
    uint32_t intreg_steer;
    uint32_t intreg_pol;
	//set
	//clr
    uint32_t intreg_en;
    uint32_t intreg_pending;
#ifdef DEBUG_IRQ_COUNT
    uint64_t irq_count[32];
#endif
    qemu_irq cpu_irq;
    uint32_t pil_out;
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

static void hisense_check_interrupts(void *opaque);

// per-cpu interrupt controller
static uint64_t hisense_intctl_mem_readl(void *opaque, hwaddr addr, unsigned size)
{
    GS232_INTCTLState *s = opaque;
    uint32_t saddr, ret;

    saddr = (addr - (s->baseaddr&~TARGET_PAGE_MASK)) >> 2;
    switch (saddr) {
    case 1: //isr
        ret = s->intreg_pending & s->intreg_en;
        break;
	case 0:
		ret= s->intreg_en;
		break;
	case 2: //set
		ret= ~s->intreg_en;
		break;
	case 3: //clr
		ret=0;
		break;
	case 4:
		ret= s->intreg_pol;
		break;
	case 5:
		ret= s->intreg_edge;
		break;
    default:
        ret = 0;
        break;
    }
    DPRINTF("read cpu %d reg 0x" TARGET_FMT_plx " = %x\n", cpu, addr, ret);

    return ret;
}

static void hisense_intctl_mem_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    GS232_INTCTLState *s = opaque;
    uint32_t saddr;

    saddr = (addr - (s->baseaddr&~TARGET_PAGE_MASK)) >> 2;
    //printf("write cpu %d reg 0x" TARGET_FMT_plx " = %x\n", cpu, addr, val);
    switch (saddr) {
    case 1: //isr
        s->intreg_pending &= ~val;
        hisense_check_interrupts(s);
        break;
	case 0:
		s->intreg_en=~val;
        hisense_check_interrupts(s);
		break;
	case 2: //set mask
		s->intreg_en = ~val;
        hisense_check_interrupts(s);
		break;
	case 3: //clr
		s->intreg_pending &= ~(val & s->intreg_edge);
        hisense_check_interrupts(s);
		break;
	case 4:
		s->intreg_pol=val;
        hisense_check_interrupts(s);
		break;
	case 5:
		s->intreg_edge=val;
        hisense_check_interrupts(s);
		break;
    default:
        break;
    }
}

static const MemoryRegionOps hisense_intctl_mem_ops = {
    .read = hisense_intctl_mem_readl,
    .write = hisense_intctl_mem_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};



void hisense_irq_info(void *opaque);
void *hisense_intctl_init(hwaddr addr,qemu_irq parent_irq);

void hisense_irq_info(void *opaque)
{
#ifndef DEBUG_IRQ_COUNT
    printf("irq statistic code not compiled.\n");
#else
    GS232_INTCTLState *s = opaque;
    int i;
    int64_t count;

    printf("IRQ statistics:\n");
    for (i = 0; i < 32; i++) {
        count = s->irq_count[i];
        if (count > 0)
            printf("%2d: %" PRId64 "\n", i, count);
    }
#endif
}

static void hisense_check_interrupts(void *opaque)
{
    GS232_INTCTLState *s = opaque;
    uint32_t pil_pending;


        pil_pending = s->intreg_pending & s->intreg_en;

            if (pil_pending ) {
                    qemu_irq_raise(s->cpu_irq);
            } else {
                if (s->pil_out)
                    qemu_irq_lower(s->cpu_irq);
            }
        s->pil_out = pil_pending;
    DPRINTF("pending %x \n", pil_pending);
}

/*
 * "irq" here is the bit number in the system interrupt register to
 * separate serial and keyboard interrupts sharing a level.
 */
static void hisense_set_irq(void *opaque, int irq, int level)
{
    GS232_INTCTLState *s = opaque;
    uint32_t mask = 1 << irq;

    DPRINTF("Set cpu %d irq %d level %d\n", s->target_cpu, irq,
            level);
        if (level) {
            s->intreg_pending |= mask;
        } else {
            s->intreg_pending &= ~mask | s->intreg_edge;
        }
        hisense_check_interrupts(s);
}



static void hisense_intctl_reset(void *opaque)
{
    GS232_INTCTLState *s = opaque;

    s->intreg_pending = 0;
    hisense_check_interrupts(s);
}


void *hisense_intctl_init(hwaddr addr,qemu_irq parent_irq)
{
	qemu_irq *irqs;
	GS232_INTCTLState *s;
	MemoryRegion *address_space_mem = get_system_memory();

	s = g_malloc0(sizeof(GS232_INTCTLState));
	if (!s)
		return NULL;



	MemoryRegion *iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(iomem, &hisense_intctl_mem_ops, s, "inctl", INTCTL_SIZE);
	memory_region_add_subregion(address_space_mem, addr, iomem);

	s->cpu_irq = parent_irq;
	s->baseaddr=addr;

    qemu_register_reset(hisense_intctl_reset, s);
    irqs = qemu_allocate_irqs(hisense_set_irq, s, 32);

    hisense_intctl_reset(s);
    return irqs;
}
