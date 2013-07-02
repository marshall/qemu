/*
 * Arduino Due (Cortex-M3) peripherals
 */

#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/boards.h"
#include "exec/address-spaces.h"

/* System controller */

enum {
    REG_PMC_SCER,
    REG_PMC_SCDR,
    REG_PMC_SCSR,
    REG_PMC_PCER0,
    REG_PMC_PCDR0,
    REG_PMC_PCSR0,
    REG_CKGR_UCKR,
    REG_CKGR_MOR,
    REG_CKGR_MCFR,
    REG_CKGR_PLLAR,
    REG_PMC_MCKR,
    REG_PMC_USB,
    REG_PMC_PCK0,
    REG_PMC_PCK1,
    REG_PMC_PCK2,
    REG_PMC_IER,
    REG_PMC_IDR,
    REG_PMC_SR,
    REG_PMC_IMR,
    REG_PMC_FSMR,
    REG_PMC_FSPR,
    REG_PMC_FOCR,
    REG_PMC_WPMR,
    REG_PMC_WPSR,
    REG_PMC_PCER1,
    REG_PMC_PCDR1,
    REG_PMC_PCSR1,
    REG_PMC_PCR,

    REG_PMC_COUNT
};

typedef struct {
    MemoryRegion iomem;
    qemu_irq irq;

    // power mgmt controller
    uint32_t pmc[REG_PMC_COUNT];
  } sysc_state;

static uint64_t sysc_read(void *opaque,  hwaddr addr, unsigned size)
{
    sysc_state *s = (sysc_state *) opaque;

    if (addr % 4 == 0) {
        if (addr >= 0x600 && addr < 0x800) {
            return s->pmc[(addr - 0x600) / 4];
        }
    }

    hw_error("sysc_read: Bad offset 0x%x\n", (int) addr);
    return 0;
}

static void pmc_write(sysc_state *s, hwaddr addr, uint64_t value)
{
#define SET_PMC_REG(r, v) s->pmc[r] |= v
#define CLEAR_PMC_REG(r, v) s->pmc[r] &= ~(v)

    switch (addr) {
        case REG_PMC_SCER:
            SET_PMC_REG(REG_PMC_SCSR, value); break;
        case REG_PMC_SCDR:
            CLEAR_PMC_REG(REG_PMC_SCSR, value); break;

        case REG_PMC_PCER0:
            SET_PMC_REG(REG_PMC_PCSR0, value); break;
        case REG_PMC_PCDR0:
            CLEAR_PMC_REG(REG_PMC_PCSR0, value); break;

        case REG_CKGR_UCKR:
            s->pmc[REG_CKGR_UCKR] = value; break;

    }
}

static void sysc_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    sysc_state *s = (sysc_state *) opaque;

    if (addr % 4 == 0) {
        if (addr >= 0x600 && addr < 0x800) {
            pmc_write(s, (addr - 0x600) / 4, value);
            return;
        }
    }
}

static const MemoryRegionOps sysc_ops = {
    .read = sysc_read,
    .write = sysc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int sysc_reset(void *opaque)
{
    sysc_state *s = (sysc_state *) opaque;
    s->pmc[REG_PMC_SCSR] = 1;
    s->pmc[REG_PMC_PCSR0] = 0;
    s->pmc[REG_CKGR_UCKR] = 0x10200800;
    s->pmc[REG_CKGR_MOR] = 1;
    s->pmc[REG_CKGR_MCFR] = 0;
    s->pmc[REG_CKGR_PLLAR] = 0x3f00;
    s->pmc[REG_PMC_MCKR] = 1;
    s->pmc[REG_PMC_USB] = 0;
    s->pmc[REG_PMC_PCK0] = 0;
    s->pmc[REG_PMC_PCK1] = 0;
    s->pmc[REG_PMC_PCK2] = 0;
    s->pmc[REG_PMC_SR] = 0x10008;
    s->pmc[REG_PMC_IMR] = 0;
    s->pmc[REG_PMC_FSMR] = 0;
    s->pmc[REG_PMC_FSPR] = 0;
    s->pmc[REG_PMC_WPMR] = 0;
    s->pmc[REG_PMC_WPSR] = 0;
    s->pmc[REG_PMC_PCSR1] = 0;
    s->pmc[REG_PMC_PCR] = 0;
    return 0;
}

static int sam3xa_sys_init(uint32_t base)
{
    sysc_state *s = (sysc_state *) g_malloc0(sizeof(sysc_state));
    //s->irq = irq;

    memory_region_init_io(&s->iomem, &sysc_ops, s, "sysc", 0x2600);
    memory_region_add_subregion(get_system_memory(), base, &s->iomem);
    sysc_reset(s);

    return 0;
}

static void arduino_due_init(QEMUMachineInitArgs *args)
{
    static const int serial_irqs[4] = {
        8, // UART = Serial
        17, // USART0 = Serial1
        18, // USART1 = Serial2
        20, // USART3 = Serial3
    };

    static const uint32_t serial_addrs[4] = {
        0x400E0800, // UART
        0x40098000, // USART0
        0x4009C000, // USART1
        0x400A4000, // USART3
    };

    static const int flash_size = 512; // 512kb boot, 512kb program, 1024kb rom
    static const int sram_size = 96; // 96kb
    qemu_irq *pic;
    int i;

    MemoryRegion *address_space_mem = get_system_memory();
    pic = armv7m_init(address_space_mem, flash_size, sram_size,
                      args->kernel_filename, args->cpu_model);

    sam3xa_sys_init(0x400e0000);

    // Serial devices
    //for (i = 0; i < 4; i++) {
        sysbus_create_simple("sam3xa_uart", serial_addrs[0], pic[serial_irqs[0]]);
   // }
}

static QEMUMachine arduino_due_machine = {
    .name = "arduino_due",
    .desc = "Arduino Due",
    .init = arduino_due_init,
    DEFAULT_MACHINE_OPTIONS,
};

static void arduino_due_machine_init(void)
{
    qemu_register_machine(&arduino_due_machine);
}

machine_init(arduino_due_machine_init);
