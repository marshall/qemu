/*
 * ATMEL SAM3XA UART
 *
 * This code is licensed under the GPL.
 */

#include "hw/sysbus.h"
#include "sysemu/char.h"
#include "trace.h"

#define REG_UART_CR            (0x400E0800U) /**< \brief (UART) Control Register */
#define REG_UART_MR            (0x400E0804U) /**< \brief (UART) Mode Register */
#define REG_UART_IER           (0x400E0808U) /**< \brief (UART) Interrupt Enable Register */
#define REG_UART_IDR           (0x400E080CU) /**< \brief (UART) Interrupt Disable Register */
#define REG_UART_IMR           (0x400E0810U) /**< \brief (UART) Interrupt Mask Register */
#define REG_UART_SR            (0x400E0814U) /**< \brief (UART) Status Register */
#define REG_UART_RHR           (0x400E0818U) /**< \brief (UART) Receive Holding Register */
#define REG_UART_THR           (0x400E081CU) /**< \brief (UART) Transmit Holding Register */
#define REG_UART_BRGR          (0x400E0820U) /**< \brief (UART) Baud Rate Generator Register */
#define REG_UART_RPR           (0x400E0900U) /**< \brief (UART) Receive Pointer Register */
#define REG_UART_RCR           (0x400E0904U) /**< \brief (UART) Receive Counter Register */
#define REG_UART_TPR           (0x400E0908U) /**< \brief (UART) Transmit Pointer Register */
#define REG_UART_TCR           (0x400E090CU) /**< \brief (UART) Transmit Counter Register */
#define REG_UART_RNPR          (0x400E0910U) /**< \brief (UART) Receive Next Pointer Register */
#define REG_UART_RNCR          (0x400E0914U) /**< \brief (UART) Receive Next Counter Register */
#define REG_UART_TNPR          (0x400E0918U) /**< \brief (UART) Transmit Next Pointer Register */
#define REG_UART_TNCR          (0x400E091CU) /**< \brief (UART) Transmit Next Counter Register */
#define REG_UART_PTCR          (0x400E0920U) /**< \brief (UART) Transfer Control Register */
#define REG_UART_PTSR          (0x400E0924U) /**< \brief (UART) Transfer Status Register */

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    CharDriverState *chr;
    qemu_irq irq;

    uint32_t mode;
    uint32_t mask;
    uint32_t status;
    uint8_t read_fifo[16];
    uint8_t read_pos;
    uint8_t read_count;
} sam3xa_uart_state;

#define STATUS_RXRDY   (1 << 0)
#define STATUS_TXRDY   (1 << 1)
#define STATUS_ENDRX   (1 << 3)
#define STATUS_ENDTX   (1 << 4)
#define STATUS_OVRE    (1 << 5)
#define STATUS_FRAME   (1 << 6)
#define STATUS_PARE    (1 << 7)
#define STATUS_TXEMPTY (1 << 9)
#define STATUS_TXBUFE  (1 << 11)
#define STATUS_RXBUFF  (1 << 12)

#define CONTROL_RSTRX (1 << 2)
#define CONTROL_RXEN  (1 << 4)
#define CONTROL_TXEN  (1 << 6)

static void
uart_update(sam3xa_uart_state *s)
{
    uint32_t flags;

    // We don't queue TX chars
    s->status |= STATUS_TXEMPTY | STATUS_TXBUFE;

    if (s->read_count) {
        s->status |= STATUS_RXRDY;
    }

    if (s->read_count == sizeof(s->read_fifo)) {
        s->status |= STATUS_RXBUFF;
    }

    flags = s->status & s->mask;
    trace_sam3xa_uart_update(s->status, s->mask, flags);

    qemu_set_irq(s->irq, flags != 0);
}

static uint64_t
uart_read(void *opaque, hwaddr addr, unsigned size)
{
    printf("uart_read %p\n", addr);
    sam3xa_uart_state *s = (sam3xa_uart_state *) opaque;
    uint32_t c;

    switch (addr) {
        case 0x04: /* UART_MR (mode) */
            return s->mode;
        case 0x10: /* UART_IMR (interrupt mask) */
            return s->mask;
        case 0x14: /* UART_SR (status) */
            return s->status;
        case 0x18: /* UART_RHR (receiver holding) */
            c = s->read_fifo[(s->read_pos - s->read_count) & 15];
            if (s->read_count) {
                s->read_count--;
            }

            uart_update(s);
            if (s->chr) {
                qemu_chr_accept_input(s->chr);
            }
            return c;

        // write-only registers
        case 0x00: /* UART_CR (control) */
        case 0x08: /* UART_IER (interrupt enable) */
        case 0x0c:/* UART_IDR (interrupt disable) */
        default:
            return 0;
    }
}

static void
uart_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    sam3xa_uart_state *s = (sam3xa_uart_state *) opaque;
    uint32_t value = val64;
    printf("uart_write %p, val=%p\n", addr, value);
    unsigned char ch = value;

    switch (addr) {
        case 0x00: /* UART_CR (control) */
            if (value & CONTROL_RSTRX) {
                s->read_pos = s->read_count = 0;
            }
            break;
        case 0x08: /* UART_IER (interrupt enable) */
            s->mask |= value;
            break;
        case 0x0c:/* UART_IDR (interrupt disable) */
            s->mask &= ~value;
            break;
        case 0x1c: /* UART_THR (transmit holder) */
            if (s->chr) {
                qemu_chr_fe_write(s->chr, &ch, 1);
            }
            break;
    }

    uart_update(s);
}

static const MemoryRegionOps sam3xa_uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_sam3xa_uart = {
    .name = "sam3xa_uart",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(mode, sam3xa_uart_state),
        VMSTATE_UINT32(mask, sam3xa_uart_state),
        VMSTATE_UINT32(status, sam3xa_uart_state),
        VMSTATE_UINT8_ARRAY(read_fifo, sam3xa_uart_state, 16),
        VMSTATE_UINT8(read_pos, sam3xa_uart_state),
        VMSTATE_UINT8(read_count, sam3xa_uart_state),
        VMSTATE_END_OF_LIST()
    }
};

static void uart_rx(void *opaque, const uint8_t *buf, int size)
{
    printf("uart_rx, size=%d, buf=%c\n", size, (char) *buf);
    sam3xa_uart_state *s = (sam3xa_uart_state *) opaque;

    if (s->read_count >= 16) {
        printf("WARNING: UART dropped char.\n");
        return;
    }

    s->read_fifo[s->read_pos] = *buf;
    s->read_pos++;
    s->read_pos &= 0x7;
    s->read_count++;

    uart_update(s);
}

static int uart_can_rx(void *opaque)
{
    sam3xa_uart_state *s = (sam3xa_uart_state *) opaque;

    trace_sam3xa_uart_can_rx(s->read_count < sizeof(s->read_fifo));
    return s->read_count < sizeof(s->read_fifo);
}

static void uart_event(void *opaque, int event)
{
    printf("uart_event: %d\n", event);
}

static int sam3xa_uart_init(SysBusDevice *dev)
{
    printf("sam3xa_uart_init\n");
    sam3xa_uart_state *s = FROM_SYSBUS(sam3xa_uart_state, dev);

    // TODO init status
    //s->status |= STATUS_TXEN;

    memory_region_init_io(&s->iomem, &sam3xa_uart_ops, s, "sam3xa_uart", 0x0124);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);

    s->chr = qemu_char_get_next_serial();
    if (s->chr) {
        qemu_chr_add_handlers(s->chr, uart_can_rx, uart_rx, uart_event, s);
    }

    vmstate_register(&dev->qdev, -1, &vmstate_sam3xa_uart, s);
    return 0;
}

static void sam3xa_uart_class_init(ObjectClass *klass, void *data)
{
    printf("sam3xa_uart_class_init\n");
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    sdc->init = sam3xa_uart_init;
}

static const TypeInfo sam3xa_uart_info = {
    .name          = "sam3xa_uart",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(sam3xa_uart_state),
    .class_init    = sam3xa_uart_class_init,
};

static void sam3xa_uart_register_types(void)
{
    type_register_static(&sam3xa_uart_info);
}

type_init(sam3xa_uart_register_types)
