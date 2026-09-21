/*
 * QEMU model of the Loongson Chiplab RV32 board (chiplab 实验箱, Artix-7 A200T)
 *
 * This machine models the chiplab RV32-GC SoC closely enough to run the real
 * SPI-XIP boot chain (boot_stub -> OpenSBI fw_jump -> U-Boot) and the U-Boot
 * NAND/MTD stack on a PC, so that the runtime checks that need hardware
 * (saveenv persistence, bootcmd autoboot) can be exercised without the board.
 *
 * Address map (rv32gc-cpu/docs/kb/platform-facts.md + u-boot chiplab-rv32.dts):
 *
 *   0x0000_0000  DDR3             128 MiB  machine RAM
 *   0x1C00_0000  SPI XIP window    16 MiB  flash offset 0x000000 (XIP, read-only)
 *   0x03F0_0000  -dtb             payload  FDT for U-Boot's booti
 *   0x0400_0000  -kernel          payload  Linux Image (raw)
 *   0x0600_0000  -initrd          payload  initramfs (cpio/newc)
 *   0x1FD0_0000  confreg           64 KiB  platform DMA order register @ 0x1160
 *   0x1FE0_01E0  UART0 (16550)     0x100   8-bit registers, 33 MHz
 *   0x1FE7_8000  NAND controller  0x1000   K9F1G08U0C + DMA doorbell @ 0x40
 *   0x1FE8_0000  SPI alias        64 KiB   flash offset 0x00E8_0000 (r/w)
 *   0x1F00_0000  CLINT            64 KiB   SiFive layout (mtimecmp 0x4000, mtime 0xbff8)
 *   0x1F10_0000  PLIC              4 MiB   31 sources
 *
 * Reset PC = 0x1C00_0000: the platform has no boot ROM, the CPU starts
 * executing straight out of the SPI XIP window.
 *
 * The NAND controller register semantics are modelled bit for bit after
 * u-boot/drivers/mtd/nand/raw/chiplab_nand.{c,h} (which in turn follows
 * chiplab/IP/APB_DEV/NAND/nand.v); the platform DMA engine follows the
 * descriptor format of chiplab/IP/DMA/dma.v as packed by the driver.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/loader.h"
#include "hw/qdev-properties.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/sysbus.h"
#include "sysemu/block-backend.h"
#include "sysemu/blockdev.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"

/* ------------------------------------------------------------------------ */
/* Platform constants                                                        */
/* ------------------------------------------------------------------------ */

/* 33 MHz uncore domain (platform-facts.md §5, config.h FREQ) */
#define CHIPLAB_TIMEBASE_FREQ   33000000

/* Reset vector: SPI XIP main window (platform has no boot ROM) */
#define CHIPLAB_RESET_PC        0x1c000000

#define CHIPLAB_DDR_SIZE        (128 * MiB)

/*
 * Optional payloads pre-loaded into DDR before reset (-kernel/-initrd/-dtb).
 * They are only *placed* in RAM: the CPU still resets into the SPI XIP window
 * (boot_stub -> OpenSBI -> U-Boot), and U-Boot boots them, e.g.
 *
 *   setenv filesize 0x<initrd size>
 *   booti 0x04000000 0x06000000:${filesize} 0x03f00000
 *
 * Layout check (128 MiB DDR3):
 *   0x00000000..0x000FFFFF  early boot scratch (boot_stub stack-free, unused)
 *   0x01000000..0x010FFFFF  OpenSBI (loaded by the boot stub)
 *   0x02000000..0x0207FFFF  U-Boot text/data (pre-relocation)
 *   0x03F00000..0x03FFFFFF  FDT           (-dtb)      < 1 MiB, below the kernel
 *   0x04000000..0x05FFFFFF  kernel Image  (-kernel)   32 MiB window
 *   0x06000000..0x07AFFFFF  initramfs     (-initrd)   26 MiB window
 *   0x07B00000..0x07FFFFFF  relocated U-Boot + malloc (SYS_MALLOC_LEN = 4 MiB)
 */
#define CHIPLAB_KERNEL_ADDR     0x04000000
#define CHIPLAB_KERNEL_MAX      (32 * MiB)
#define CHIPLAB_INITRD_ADDR     0x06000000
#define CHIPLAB_INITRD_MAX      (26 * MiB)
#define CHIPLAB_DTB_ADDR        0x03f00000
#define CHIPLAB_DTB_MAX         (1 * MiB)

#define CHIPLAB_XIP_BASE        0x1c000000
/*
 * XIP main window: the whole 16 MiB SPI device is visible at 0x1C00_0000
 * (dts: spi-xip@1c000000 reg = <0x1c000000 0x1000000>; the platform's
 * axi_mux only decodes addr[31:20]==0x1C0, i.e. 1 MiB, so the upper 15 MiB is
 * a model-level extension - documented in docs/linux-port/README.md).
 */
#define CHIPLAB_XIP_SIZE        (16 * MiB)
#define CHIPLAB_XIP_ALIAS_BASE  0x1fe80000
#define CHIPLAB_XIP_ALIAS_SIZE  (64 * KiB)
/* alias window 0x1FE8_0000+N aliases flash offset 0x00E8_0000+N */
#define CHIPLAB_XIP_ALIAS_FLASH_OFF 0x00e80000

#define CHIPLAB_FLASH_SIZE      (16 * MiB)     /* S25FL128S */

#define CHIPLAB_CONFREG_BASE    0x1fd00000
#define CHIPLAB_CONFREG_SIZE    (64 * KiB)
#define CHIPLAB_DMA_ORDER_OFF   0x1160         /* confreg_syn.v:33 */

#define CHIPLAB_NAND_BASE       0x1fe78000
#define CHIPLAB_NAND_SIZE       0x1000

#define CHIPLAB_UART0_BASE      0x1fe001e0

#define CHIPLAB_CLINT_BASE      0x1f000000
#define CHIPLAB_CLINT_SIZE      (64 * KiB)

#define CHIPLAB_PLIC_BASE       0x1f100000
#define CHIPLAB_PLIC_SIZE       (4 * MiB)
#define CHIPLAB_PLIC_NUM_SOURCES 31            /* dts: riscv,ndev = 31 */
#define CHIPLAB_PLIC_NUM_PRIO_BITS 3
#define CHIPLAB_UART0_IRQ       1              /* PLIC source 1 */

/* SiFive PLIC register layout (same as -M virt) */
#define CHIPLAB_PLIC_PRIORITY_BASE   0x00
#define CHIPLAB_PLIC_PENDING_BASE    0x1000
#define CHIPLAB_PLIC_ENABLE_BASE     0x2000
#define CHIPLAB_PLIC_ENABLE_STRIDE   0x80
#define CHIPLAB_PLIC_CONTEXT_BASE    0x200000
#define CHIPLAB_PLIC_CONTEXT_STRIDE  0x1000

enum {
    CHIPLAB_DEV_DDR = 0,
    CHIPLAB_DEV_SPI_XIP,
    CHIPLAB_DEV_CONFREG,
    CHIPLAB_DEV_UART0,
    CHIPLAB_DEV_NAND,
    CHIPLAB_DEV_SPI_ALIAS,
    CHIPLAB_DEV_CLINT,
    CHIPLAB_DEV_PLIC,
};

static const MemMapEntry chiplab_memmap[] = {
    [CHIPLAB_DEV_DDR]       = { 0x00000000,          CHIPLAB_DDR_SIZE },
    [CHIPLAB_DEV_SPI_XIP]   = { CHIPLAB_XIP_BASE,    CHIPLAB_XIP_SIZE },
    [CHIPLAB_DEV_CONFREG]   = { CHIPLAB_CONFREG_BASE, CHIPLAB_CONFREG_SIZE },
    [CHIPLAB_DEV_UART0]     = { CHIPLAB_UART0_BASE,  0x100 },
    [CHIPLAB_DEV_NAND]      = { CHIPLAB_NAND_BASE,   CHIPLAB_NAND_SIZE },
    [CHIPLAB_DEV_SPI_ALIAS] = { CHIPLAB_XIP_ALIAS_BASE, CHIPLAB_XIP_ALIAS_SIZE },
    [CHIPLAB_DEV_CLINT]     = { CHIPLAB_CLINT_BASE,  CHIPLAB_CLINT_SIZE },
    [CHIPLAB_DEV_PLIC]      = { CHIPLAB_PLIC_BASE,   CHIPLAB_PLIC_SIZE },
};

/* ======================================================================== */
/* SPI NOR flash: XIP main window + 64 KiB alias window                     */
/* ======================================================================== */

#define TYPE_CHIPLAB_SPI_FLASH "chiplab.spi-flash"
OBJECT_DECLARE_SIMPLE_TYPE(ChiplabSpiFlashState, CHIPLAB_SPI_FLASH)

struct ChiplabSpiFlashState {
    SysBusDevice parent_obj;

    MemoryRegion xip;       /* 0x1c000000, 1 MiB, flash offset 0, read only */
    MemoryRegion alias;     /* 0x1fe80000, 64 KiB, flash offset 0xe80000, r/w */

    uint8_t *storage;       /* whole 16 MiB device image (RAM mirror) */
    char *image;            /* initial content (boot image), optional */
};

static uint64_t chiplab_flash_read_bytes(const uint8_t *p, unsigned size)
{
    switch (size) {
    case 1:
        return ldub_p(p);
    case 2:
        return lduw_le_p(p);
    case 4:
        return ldl_le_p(p);
    case 8:
        return ldq_le_p(p);
    default:
        g_assert_not_reached();
    }
}

static void chiplab_flash_write_bytes(uint8_t *p, uint64_t value, unsigned size)
{
    switch (size) {
    case 1:
        stb_p(p, value);
        break;
    case 2:
        stw_le_p(p, value);
        break;
    case 4:
        stl_le_p(p, value);
        break;
    case 8:
        stq_le_p(p, value);
        break;
    default:
        g_assert_not_reached();
    }
}

/*
 * Main XIP window: 0x1C00_0000 + N -> flash offset N (N < 1 MiB).
 *
 * This is the window the core fetches from after reset (XIP straight out of
 * the SPI chip, bypassing the caches on real hardware).  It is read-only:
 * updates go through the alias window (see below), matching the platform
 * note "XIP 写只能走 0x1FE8 别名".
 */
static uint64_t chiplab_flash_xip_read(void *opaque, hwaddr addr, unsigned size)
{
    ChiplabSpiFlashState *s = opaque;

    if (addr + size > CHIPLAB_XIP_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "chiplab-spi: XIP read beyond 1 MiB window "
                      "(offset 0x%" HWADDR_PRIx ")\n", addr);
        return 0;
    }
    return chiplab_flash_read_bytes(s->storage + addr, size);
}

static void chiplab_flash_xip_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR,
                  "chiplab-spi: write to read-only XIP window 0x%" HWADDR_PRIx
                  " (use the 0x%08x alias window instead)\n",
                  CHIPLAB_XIP_BASE + addr, CHIPLAB_XIP_ALIAS_BASE);
}

static const MemoryRegionOps chiplab_flash_xip_ops = {
    .read = chiplab_flash_xip_read,
    .write = chiplab_flash_xip_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * Alias window: 0x1FE8_0000 + N -> flash offset 0x00E8_0000 + N (64 KiB).
 * Read/write; writes update the in-RAM mirror of the device, so a subsequent
 * read through the main XIP window sees the new content (same storage body).
 * The host file passed with -bios is never written back.
 */
static uint64_t chiplab_flash_alias_read(void *opaque, hwaddr addr, unsigned size)
{
    ChiplabSpiFlashState *s = opaque;

    if (addr + size > CHIPLAB_XIP_ALIAS_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "chiplab-spi: alias read beyond 64 KiB window "
                      "(offset 0x%" HWADDR_PRIx ")\n", addr);
        return 0;
    }
    return chiplab_flash_read_bytes(s->storage + CHIPLAB_XIP_ALIAS_FLASH_OFF +
                                    addr, size);
}

static void chiplab_flash_alias_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned size)
{
    ChiplabSpiFlashState *s = opaque;

    if (addr + size > CHIPLAB_XIP_ALIAS_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "chiplab-spi: alias write beyond 64 KiB window "
                      "(offset 0x%" HWADDR_PRIx ")\n", addr);
        return;
    }
    chiplab_flash_write_bytes(s->storage + CHIPLAB_XIP_ALIAS_FLASH_OFF + addr,
                              value, size);
}

static const MemoryRegionOps chiplab_flash_alias_ops = {
    .read = chiplab_flash_alias_read,
    .write = chiplab_flash_alias_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void chiplab_spi_flash_realize(DeviceState *dev, Error **errp)
{
    ChiplabSpiFlashState *s = CHIPLAB_SPI_FLASH(dev);
    ssize_t loaded;

    s->storage = g_malloc(CHIPLAB_FLASH_SIZE);
    memset(s->storage, 0xff, CHIPLAB_FLASH_SIZE);

    memory_region_init_io(&s->xip, OBJECT(dev), &chiplab_flash_xip_ops, s,
                          "chiplab.spi-flash.xip", CHIPLAB_XIP_SIZE);
    memory_region_init_io(&s->alias, OBJECT(dev), &chiplab_flash_alias_ops, s,
                          "chiplab.spi-flash.alias", CHIPLAB_XIP_ALIAS_SIZE);

    if (s->image && s->image[0]) {
        loaded = load_image_size(s->image, s->storage, CHIPLAB_FLASH_SIZE);
        if (loaded < 0) {
            error_setg_errno(errp, errno, "could not load SPI boot image '%s'",
                             s->image);
            return;
        }
        if (loaded > CHIPLAB_XIP_SIZE) {
            warn_report("chiplab-spi: boot image '%s' is %zd bytes, larger "
                        "than the 1 MiB XIP window; only the first 1 MiB is "
                        "reachable by the CPU", s->image, loaded);
        }
        info_report("chiplab-spi: loaded %zd bytes from '%s' at flash offset 0",
                    loaded, s->image);
    }
}

/*
 * No reset handler on purpose: a CPU reset must not reload the flash, or the
 * content written through the alias window would be lost (real NOR flash
 * keeps its content across a reset).
 */
static void chiplab_spi_flash_init(Object *obj)
{
    ChiplabSpiFlashState *s = CHIPLAB_SPI_FLASH(obj);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->xip);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->alias);
}

static Property chiplab_spi_flash_props[] = {
    DEFINE_PROP_STRING("image", ChiplabSpiFlashState, image),
    DEFINE_PROP_END_OF_LIST(),
};

static void chiplab_spi_flash_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = chiplab_spi_flash_realize;
    device_class_set_props(dc, chiplab_spi_flash_props);
}

static const TypeInfo chiplab_spi_flash_type_info = {
    .name = TYPE_CHIPLAB_SPI_FLASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ChiplabSpiFlashState),
    .instance_init = chiplab_spi_flash_init,
    .class_init = chiplab_spi_flash_class_init,
};

/* ======================================================================== */
/* NAND controller (K9F1G08U0C behind the chiplab APB controller)           */
/* ======================================================================== */

#define CHIPLAB_NAND_PAGE_SIZE     2048
#define CHIPLAB_NAND_OOB_SIZE      64
#define CHIPLAB_NAND_PAGE_TOTAL    (CHIPLAB_NAND_PAGE_SIZE + CHIPLAB_NAND_OOB_SIZE)
#define CHIPLAB_NAND_PAGES_PER_BLOCK 64
#define CHIPLAB_NAND_BLOCK_SIZE    (CHIPLAB_NAND_PAGE_SIZE * CHIPLAB_NAND_PAGES_PER_BLOCK)
#define CHIPLAB_NAND_BLOCKS        1024
#define CHIPLAB_NAND_PAGES         (CHIPLAB_NAND_BLOCKS * CHIPLAB_NAND_PAGES_PER_BLOCK)
/*
 * Host image layout: raw interleaved, 2112 bytes per page
 * (page N at offset N * 2112, main area 0..2047 then spare area 2048..2111).
 * This is the usual "nanddump with OOB" layout and keeps the file self
 * describing; the chip itself is addressed by page/column.
 */
#define CHIPLAB_NAND_IMAGE_SIZE    ((uint64_t)CHIPLAB_NAND_PAGES * CHIPLAB_NAND_PAGE_TOTAL)

/* controller registers (nand.v; chiplab_nand.h §1) */
enum {
    NAND_REG_CMD        = 0x00,
    NAND_REG_ADDRL      = 0x04,
    NAND_REG_ADDRH      = 0x08,
    NAND_REG_TIMING     = 0x0c,
    NAND_REG_IDL        = 0x10,
    NAND_REG_STATUS_IDH = 0x14,
    NAND_REG_PARAM      = 0x18,
    NAND_REG_OP_NUM     = 0x1c,
    NAND_REG_CE_MAP0    = 0x20,
    NAND_REG_CE_MAP1    = 0x24,
    NAND_REG_RDY_MAP0   = 0x28,
    NAND_REG_RDY_MAP1   = 0x2c,
    NAND_REG_DOORBELL   = 0x40,
};

/* command word bits (nand.v:461..536, :180/:206/:208) */
enum {
    NAND_CMD_VALID      = 1u << 0,
    NAND_CMD_READ       = 1u << 1,
    NAND_CMD_WRITE      = 1u << 2,
    NAND_CMD_ERASE      = 1u << 3,
    NAND_CMD_ERASE_SER  = 1u << 4,
    NAND_CMD_READ_ID    = 1u << 5,
    NAND_CMD_RESET      = 1u << 6,
    NAND_CMD_READ_STAT  = 1u << 7,
    NAND_CMD_OP_MAIN    = 1u << 8,
    NAND_CMD_OP_SPARE   = 1u << 9,
    NAND_CMD_DONE       = 1u << 10,      /* read-only */
    NAND_CMD_INT_EN     = 1u << 13,
    NAND_CMD_DMA_REQ    = 1u << 31,      /* read-only */
};

#define NAND_PARAM_RESET       0x08005000u   /* nand.v:160, nand_type = 2 */
#define NAND_TIMING_RESET      0x0412u
#define NAND_PARAM_OP_SCOPE_SHIFT 16
#define NAND_PARAM_ID_NUM_SHIFT   12

#define NAND_STATUS_FAIL       0x01u         /* bit0: operation failed */
#define NAND_STATUS_READY      0x40u         /* bit6: device ready */
#define NAND_STATUS_WP         0x80u         /* bit7: 1 = not write protected */
/*
 * Status byte the controller samples: ready and writable.  The WP bit must be
 * set, otherwise U-Boot's nand_check_wp() treats the chip as write protected
 * and every erase/program fails with -EIO.
 */
#define NAND_STATUS_OK         (NAND_STATUS_READY | NAND_STATUS_WP)

/* ID bytes EC F1 00 1D 15 (Samsung K9F1G08U0C), byte 0 in ID_INFORM[7:0] */
#define NAND_ID_INFORM         0x0000151d00f1ecULL

#define TYPE_CHIPLAB_NAND "chiplab.nand"
OBJECT_DECLARE_SIMPLE_TYPE(ChiplabNandState, CHIPLAB_NAND)

struct ChiplabNandState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    /* backing store */
    char *image;             /* host file, optional */
    int fd;                  /* -1 when running without a backing file */
    uint8_t *ram;            /* backing store when fd < 0 */

    /* register file */
    uint32_t cmd;
    uint32_t addr_l;
    uint32_t addr_h;
    uint32_t timing;
    uint32_t param;
    uint32_t op_num;
    uint32_t ce_map0, ce_map1, rdy_map0, rdy_map1;
    bool ce_map1_written, rdy_map1_written;
    uint32_t doorbell;       /* last word on the device data port */

    /* operation state */
    bool done;
    uint32_t state;          /* NAND_STATE field [28:24] */
    uint8_t status;          /* last sampled status byte */
    uint64_t id_inform;      /* 48-bit ID shift register */
    uint8_t frame[CHIPLAB_NAND_PAGE_TOTAL];
    uint32_t frame_len;
    uint32_t cursor;
    bool write_pending;
    bool main_op, spare_op;
};

/* ---- backing store helpers -------------------------------------------- */

static void chiplab_nand_load(ChiplabNandState *s, uint64_t off,
                              void *buf, size_t len)
{
    if (off + len > CHIPLAB_NAND_IMAGE_SIZE) {
        memset(buf, 0xff, len);
        return;
    }
    if (s->fd < 0) {
        memcpy(buf, s->ram + off, len);
        return;
    }
    while (len) {
        ssize_t n = pread(s->fd, buf, len, off);

        if (n <= 0) {
            memset(buf, 0xff, len);
            return;
        }
        buf = (uint8_t *)buf + n;
        off += n;
        len -= n;
    }
}

static void chiplab_nand_store(ChiplabNandState *s, uint64_t off,
                               const void *buf, size_t len)
{
    if (off + len > CHIPLAB_NAND_IMAGE_SIZE) {
        return;
    }
    if (s->fd < 0) {
        memcpy(s->ram + off, buf, len);
        return;
    }
    while (len) {
        ssize_t n = pwrite(s->fd, buf, len, off);

        if (n <= 0) {
            error_report("chiplab-nand: write to '%s' failed", s->image);
            return;
        }
        buf = (const uint8_t *)buf + n;
        off += n;
        len -= n;
    }
}

static void chiplab_nand_erase_range(ChiplabNandState *s, uint64_t off,
                                     uint64_t len)
{
    static const uint8_t ff[4096] = { [0 ... 4095] = 0xff };

    while (len) {
        size_t chunk = MIN(len, sizeof(ff));

        chiplab_nand_store(s, off, ff, chunk);
        off += chunk;
        len -= chunk;
    }
}

/* ---- device state ------------------------------------------------------ */

static void chiplab_nand_reset_state(ChiplabNandState *s)
{
    s->cmd = 0;
    s->addr_l = 0;
    s->addr_h = 0;
    s->timing = NAND_TIMING_RESET;
    s->param = NAND_PARAM_RESET;
    s->op_num = 0;
    s->ce_map0 = s->rdy_map0 = 0;
    s->ce_map1 = s->rdy_map1 = 0;
    s->ce_map1_written = s->rdy_map1_written = false;
    s->doorbell = 0;
    s->done = false;
    s->state = 0;
    s->status = NAND_STATUS_OK;
    s->id_inform = 0;
    s->frame_len = 0;
    s->cursor = 0;
    s->write_pending = false;
    s->main_op = s->spare_op = false;
}

static void chiplab_nand_build_id(ChiplabNandState *s)
{
    unsigned id_num = (s->param >> NAND_PARAM_ID_NUM_SHIFT) & 0x7;
    uint64_t id = NAND_ID_INFORM;

    if (id_num == 0 || id_num > 6) {
        id_num = 5;
    }
    s->id_inform = id & (((uint64_t)1 << (8 * id_num)) - 1);
}

/*
 * Chip level reset (the 0xFF command the controller drives for CMD_RESET).
 * Only the operation state is cleared: the controller registers keep their
 * values, and the operation still completes (DONE), which is what the
 * driver's cnand_wait_done() after NAND_CMD_RESET expects.
 */
static void chiplab_nand_reset_op(ChiplabNandState *s)
{
    s->write_pending = false;
    s->main_op = s->spare_op = false;
    s->frame_len = 0;
    s->cursor = 0;
    s->status = NAND_STATUS_OK;
    s->state = 0;
    s->done = true;
}

/* Row address register holds the page number (chiplab_nand.h §5). */
static uint32_t chiplab_nand_page(ChiplabNandState *s)
{
    return s->addr_h & 0xffff;
}

static void chiplab_nand_do_read(ChiplabNandState *s)
{
    uint32_t page = chiplab_nand_page(s);
    uint32_t len = 0;

    s->main_op = s->cmd & NAND_CMD_OP_MAIN;
    s->spare_op = s->cmd & NAND_CMD_OP_SPARE;
    if (!s->main_op && !s->spare_op) {
        s->main_op = true;      /* the controller always transfers something */
    }

    if (s->main_op) {
        if (page < CHIPLAB_NAND_PAGES) {
            chiplab_nand_load(s, (uint64_t)page * CHIPLAB_NAND_PAGE_TOTAL,
                              s->frame, CHIPLAB_NAND_PAGE_SIZE);
        } else {
            memset(s->frame, 0xff, CHIPLAB_NAND_PAGE_SIZE);
        }
        len = CHIPLAB_NAND_PAGE_SIZE;
    }
    if (s->spare_op) {
        if (page < CHIPLAB_NAND_PAGES) {
            chiplab_nand_load(s, (uint64_t)page * CHIPLAB_NAND_PAGE_TOTAL +
                              CHIPLAB_NAND_PAGE_SIZE,
                              s->frame + len, CHIPLAB_NAND_OOB_SIZE);
        } else {
            memset(s->frame + len, 0xff, CHIPLAB_NAND_OOB_SIZE);
        }
        len += CHIPLAB_NAND_OOB_SIZE;
    }

    s->frame_len = len;
    s->cursor = 0;
    s->status = NAND_STATUS_OK;
    s->done = true;
}

static void chiplab_nand_do_write(ChiplabNandState *s)
{
    s->main_op = s->cmd & NAND_CMD_OP_MAIN;
    s->spare_op = s->cmd & NAND_CMD_OP_SPARE;
    if (!s->main_op && !s->spare_op) {
        s->main_op = true;
    }

    s->frame_len = (s->main_op ? CHIPLAB_NAND_PAGE_SIZE : 0) +
                   (s->spare_op ? CHIPLAB_NAND_OOB_SIZE : 0);
    s->cursor = 0;
    memset(s->frame, 0xff, sizeof(s->frame));
    s->write_pending = true;
    s->done = false;            /* completes when the last word arrives */
}

static void chiplab_nand_program_page(ChiplabNandState *s)
{
    uint32_t page = chiplab_nand_page(s);
    uint64_t off = (uint64_t)page * CHIPLAB_NAND_PAGE_TOTAL;
    uint32_t pos = 0;

    if (page >= CHIPLAB_NAND_PAGES) {
        s->status = NAND_STATUS_FAIL | NAND_STATUS_OK;
        return;
    }
    if (s->main_op) {
        chiplab_nand_store(s, off, s->frame, CHIPLAB_NAND_PAGE_SIZE);
        pos = CHIPLAB_NAND_PAGE_SIZE;
    }
    if (s->spare_op) {
        chiplab_nand_store(s, off + CHIPLAB_NAND_PAGE_SIZE,
                           s->frame + pos, CHIPLAB_NAND_OOB_SIZE);
    }
    s->status = NAND_STATUS_OK;
}

static void chiplab_nand_erase_block(ChiplabNandState *s)
{
    uint32_t block = chiplab_nand_page(s) / CHIPLAB_NAND_PAGES_PER_BLOCK;

    if (block >= CHIPLAB_NAND_BLOCKS) {
        s->status = NAND_STATUS_FAIL | NAND_STATUS_OK;
        return;
    }
    chiplab_nand_erase_range(s, (uint64_t)block * CHIPLAB_NAND_PAGES_PER_BLOCK *
                             CHIPLAB_NAND_PAGE_TOTAL,
                             (uint64_t)CHIPLAB_NAND_PAGES_PER_BLOCK *
                             CHIPLAB_NAND_PAGE_TOTAL);
    s->status = NAND_STATUS_OK;
}

/*
 * A write to 0x00 with CMD_VALID starts one controller operation.  The
 * controller then runs the chip sequence itself (0x00/0x30, 0x80/0x10,
 * 0x60/0xD0, 0x90, 0x70, 0xFF) - exactly the sequences chiplab_nand.c
 * expects the controller to drive.
 */
static void chiplab_nand_command(ChiplabNandState *s)
{
    if (!(s->cmd & NAND_CMD_VALID)) {
        return;
    }

    if (s->cmd & NAND_CMD_RESET) {
        chiplab_nand_reset_op(s);
        return;
    }
    if (s->cmd & NAND_CMD_READ_ID) {
        chiplab_nand_build_id(s);
        s->status = NAND_STATUS_OK;
        s->done = true;
        return;
    }
    if (s->cmd & NAND_CMD_READ_STAT) {
        s->status = NAND_STATUS_OK;
        s->done = true;
        return;
    }
    if (s->cmd & NAND_CMD_ERASE) {
        chiplab_nand_erase_block(s);
        s->done = true;
        return;
    }
    if (s->cmd & NAND_CMD_WRITE) {
        chiplab_nand_do_write(s);
        return;
    }
    if (s->cmd & NAND_CMD_READ) {
        chiplab_nand_do_read(s);
        return;
    }

    qemu_log_mask(LOG_UNIMP, "chiplab-nand: unhandled command word 0x%08x\n",
                  s->cmd);
    s->done = true;
}

/* Device data port: the DMA engine calls these directly (doorbell word). */
static uint32_t chiplab_nand_dma_read_word(ChiplabNandState *s)
{
    uint32_t v = 0xffffffff;

    if (s->cursor + 4 <= s->frame_len) {
        v = ldl_le_p(s->frame + s->cursor);
        s->cursor += 4;
    }
    s->doorbell = v;
    return v;
}

static void chiplab_nand_dma_write_word(ChiplabNandState *s, uint32_t value)
{
    s->doorbell = value;

    if (!s->write_pending) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "chiplab-nand: DMA write to device without a pending "
                      "WRITE command\n");
        return;
    }
    if (s->cursor + 4 <= s->frame_len) {
        stl_le_p(s->frame + s->cursor, value);
        s->cursor += 4;
    }
    if (s->cursor >= s->frame_len) {
        chiplab_nand_program_page(s);
        s->write_pending = false;
        s->done = true;
        s->state = 0;
    }
}

/* ---- MMIO -------------------------------------------------------------- */

static uint64_t chiplab_nand_read(void *opaque, hwaddr addr, unsigned size)
{
    ChiplabNandState *s = opaque;
    uint32_t v;

    switch (addr & ~0x3u) {
    case NAND_REG_CMD:
        /* nand.v:180-182: hardware fills the high bits on read-back */
        v = (s->cmd & 0xffff) |
            (s->done ? NAND_CMD_DONE : 0) |
            (0xfu << 16) |                       /* NAND_IORDY_i: ready */
            (0x0u << 20) |                       /* NAND_CE_o: CE0 asserted */
            (s->state << 24) |
            (s->write_pending ? NAND_CMD_DMA_REQ : 0);
        break;
    case NAND_REG_ADDRL:
        v = s->addr_l & 0x3fff;
        break;
    case NAND_REG_ADDRH:
        v = s->addr_h & 0x1ffffff;
        break;
    case NAND_REG_TIMING:
        v = s->timing;
        break;
    case NAND_REG_IDL:
        v = (uint32_t)s->id_inform;
        break;
    case NAND_REG_STATUS_IDH:
        /* nand.v:347: {status[7:0], ID_INFORM[47:32]} */
        v = ((uint32_t)s->status << 16) |
            (uint32_t)((s->id_inform >> 32) & 0xffff);
        break;
    case NAND_REG_PARAM:
        v = s->param;
        break;
    case NAND_REG_OP_NUM:
        v = s->op_num;
        break;
    case NAND_REG_CE_MAP0:
        v = s->ce_map0;
        break;
    case NAND_REG_CE_MAP1:
        /* nand.v:194: auto-filled with {READ_MAX_COUNT, NAND_OP_NUM[15:0]} */
        v = s->ce_map1_written ? s->ce_map1 : (s->op_num & 0xffff);
        break;
    case NAND_REG_RDY_MAP0:
        v = s->rdy_map0;
        break;
    case NAND_REG_RDY_MAP1:
        /* nand.v:197: auto-filled with {WRITE_MAX_COUNT, NAND_OP_NUM[15:0]} */
        v = s->rdy_map1_written ? s->rdy_map1 : (s->op_num & 0xffff);
        break;
    case NAND_REG_DOORBELL:
        v = s->doorbell;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "chiplab-nand: read from unimplemented "
                      "register 0x%" HWADDR_PRIx "\n", addr);
        v = 0;
        break;
    }

    return extract32(v, (addr & 3) * 8, size * 8);
}

static void chiplab_nand_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    ChiplabNandState *s = opaque;
    uint32_t v = value;

    switch (addr & ~0x3u) {
    case NAND_REG_CMD:
        s->cmd = v;
        chiplab_nand_command(s);
        break;
    case NAND_REG_ADDRL:
        s->addr_l = v & 0x3fff;
        break;
    case NAND_REG_ADDRH:
        s->addr_h = v & 0x1ffffff;
        break;
    case NAND_REG_TIMING:
        /* nand.v:186-189: write values are clamped from below */
        s->timing = (MAX(v & 0xff, 5)) | (MAX((v >> 8) & 0xff, 2) << 8);
        break;
    case NAND_REG_PARAM:
        s->param = v;
        break;
    case NAND_REG_OP_NUM:
        s->op_num = v;
        break;
    case NAND_REG_CE_MAP0:
        s->ce_map0 = v;
        break;
    case NAND_REG_CE_MAP1:
        s->ce_map1 = v;
        s->ce_map1_written = true;
        break;
    case NAND_REG_RDY_MAP0:
        s->rdy_map0 = v;
        break;
    case NAND_REG_RDY_MAP1:
        s->rdy_map1 = v;
        s->rdy_map1_written = true;
        break;
    case NAND_REG_DOORBELL:
        /*
         * CPU side doorbell access: chiplab_nand.c clears the DMA
         * acknowledge here before arming the transfer, it is not a data
         * write.  Page data only moves when the DMA engine drives the port
         * (chiplab_nand_dma_write_word()).
         */
        s->doorbell = v;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "chiplab-nand: write to unimplemented "
                      "register 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps chiplab_nand_ops = {
    .read = chiplab_nand_read,
    .write = chiplab_nand_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void chiplab_nand_reset(DeviceState *dev)
{
    ChiplabNandState *s = CHIPLAB_NAND(dev);

    chiplab_nand_reset_state(s);
    chiplab_nand_build_id(s);
}

static void chiplab_nand_realize(DeviceState *dev, Error **errp)
{
    ChiplabNandState *s = CHIPLAB_NAND(dev);
    struct stat st;
    Error *err = NULL;

    memory_region_init_io(&s->mmio, OBJECT(dev), &chiplab_nand_ops, s,
                          "chiplab.nand", CHIPLAB_NAND_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    s->fd = -1;
    if (s->image && s->image[0]) {
        s->fd = qemu_open(s->image, O_RDWR, &err);
        if (s->fd < 0 && errno == ENOENT) {
            /* first use: create the image with the full device geometry */
            error_free(err);
            err = NULL;
            s->fd = qemu_create(s->image, O_RDWR, 0644, &err);
        }
        if (s->fd < 0) {
            error_propagate(errp, err);
            return;
        }
        if (fstat(s->fd, &st) < 0) {
            error_setg_errno(errp, errno, "chiplab-nand: cannot stat '%s'",
                             s->image);
            return;
        }
        if (st.st_size > CHIPLAB_NAND_IMAGE_SIZE) {
            warn_report("chiplab-nand: image '%s' is larger than the device "
                        "(%" PRIu64 " > %" PRIu64 " bytes); using the first "
                        "%" PRIu64 " bytes", s->image, (uint64_t)st.st_size,
                        CHIPLAB_NAND_IMAGE_SIZE, CHIPLAB_NAND_IMAGE_SIZE);
        } else if (st.st_size < CHIPLAB_NAND_IMAGE_SIZE) {
            /* grow to full geometry and mark the new area as erased */
            if (ftruncate(s->fd, CHIPLAB_NAND_IMAGE_SIZE) < 0) {
                error_setg_errno(errp, errno, "chiplab-nand: cannot grow '%s'",
                                 s->image);
                return;
            }
            if (st.st_size == 0) {
                chiplab_nand_erase_range(s, 0, CHIPLAB_NAND_IMAGE_SIZE);
            } else {
                chiplab_nand_erase_range(s, st.st_size,
                                         CHIPLAB_NAND_IMAGE_SIZE - st.st_size);
            }
            info_report("chiplab-nand: initialised '%s' (%" PRIu64 " bytes, "
                        "erased)", s->image, CHIPLAB_NAND_IMAGE_SIZE);
        }
    } else {
        s->ram = g_malloc(CHIPLAB_NAND_IMAGE_SIZE);
        memset(s->ram, 0xff, CHIPLAB_NAND_IMAGE_SIZE);
        info_report("chiplab-nand: no backing image, writes are volatile "
                    "(use -machine chiplab,nand-image=<file> to persist)");
    }

    chiplab_nand_reset(dev);
}

static void chiplab_nand_init(Object *obj)
{
}

static Property chiplab_nand_props[] = {
    DEFINE_PROP_STRING("image", ChiplabNandState, image),
    DEFINE_PROP_END_OF_LIST(),
};

static void chiplab_nand_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = chiplab_nand_realize;
    device_class_set_legacy_reset(dc, chiplab_nand_reset);
    device_class_set_props(dc, chiplab_nand_props);
}

static const TypeInfo chiplab_nand_type_info = {
    .name = TYPE_CHIPLAB_NAND,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ChiplabNandState),
    .instance_init = chiplab_nand_init,
    .class_init = chiplab_nand_class_init,
};

/* ======================================================================== */
/* confreg block: platform DMA engine order register                        */
/* ======================================================================== */

#define CHIPLAB_DMA_ORDER_ASK    (1u << 2)   /* dma.v:136 */
#define CHIPLAB_DMA_ORDER_START  (1u << 3)   /* dma.v:137 */
#define CHIPLAB_DMA_ORDER_STOP   (1u << 4)
#define CHIPLAB_DMA_ORDER_ADDR_MASK 0xffffffe0u
#define CHIPLAB_DMA_CMD_RW       (1u << 12)  /* dma.v:572 */
#define CHIPLAB_DMA_NAND_DEV_ADDR 0x1fe78040u

#define TYPE_CHIPLAB_DMA "chiplab.dma"
OBJECT_DECLARE_SIMPLE_TYPE(ChiplabDmaState, CHIPLAB_DMA)

struct ChiplabDmaState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t order;
    ChiplabNandState *nand;
};

/*
 * Descriptor (4 x 64 bit, 32 byte aligned), as packed by
 * chiplab_nand_dma_desc_pack() in chiplab_nand.h §8 (32-bit half word view):
 *
 *   desc[0] order_addr   desc[1] mem_addr (DDR buffer)
 *   desc[2] dev_addr     desc[3] length in 4-byte words
 *   desc[4] step_length  desc[5] step_times (must be 1)
 *   desc[6] cmd          desc[7] status (filled in by the engine)
 *
 * The engine reads the descriptor over AXI and moves `length` words between
 * DDR and the device data port; cmd[12] selects the DDR side direction
 * (0 = device -> memory, 1 = memory -> device).
 */
static void chiplab_dma_run(ChiplabDmaState *s, uint32_t desc_addr)
{
    uint32_t desc[8];
    uint32_t mem_addr, dev_addr, words, steps, cmd;
    bool to_device;
    uint32_t status = 0;
    uint32_t i, step;

    address_space_read(&address_space_memory, desc_addr,
                       MEMTXATTRS_UNSPECIFIED, desc, sizeof(desc));

    mem_addr = desc[1];
    dev_addr = desc[2];
    words = desc[3];
    steps = desc[5];
    cmd = desc[6];
    to_device = cmd & CHIPLAB_DMA_CMD_RW;

    if (steps == 0) {
        steps = 1;
    }
    if (dev_addr != CHIPLAB_DMA_NAND_DEV_ADDR) {
        qemu_log_mask(LOG_UNIMP, "chiplab-dma: unsupported device address "
                      "0x%08x (only the NAND doorbell is modelled)\n",
                      dev_addr);
        status = 1;
        words = 0;
        steps = 1;
    }

    for (step = 0; step < steps; step++) {
        for (i = 0; i < words; i++) {
            uint32_t addr = mem_addr + i * 4;

            if (to_device) {
                uint32_t word;

                address_space_read(&address_space_memory, addr,
                                   MEMTXATTRS_UNSPECIFIED, &word, 4);
                chiplab_nand_dma_write_word(s->nand, word);
            } else {
                uint32_t word = chiplab_nand_dma_read_word(s->nand);

                address_space_write(&address_space_memory, addr,
                                    MEMTXATTRS_UNSPECIFIED, &word, 4);
            }
        }
    }

    /*
     * dma.v:583-584: the engine writes the completion status back into
     * descriptor word 3, high half (bytes 28..31 - a single 32 bit word; the
     * data buffer starts at byte 32 and must not be touched).
     */
    address_space_write(&address_space_memory, desc_addr + 28,
                        MEMTXATTRS_UNSPECIFIED, &status, sizeof(status));
}

static uint64_t chiplab_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    ChiplabDmaState *s = opaque;

    if (addr != CHIPLAB_DMA_ORDER_OFF) {
        qemu_log_mask(LOG_UNIMP, "chiplab-confreg: read from unmodelled "
                      "register 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
    return extract32(s->order, (addr & 3) * 8, size * 8);
}

static void chiplab_dma_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    ChiplabDmaState *s = opaque;
    uint32_t v = value;

    if (addr != CHIPLAB_DMA_ORDER_OFF) {
        qemu_log_mask(LOG_UNIMP, "chiplab-confreg: write to unmodelled "
                      "register 0x%" HWADDR_PRIx "\n", addr);
        return;
    }

    if (v & CHIPLAB_DMA_ORDER_START) {
        uint32_t desc_addr = v & CHIPLAB_DMA_ORDER_ADDR_MASK;

        /*
         * confreg_syn.v:326-329: dma_start is cleared once the descriptor has
         * been fetched, ask_valid is cleared when the transfer finishes.  The
         * transfer is synchronous here, so the guest sees both bits clear by
         * the time the polling loop in chiplab_nand.c reads the register.
         */
        s->order = desc_addr | CHIPLAB_DMA_ORDER_ASK;
        chiplab_dma_run(s, desc_addr);
        s->order = desc_addr & ~(CHIPLAB_DMA_ORDER_START |
                                 CHIPLAB_DMA_ORDER_ASK);
    } else {
        s->order = v & ~(CHIPLAB_DMA_ORDER_START | CHIPLAB_DMA_ORDER_STOP);
    }
}

static const MemoryRegionOps chiplab_dma_ops = {
    .read = chiplab_dma_read,
    .write = chiplab_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void chiplab_dma_realize(DeviceState *dev, Error **errp)
{
    ChiplabDmaState *s = CHIPLAB_DMA(dev);

    if (!s->nand) {
        error_setg(errp, "chiplab-dma: no NAND device attached");
        return;
    }
    memory_region_init_io(&s->mmio, OBJECT(dev), &chiplab_dma_ops, s,
                          "chiplab.confreg", CHIPLAB_CONFREG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
}

static void chiplab_dma_reset(DeviceState *dev)
{
    ChiplabDmaState *s = CHIPLAB_DMA(dev);

    s->order = 0;
}

static void chiplab_dma_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = chiplab_dma_realize;
    device_class_set_legacy_reset(dc, chiplab_dma_reset);
}

static const TypeInfo chiplab_dma_type_info = {
    .name = TYPE_CHIPLAB_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ChiplabDmaState),
    .class_init = chiplab_dma_class_init,
};

/* ======================================================================== */
/* Machine                                                                  */
/* ======================================================================== */

#define TYPE_RISCV_CHIPLAB_MACHINE MACHINE_TYPE_NAME("chiplab")
OBJECT_DECLARE_SIMPLE_TYPE(ChiplabMachineState, RISCV_CHIPLAB_MACHINE)

struct ChiplabMachineState {
    MachineState parent_obj;

    RISCVHartArrayState soc;
    ChiplabSpiFlashState flash;
    ChiplabNandState nand;
    ChiplabDmaState dma;
    DeviceState *plic;

    char *nand_image;
};

static void chiplab_machine_init(MachineState *machine)
{
    ChiplabMachineState *s = RISCV_CHIPLAB_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    char *plic_hart_config;
    BlockBackend *pflash_blk;
    DriveInfo *dinfo;
    const char *boot_image = machine->firmware;

    /* DDR3 at 0x0000_0000 (128 MiB) */
    if (machine->ram_size > CHIPLAB_DDR_SIZE) {
        error_report("chiplab: RAM size 0x%" PRIx64 " exceeds the platform "
                     "DDR3 window (0x%" PRIx64 ")", (uint64_t)machine->ram_size,
                     (uint64_t)CHIPLAB_DDR_SIZE);
        exit(1);
    }
    memory_region_add_subregion(system_memory, chiplab_memmap[CHIPLAB_DEV_DDR].base,
                                machine->ram);

    /* single RV32 hart, reset PC in the SPI XIP window */
    object_property_set_str(OBJECT(&s->soc), "cpu-type", machine->cpu_type,
                            &error_abort);
    object_property_set_int(OBJECT(&s->soc), "num-harts", machine->smp.cpus,
                            &error_abort);
    object_property_set_int(OBJECT(&s->soc), "hartid-base", 0, &error_abort);
    object_property_set_int(OBJECT(&s->soc), "resetvec", CHIPLAB_RESET_PC,
                            &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);

    /* CLINT: SiFive layout inside the core (mtimecmp 0x4000, mtime 0xbff8) */
    riscv_aclint_swi_create(chiplab_memmap[CHIPLAB_DEV_CLINT].base, 0,
                            machine->smp.cpus, false);
    riscv_aclint_mtimer_create(chiplab_memmap[CHIPLAB_DEV_CLINT].base +
                               RISCV_ACLINT_SWI_SIZE,
                               RISCV_ACLINT_DEFAULT_MTIMER_SIZE, 0,
                               machine->smp.cpus,
                               RISCV_ACLINT_DEFAULT_MTIMECMP,
                               RISCV_ACLINT_DEFAULT_MTIME,
                               CHIPLAB_TIMEBASE_FREQ, true);

    /* PLIC with 31 sources, M and S contexts per hart */
    plic_hart_config = riscv_plic_hart_config_string(machine->smp.cpus);
    s->plic = sifive_plic_create(chiplab_memmap[CHIPLAB_DEV_PLIC].base,
                                 plic_hart_config, machine->smp.cpus, 0,
                                 CHIPLAB_PLIC_NUM_SOURCES,
                                 (1U << CHIPLAB_PLIC_NUM_PRIO_BITS) - 1,
                                 CHIPLAB_PLIC_PRIORITY_BASE,
                                 CHIPLAB_PLIC_PENDING_BASE,
                                 CHIPLAB_PLIC_ENABLE_BASE,
                                 CHIPLAB_PLIC_ENABLE_STRIDE,
                                 CHIPLAB_PLIC_CONTEXT_BASE,
                                 CHIPLAB_PLIC_CONTEXT_STRIDE,
                                 chiplab_memmap[CHIPLAB_DEV_PLIC].size);
    g_free(plic_hart_config);

    /* 16550 UART, 8-bit registers (reg-shift 0), 33 MHz */
    serial_mm_init(system_memory, chiplab_memmap[CHIPLAB_DEV_UART0].base, 0,
                   qdev_get_gpio_in(DEVICE(s->plic), CHIPLAB_UART0_IRQ),
                   CHIPLAB_TIMEBASE_FREQ, serial_hd(0), DEVICE_LITTLE_ENDIAN);

    /*
     * The SPI boot image comes from -bios (preferred).  -drive
     * if=pflash,format=raw,unit=0,file=... is accepted as an alternative,
     * which is handy because that is how flash images are usually fed in.
     */
    object_initialize_child(OBJECT(machine), "spi-flash", &s->flash,
                            TYPE_CHIPLAB_SPI_FLASH);
    if (boot_image) {
        qdev_prop_set_string(DEVICE(&s->flash), "image", boot_image);
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->flash), &error_fatal);

    if (!boot_image) {
        dinfo = drive_get(IF_PFLASH, 0, 0);
        pflash_blk = dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
        if (pflash_blk && blk_getlength(pflash_blk) > 0) {
            int64_t len = MIN(blk_getlength(pflash_blk),
                              (int64_t)CHIPLAB_FLASH_SIZE);
            uint8_t *buf = g_malloc(len);

            if (blk_pread(pflash_blk, 0, len, buf, 0) < 0) {
                error_report("chiplab: cannot read the -drive pflash image");
                exit(1);
            }
            memcpy(s->flash.storage, buf, len);
            g_free(buf);
            info_report("chiplab-spi: loaded %" PRId64 " bytes from the "
                        "pflash drive at flash offset 0", len);
        } else {
            warn_report("chiplab: no SPI boot image given (-bios "
                        "<spi_flash.img>); the XIP window reads as erased "
                        "flash");
        }
    }

    sysbus_mmio_map(SYS_BUS_DEVICE(&s->flash), 0, chiplab_memmap[CHIPLAB_DEV_SPI_XIP].base);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->flash), 1, chiplab_memmap[CHIPLAB_DEV_SPI_ALIAS].base);

    /* NAND controller + platform DMA engine (confreg order register) */
    object_initialize_child(OBJECT(machine), "nand", &s->nand, TYPE_CHIPLAB_NAND);
    if (s->nand_image && s->nand_image[0]) {
        qdev_prop_set_string(DEVICE(&s->nand), "image", s->nand_image);
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->nand), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->nand), 0, chiplab_memmap[CHIPLAB_DEV_NAND].base);

    object_initialize_child(OBJECT(machine), "dma", &s->dma, TYPE_CHIPLAB_DMA);
    s->dma.nand = &s->nand;
    sysbus_realize(SYS_BUS_DEVICE(&s->dma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dma), 0, chiplab_memmap[CHIPLAB_DEV_CONFREG].base);

    /*
     * Optional DDR payloads: -kernel / -initrd / -dtb are loaded into RAM
     * before reset so that U-Boot can boot them (see the address layout in the
     * CHIPLAB_*_ADDR comment above).  Nothing is patched: the FDT comes from
     * -dtb (or from the SPI image for the U-Boot stage) and the kernel command
     * line is the one in the FDT /chosen.
     */
    if (machine->kernel_filename) {
        ssize_t klen = load_image_targphys(machine->kernel_filename,
                                           CHIPLAB_KERNEL_ADDR,
                                           CHIPLAB_KERNEL_MAX);
        if (klen < 0) {
            error_report("chiplab: could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }
        info_report("chiplab: kernel '%s' loaded at 0x%08x (%zd bytes)",
                    machine->kernel_filename, CHIPLAB_KERNEL_ADDR, klen);
    }
    if (machine->initrd_filename) {
        ssize_t ilen = load_image_targphys(machine->initrd_filename,
                                           CHIPLAB_INITRD_ADDR,
                                           CHIPLAB_INITRD_MAX);
        if (ilen < 0) {
            error_report("chiplab: could not load initrd '%s'",
                         machine->initrd_filename);
            exit(1);
        }
        info_report("chiplab: initrd '%s' loaded at 0x%08x (%zd bytes); "
                    "use 'setenv filesize 0x%zx' + "
                    "'booti 0x%08x 0x%08x:${filesize} 0x%08x'",
                    machine->initrd_filename, CHIPLAB_INITRD_ADDR, ilen,
                    (size_t)ilen, CHIPLAB_KERNEL_ADDR, CHIPLAB_INITRD_ADDR,
                    CHIPLAB_DTB_ADDR);
    }
    if (machine->dtb) {
        ssize_t dlen = load_image_targphys(machine->dtb, CHIPLAB_DTB_ADDR,
                                           CHIPLAB_DTB_MAX);
        if (dlen < 0) {
            error_report("chiplab: could not load dtb '%s'", machine->dtb);
            exit(1);
        }
        info_report("chiplab: dtb '%s' loaded at 0x%08x (%zd bytes)",
                    machine->dtb, CHIPLAB_DTB_ADDR, dlen);
    }
}

static char *chiplab_machine_get_nand_image(Object *obj, Error **errp)
{
    ChiplabMachineState *s = RISCV_CHIPLAB_MACHINE(obj);

    return g_strdup(s->nand_image ? s->nand_image : "");
}

static void chiplab_machine_set_nand_image(Object *obj, const char *value,
                                           Error **errp)
{
    ChiplabMachineState *s = RISCV_CHIPLAB_MACHINE(obj);

    g_free(s->nand_image);
    s->nand_image = g_strdup(value);
}

static void chiplab_machine_instance_init(Object *obj)
{
    ChiplabMachineState *s = RISCV_CHIPLAB_MACHINE(obj);

    object_initialize_child(obj, "soc", &s->soc, TYPE_RISCV_HART_ARRAY);
}

static void chiplab_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Loongson Chiplab RV32 (Artix-7 A200T FPGA SoC)";
    mc->init = chiplab_machine_init;
    mc->default_cpu_type = TYPE_RISCV_CPU_BASE;
    mc->default_ram_size = CHIPLAB_DDR_SIZE;
    mc->default_ram_id = "chiplab.ddr";
    mc->max_cpus = 1;
    mc->min_cpus = 1;
    mc->default_cpus = 1;

    object_class_property_add_str(oc, "nand-image",
                                  chiplab_machine_get_nand_image,
                                  chiplab_machine_set_nand_image);
    object_class_property_set_description(oc, "nand-image",
        "Host file used as the 128 MiB NAND image (raw interleaved layout, "
        "2112 bytes per page); created and erased on first use. Enables "
        "saveenv persistence across QEMU runs.");
}

static const TypeInfo chiplab_machine_typeinfo = {
    .name = TYPE_RISCV_CHIPLAB_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(ChiplabMachineState),
    .instance_init = chiplab_machine_instance_init,
    .class_init = chiplab_machine_class_init,
};

static void chiplab_machine_register_types(void)
{
    type_register_static(&chiplab_spi_flash_type_info);
    type_register_static(&chiplab_nand_type_info);
    type_register_static(&chiplab_dma_type_info);
    type_register_static(&chiplab_machine_typeinfo);
}

type_init(chiplab_machine_register_types)
