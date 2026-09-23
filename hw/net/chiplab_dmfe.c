// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Loongson Chiplab FPGA SoC - DEC 21140 "Tulip" compatible MAC (MMIO model)
 *
 * 硬件口径唯一真源：chiplab/IP/MAC/ 下的 Verilog 源码：
 *   csr.v        CSR0..CSR11 读写语义、W1C 状态位、中断汇总
 *   rlsm.v/rc.v  接收描述符、帧长覆写、地址过滤
 *   tlsm.v       发送描述符、SETUP 帧写过滤 RAM、发送完成状态
 *   rstc.v       软复位握手
 *   maccsr2axi.v CSRADDRESSWIDTH=8 ⇒ CSR 地址只用 addr[7:0]
 *
 * 与 hw/net/tulip.c（PCI 版 21143）的关系：寄存器名字/位序沿用同一套
 * Tulip 口径，但凡本 RTL 与标准 Tulip 不同的地方一律以 RTL 为准，并在
 * 相应位置标注 RTL 出处。差异清单见 docs/linux-port/mac-tftp-boot.md
 * 的"QEMU 模型 ↔ RTL 逐项对齐表"。
 *
 * U-Boot 侧驱动：u-boot/drivers/net/chiplab_dmfe.c（配套，非 PCI）。
 */

#include "qemu/osdep.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/net/chiplab_dmfe.h"
#include "net/net.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(ChiplabDmfeState, CHIPLAB_DMFE)

/* ------------------------------------------------------------------ */
/* 寄存器偏移：CSR n 位于 n*8（utility.v:77-134 的 CSR*_ID = n*2，      */
/* csr.v:458 用 csraddr[7:2] 选 CSR ⇒ 字节偏移 = ID*4 = n*8）           */
/* ------------------------------------------------------------------ */
#define CSR0    0x00
#define CSR1    0x08
#define CSR2    0x10
#define CSR3    0x18
#define CSR4    0x20
#define CSR5    0x28
#define CSR6    0x30
#define CSR7    0x38
#define CSR8    0x40
#define CSR9    0x48
#define CSR10   0x50
#define CSR11   0x58
#define CSR_MAX 0x60

/* CSR0（csr.v:1150-1152 读回；571-597 写；CSR0_RV=32'hFE00_0000） */
#define CSR0_SWR        (1u << 0)
#define CSR0_BAR        (1u << 1)
#define CSR0_DSL_SHIFT  2
#define CSR0_DSL_MASK   (0x1fu << CSR0_DSL_SHIFT)
#define CSR0_BLE        (1u << 7)
#define CSR0_PBL_SHIFT  8
#define CSR0_PBL_MASK   (0x3fu << CSR0_PBL_SHIFT)
#define CSR0_TAP_SHIFT  17
#define CSR0_TAP_MASK   (0x7u << CSR0_TAP_SHIFT)
#define CSR0_DBO        (1u << 20)
/* 读回恒定的 0xFE00_0000（csr.v:1150 把 CSR0_RV[31:21] 原样拼接） */
#define CSR0_RO_MASK    0xfe000000u

/* CSR5（csr.v:1153-1156 读回；746-798 W1C；805-860 硬件置位） */
#define CSR5_TI         (1u << 0)
#define CSR5_TPS        (1u << 1)
#define CSR5_TU         (1u << 2)
#define CSR5_UNF        (1u << 5)
#define CSR5_RI         (1u << 6)
#define CSR5_RU         (1u << 7)
#define CSR5_RPS        (1u << 8)
#define CSR5_ETI        (1u << 10)
#define CSR5_GTE        (1u << 11)
#define CSR5_ERI        (1u << 14)
#define CSR5_AIS        (1u << 15)
#define CSR5_NIS        (1u << 16)
#define CSR5_RS_SHIFT   17
#define CSR5_TS_SHIFT   20
#define CSR5_RO_MASK    0xf0000000u   /* CSR5_RV[31:28]=1111 */
/* W1C 位集合（csr.v:781-798） */
#define CSR5_W1C        (CSR5_TI | CSR5_TPS | CSR5_TU | CSR5_UNF | CSR5_RI | \
                         CSR5_RU | CSR5_RPS | CSR5_ETI | CSR5_GTE | \
                         CSR5_ERI | CSR5_AIS | CSR5_NIS)

/* CSR6（csr.v:1156-1163 读回；893-950 写入） */
#define CSR6_HP         (1u << 0)
#define CSR6_SR         (1u << 1)
#define CSR6_HO         (1u << 2)
#define CSR6_PB         (1u << 3)
#define CSR6_IF         (1u << 4)
#define CSR6_PR         (1u << 6)
#define CSR6_PM         (1u << 7)
#define CSR6_FD         (1u << 9)
#define CSR6_ST         (1u << 13)
#define CSR6_TR_MASK    (3u << 14)
#define CSR6_RA         (1u << 30)   /* receive all（csr.v:899 csr6_ra <= csrdata_c[30]） */
#define CSR6_SF         (1u << 21)   /* csr.v:906 csr6_sf <= csrdata_c[21] */
#define CSR6_TTM        (1u << 22)   /* csr.v:903 csr6_ttm <= csrdata_c[22] */
/* 读回恒定 1 的位：csr.v:1159-1164 的拼接是 32 bit（csr6_tr 是 2 bit），
 * RV[29]/RV[28]/RV[25] 分别落在 bit29/28/25；PR 复位值 RV[6]=1（csr.v:886）
 * ⇒ CSR6 复位读值 = 0x3200_0040（= CSR6_RV，utility.v:109 自洽） */
#define CSR6_RO_MASK    0x32000000u
/* 写路径只认这些位（csr.v:893-950） */
#define CSR6_WR_MASK    (CSR6_RA | CSR6_TTM | CSR6_SF | CSR6_TR_MASK | \
                         CSR6_ST | CSR6_FD | CSR6_PM | CSR6_PR | CSR6_PB | \
                         CSR6_SR)

/* CSR7（csr.v:1166-1169 读回；CSR7_RV=32'hF3FE_0000；bit9 是 CSR6_RV[9]=1） */
#define CSR7_TIE        (1u << 0)
#define CSR7_TSE        (1u << 1)
#define CSR7_TUE        (1u << 2)
#define CSR7_UNE        (1u << 5)
#define CSR7_RIE        (1u << 6)
#define CSR7_RUE        (1u << 7)
#define CSR7_RSE        (1u << 8)
#define CSR7_ETE        (1u << 10)
#define CSR7_GTE        (1u << 11)
#define CSR7_ERE        (1u << 14)
#define CSR7_AIE        (1u << 15)
#define CSR7_NIE        (1u << 16)
/* 读回固定部分 = CSR7_RV & ~(所有可写使能位)：bit14(ERE) 等是寄存器驱动的，
 * 不能算只读；bit9 由 CSR6_RV[9]=0 驱动（RTL 笔误，读 0） */
#define CSR7_RO_MASK    (0xf3fe0000u & ~(CSR7_NIE | CSR7_AIE | CSR7_ERE | \
                         CSR7_GTE | CSR7_ETE | CSR7_RSE | CSR7_RUE | \
                         CSR7_RIE | CSR7_UNE | CSR7_TUE | CSR7_TSE | \
                         CSR7_TIE))
#define CSR7_WR_MASK    (CSR7_NIE | CSR7_AIE | CSR7_ERE | CSR7_GTE | CSR7_ETE | \
                         CSR7_RSE | CSR7_RUE | CSR7_RIE | CSR7_UNE | \
                         CSR7_TUE | CSR7_TSE | CSR7_TIE)

/* CSR8（csr.v:1170-1172 读回：{RV[31:29], oco, foc[10:0], mfo, mfc[15:0]}） */
#define CSR8_RO_MASK    0xe0000000u
#define CSR8_MFC_MASK   0x0000ffffu
#define CSR8_FOC_SHIFT  17
#define CSR8_FOC_MASK   (0x7ffu << CSR8_FOC_SHIFT)

/* CSR9（csr.v:1174-1176 读回；1044-1071 写；CSR9_RV=32'hFFF4_83FF） */
#define CSR9_SCS        (1u << 0)    /* SROM chip select（本模型不接 EEPROM） */
#define CSR9_SCLK       (1u << 1)
#define CSR9_SDI        (1u << 2)
#define CSR9_SDO        (1u << 3)
#define CSR9_MDC        (1u << 16)   /* 软件直接翻转 MDC 电平 */
#define CSR9_MDO        (1u << 17)
#define CSR9_MDEN       (1u << 18)   /* 1 = MAC 驱动 MDIO */
#define CSR9_MDI        (1u << 19)   /* PHY 驱动 */
#define CSR9_RO_MASK    0xfff083f0u  /* RV[31:20]=0xFFF、RV[15:4]=0x83F；
                                      * bit19..16 是 mdi/mii/mdo/mdc，不在只读掩码里 */

/* CSR10（csr.v:1178） */
#define CSR10_INSERT_EN (1u << 0)

/* CSR11（csr.v:1180；CSR11_RV=32'hFFFE_0000，本模型按"占位"实现：只存回
 * 复位装载值，不做计数——见对齐表） */
#define CSR11_RO_MASK   0xfffe0000u

/* 描述符控制/状态位（tlsm.v:1552-1557、rlsm.v:545） */
#define DESC_OWN        (1u << 31)
#define TDES1_IC        (1u << 31)
#define TDES1_LS        (1u << 30)
#define TDES1_FS        (1u << 29)
#define TDES1_SET       (1u << 27)
#define TDES1_TER       (1u << 25)
#define TDES1_TCH       (1u << 24)
#define TDES1_BS1_MASK  0x7ffu
#define RDES0_FF        (1u << 30)
#define RDES0_LEN_SHIFT 16
#define RDES0_LEN_MASK  0x3fffu
#define RDES0_ES        (1u << 15)
#define RDES0_RDE       (1u << 14)
#define RDES0_RF        (1u << 11)
#define RDES0_MF        (1u << 10)
#define RDES0_RFS       (1u << 9)
#define RDES0_RLS       (1u << 8)
#define RDES0_TL        (1u << 7)
#define RDES0_CS        (1u << 6)
#define RDES0_FTP       (1u << 5)
#define RDES0_RE        (1u << 3)
#define RDES0_DB        (1u << 2)
#define RDES0_CE        (1u << 1)
#define RDES0_OV        (1u << 0)
#define RDES1_RER       (1u << 25)
#define RDES1_RCH       (1u << 24)
#define RDES1_BS1_MASK  0x7ffu

/* 打开后把每帧 TX/RX 打到 stderr（默认关：一次 Linux 启动有上万帧） */
/* #define CHIPLAB_DMFE_TRACE 1 */
#ifdef CHIPLAB_DMFE_TRACE
#define DMFE_TRACE(...) qemu_log_mask(LOG_TRACE, __VA_ARGS__)
#else
#define DMFE_TRACE(...) do { } while (0)
#endif

#define DMFE_DESC_SIZE  16
#define DMFE_FILTER_LEN 64           /* 64 x 16 地址过滤 RAM（ADDRDEPTH=6） */
#define DMFE_MAX_FRAME  2048
#define DMFE_PHY_ADDR   1            /* 板级 PHYAD=1 */

/* MII PHY 寄存器（通用 MII PHY，型号"未核实"，见对齐表 §PHY） */
#define PHY_BMCR        0
#define PHY_BMSR        1
#define PHY_ID1         2
#define PHY_ID2         3
#define PHY_ANAR        4
#define PHY_ANLPAR      5
#define PHY_ANER        6
#define PHY_REG_NUM     32
#define PHY_ID1_VAL     0x0181       /* 自选占位值：未核实板卡 PHY 型号 */
#define PHY_ID2_VAL     0x0000
#define PHY_BMSR_VAL    0x786d       /* 100/10 FD+HD、autoneg able/complete、link up */

struct ChiplabDmfeState {
    SysBusDevice parent_obj;

    NICConf conf;
    NICState *nic;
    MemoryRegion mmio;
    qemu_irq irq;

    /* CSR 状态 */
    uint32_t csr0;               /* 可写位（swr 见下） */
    uint32_t csr0_swr;
    int64_t swr_until_ns;        /* 软复位握手窗口（rstc.v:104-118） */
    uint32_t csr3;               /* RX 环基址 */
    uint32_t csr4;               /* TX 环基址 */
    uint32_t csr5;               /* W1C 状态位 */
    uint32_t csr6;
    uint32_t csr7;
    uint32_t csr8_mfc;           /* 丢失帧计数（只读） */
    uint32_t csr8_foc;
    uint32_t csr9;               /* MIIM/SROM 位 */
    uint32_t csr10;
    uint32_t csr11;
    uint8_t ft;                  /* 过滤类型：SETUP 帧 TDES1[28:22] 采样
                                  * （tlsm.v:763-766），驱动 CSR6 IF/HO/HP */

    /* 过滤 RAM（64 x 16 bit，tlsm.v:2126-2160 写入 / rc.v:1454-1470 比对） */
    uint16_t filter[DMFE_FILTER_LEN];

    /* 描述符环当前位置 */
    uint32_t tx_desc;            /* 当前发送描述符地址 */
    uint32_t rx_desc;            /* 当前接收描述符地址 */

    /* 接收暂存：QEMU 后端一次给一帧 */
    uint8_t rx_frame[DMFE_MAX_FRAME];
    size_t rx_len;

    /* MIIM 软件位拍状态机（MDC 上升沿采样 MDO / 驱动 MDI） */
    uint16_t phy_regs[PHY_REG_NUM];
    uint32_t mdio_preamble;
    bool mdio_in_frame;
    int mdio_nbits;
    uint32_t mdio_frame;
    uint32_t mdio_data;
    bool mdio_read;
    int mdio_phyad;
    int mdio_regad;
};

/* ------------------------------------------------------------------ */
/* 访存：描述符/缓冲都在 DDR（本板 U-Boot 无 MMU，地址即物理地址）      */
/* ------------------------------------------------------------------ */
static uint32_t dmfe_ldl(ChiplabDmfeState *s, uint32_t addr)
{
    return address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

static void dmfe_stl(ChiplabDmfeState *s, uint32_t addr, uint32_t val)
{
    address_space_stl_le(&address_space_memory, addr, val,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

static void dmfe_read_buf(ChiplabDmfeState *s, uint32_t addr, void *buf,
                          size_t len)
{
    MemTxResult r = address_space_read(&address_space_memory, addr,
                                       MEMTXATTRS_UNSPECIFIED, buf, len);

    if (r != MEMTX_OK) {
        memset(buf, 0, len);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "chiplab-dmfe: cannot read 0x%zx bytes at 0x%08x\n",
                      len, addr);
    }
}

static void dmfe_write_buf(ChiplabDmfeState *s, uint32_t addr, const void *buf,
                           size_t len)
{
    MemTxResult r = address_space_write(&address_space_memory, addr,
                                        MEMTXATTRS_UNSPECIFIED, buf, len);

    if (r != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "chiplab-dmfe: cannot write 0x%zx bytes at 0x%08x\n",
                      len, addr);
    }
}

/* ------------------------------------------------------------------ */
/* 中断：csr.v:2364-2366  level = (nis & nie) | (ais & aie)            */
/* ------------------------------------------------------------------ */
static bool dmfe_nis(ChiplabDmfeState *s)
{
    return ((s->csr5 & CSR5_RI) && (s->csr7 & CSR7_RIE)) ||
           ((s->csr5 & CSR5_TI) && (s->csr7 & CSR7_TIE)) ||
           ((s->csr5 & CSR5_ERI) && (s->csr7 & CSR7_ERE)) ||
           ((s->csr5 & CSR5_TU) && (s->csr7 & CSR7_TUE)) ||
           ((s->csr5 & CSR5_GTE) && (s->csr7 & CSR7_GTE));
}

static bool dmfe_ais(ChiplabDmfeState *s)
{
    return ((s->csr5 & CSR5_ETI) && (s->csr7 & CSR7_ETE)) ||
           ((s->csr5 & CSR5_RPS) && (s->csr7 & CSR7_RSE)) ||
           ((s->csr5 & CSR5_RU) && (s->csr7 & CSR7_RUE)) ||
           ((s->csr5 & CSR5_UNF) && (s->csr7 & CSR7_UNE)) ||
           ((s->csr5 & CSR5_TPS) && (s->csr7 & CSR7_TSE));
}

static void dmfe_update_irq(ChiplabDmfeState *s)
{
    qemu_set_irq(s->irq, (dmfe_nis(s) && (s->csr7 & CSR7_NIE)) ||
                         (dmfe_ais(s) && (s->csr7 & CSR7_AIE)));
}

/* ------------------------------------------------------------------ */
/* 描述符步进：tlsm.v:826-834（TCH/TER/DSL）、rlsm.v:1028-1046        */
/*   链模式（TCH/RCH）⇒ 用描述符第 4 字作 next；                       */
/*   环尾（TER/RER）⇒ 回到 CSR4/CSR3 基址；                            */
/*   否则 ⇒ 当前 + 16 + (dsl<<2)（Tulip 步长定义；本模型 dsl 默认 0）  */
/* ------------------------------------------------------------------ */
static uint32_t dmfe_next_desc(ChiplabDmfeState *s, uint32_t cur,
                               uint32_t des1, uint32_t next, bool tx)
{
    uint32_t dsl = (s->csr0 & CSR0_DSL_MASK) >> CSR0_DSL_SHIFT;

    if (tx) {
        if (des1 & TDES1_TER) {
            return s->csr4;
        }
        if (des1 & TDES1_TCH) {
            return next;
        }
    } else {
        if (des1 & RDES1_RER) {
            return s->csr3;
        }
        if (des1 & RDES1_RCH) {
            return next;
        }
    }

    return cur + DMFE_DESC_SIZE + (dsl << 2);
}

/* ------------------------------------------------------------------ */
/* 发送：tlsm.v（SETUP 帧写过滤 RAM，普通帧发到线路上）                */
/* ------------------------------------------------------------------ */
static void dmfe_tx_setup_frame(ChiplabDmfeState *s, uint32_t buf, int len,
                                uint32_t des1)
{
    int words = len / 4;
    int i;
    uint8_t data[DMFE_MAX_FRAME];

    if (len <= 0 || len > DMFE_MAX_FRAME) {
        return;
    }
    dmfe_read_buf(s, buf, data, len);

    /*
     * tlsm.v:2126-2160：DATAWIDTH=32 时每个 DMA 字产生一次 ifwe，
     * faddr = ifaddr（6 bit，逐字递增），fdata = 数据字低 16 位。
     * 192 B 的 SETUP 帧 ⇒ 48 个 16 bit 表项写进 64x16 RAM。
     */
    for (i = 0; i < words; i++) {
        uint16_t w = data[i * 4] | (data[i * 4 + 1] << 8);

        s->filter[i % DMFE_FILTER_LEN] = w;
    }

    /*
     * tlsm.v:763-766：ft = {TDES1[28], TDES1[22]}，只在 SET 帧时采样；
     * csr.v:923-948 用它持续驱动 CSR6 的 ho/if/hp（这三位软件写不进去）：
     *   ft=00 PERFECT → ho=0,if=0,hp=0   ft=01 HASH   → hp=1
     *   ft=10 INVERSE → if=1             ft=11 HONLY  → ho=1,hp=1
     */
    s->ft = (uint8_t)((((des1 >> 28) & 1) << 1) | ((des1 >> 22) & 1));
    s->csr6 &= ~(CSR6_HO | CSR6_IF | CSR6_HP);
    switch (s->ft) {
    case 1:                          /* FT_HASH */
        s->csr6 |= CSR6_HP;
        break;
    case 2:                          /* FT_INVERSE */
        s->csr6 |= CSR6_IF;
        break;
    case 3:                          /* FT_HONLY */
        s->csr6 |= CSR6_HO | CSR6_HP;
        break;
    default:                         /* FT_PERFECT */
        break;
    }

    qemu_log_mask(LOG_TRACE,
                  "chiplab-dmfe: setup frame: %d bytes -> %d filter words "
                  "(ft=%u)\n", len, words, s->ft);
}

static void dmfe_tx_run(ChiplabDmfeState *s)
{
    uint32_t addr = s->tx_desc;
    int guard = 0;

    if (!(s->csr6 & CSR6_ST)) {
        return;
    }

    while (guard++ < 64) {
        uint32_t d[4];
        uint32_t len;

        d[0] = dmfe_ldl(s, addr + 0);
        DMFE_TRACE("chiplab-dmfe: tx_run desc=%08x d0=%08x\n", addr, d[0]);
        if (!(d[0] & DESC_OWN)) {
            break;                       /* 软件还没交给 MAC */
        }
        d[1] = dmfe_ldl(s, addr + 4);
        d[2] = dmfe_ldl(s, addr + 8);
        d[3] = dmfe_ldl(s, addr + 12);
        len = d[1] & TDES1_BS1_MASK;

        if (d[1] & TDES1_SET) {
            /* SETUP 帧：不发送，写地址过滤 RAM */
            dmfe_tx_setup_frame(s, d[2], len, d[1]);
        } else if (len > 0 && len <= DMFE_MAX_FRAME) {
            uint8_t frame[DMFE_MAX_FRAME];
            ssize_t sent = -1;

            dmfe_read_buf(s, d[2], frame, len);
            if (s->nic) {
                sent = qemu_send_packet(qemu_get_queue(s->nic), frame, len);
            }
            (void)sent;
            DMFE_TRACE("chiplab-dmfe: TX len=%u dst=%02x:%02x:%02x:%02x:%02x:"
                       "%02x sent=%zd\n", len, frame[0], frame[1], frame[2],
                       frame[3], frame[4], frame[5], sent);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "chiplab-dmfe: bad tx length %u (desc 0x%08x)\n",
                          len, addr);
        }

        /*
         * tlsm.v:1552-1557：完成时把 tstat 覆写回 TDES0（最高位为 0 ⇒ OWN 清零；
         * 成功时全部错误位为 0）。33 位/29 位拼接细节见对齐表，模型写出
         * "OWN=0 + 无错误" 的等价值。
         */
        dmfe_stl(s, addr + 0, 0);

        s->csr5 |= CSR5_TI;
        s->tx_desc = dmfe_next_desc(s, addr, d[1], d[3], true);
        addr = s->tx_desc;
    }

    dmfe_update_irq(s);
}

/* ------------------------------------------------------------------ */
/* 地址过滤：rc.v:1236-1246（PR 全通过）/ 1454-1470（过滤 RAM 比对）    */
/*   RAM[0] == DA[2]|DA[3]<<8、RAM[1] == DA[0]|DA[1]<<8、              */
/*   RAM[2] == DA[4]|DA[5]<<8                                          */
/* ------------------------------------------------------------------ */
static bool dmfe_rx_filter_ok(ChiplabDmfeState *s, const uint8_t *buf,
                              size_t len)
{
    bool mc, match = false;
    uint16_t da01, da23, da45;
    int i;

    if (len < 6) {
        return false;
    }
    mc = buf[0] & 1;                     /* dest[0] = I/G 位（rc.v:1372-1425） */

    /* 过滤模式选择（rc.v:1237-1256）：
     *   if (pr) FSM_MATCH; else if (ho | (hp & dest[0])) FSM_HASH;
     *   else if (!hp) FSM_PERF16; else FSM_PERF1;
     */
    if (s->csr6 & CSR6_PR) {
        match = true;
    } else if ((s->csr6 & CSR6_HO) || ((s->csr6 & CSR6_HP) && mc)) {
        /* 哈希过滤（rc.v:1305-1314）本模型未实现：按不匹配处理并记录 */
        qemu_log_mask(LOG_UNIMP,
                      "chiplab-dmfe: multicast hash filter not modelled "
                      "(DA %02x:%02x:%02x:%02x:%02x:%02x)\n",
                      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        match = false;
    } else {
        da01 = (uint16_t)(buf[0] | (buf[1] << 8));
        da23 = (uint16_t)(buf[2] | (buf[3] << 8));
        da45 = (uint16_t)(buf[4] | (buf[5] << 8));
        /*
         * FSM_PERF16：rc.v:1456-1465 每 3 个表项比一个目的地址
         * （RAM[3k]↔DA[2:3]、RAM[3k+1]↔DA[0:1]、RAM[3k+2]↔DA[4:5]），
         * 共 16 组（fa 扫到 50）。RTL 读口有 2 拍流水（报告 §C.3 已提示
         * 该映射需仿真实测），模型按"意图"实现。
         */
        for (i = 0; i + 2 < DMFE_FILTER_LEN && i < 48; i += 3) {
            if (s->filter[i] == da23 && s->filter[i + 1] == da01 &&
                s->filter[i + 2] == da45) {
                match = true;
                break;
            }
        }
        if (s->csr6 & CSR6_IF) {         /* 反过滤（FT_INVERSE） */
            match = !match;
        }
    }

    /*
     * 接受判定（rc.v:599/621/647 原文，Verilog 里 & 优先级高于 |）：
     *   if ((pb) & (fsm == FSM_MATCH | ra | (pm & (dest[0]))))
     * ⇒ (PB & match) | RA | (PM & 组播)
     * PB 是**与**项：PB=0 时连混杂模式收不到包（RTL 硬口径）。
     */
    if ((s->csr6 & CSR6_PB) && match) {
        return true;
    }
    if (s->csr6 & CSR6_RA) {
        return true;
    }
    if ((s->csr6 & CSR6_PM) && mc) {
        return true;
    }

    return false;
}

static bool dmfe_rx_desc_ready(ChiplabDmfeState *s)
{
    uint32_t d0;

    if (!(s->csr6 & CSR6_SR)) {
        return false;
    }
    d0 = dmfe_ldl(s, s->rx_desc + 0);

    return d0 & DESC_OWN;
}

/* 把一帧写进接收环；返回 true 表示已交付 */
static bool dmfe_rx_deliver(ChiplabDmfeState *s, const uint8_t *buf,
                            size_t len)
{
    uint32_t addr = s->rx_desc;
    uint32_t d[4];
    uint32_t bs1;
    uint32_t status;
    size_t copy;

    d[0] = dmfe_ldl(s, addr + 0);
    if (!(d[0] & DESC_OWN)) {
        return false;
    }
    d[1] = dmfe_ldl(s, addr + 4);
    d[2] = dmfe_ldl(s, addr + 8);
    d[3] = dmfe_ldl(s, addr + 12);
    bs1 = d[1] & RDES1_BS1_MASK;
    if (bs1 == 0) {
        bs1 = DMFE_MAX_FRAME;
    }

    copy = MIN(len, (size_t)bs1);
    if (s->csr10 & CSR10_INSERT_EN) {
        /*
         * rc.v:290/971：insert_en 时前 2 字节是"插入"的（低半字是上一拍的
         * 残留数据），帧体整体后移 2 字节；rlsm.v:543 再把上报长度减 2，
         * 所以 RDES0[29:16] 仍是真实帧长。模型用 0x0000 作插入字节并在
         * 对齐表里注明"数据内容不等于 RTL 的残留值"。
         */
        uint8_t ins[2] = { 0, 0 };

        dmfe_write_buf(s, d[2], ins, sizeof(ins));
        dmfe_write_buf(s, d[2] + 2, buf, copy);
    } else {
        size_t whole = (copy + 3) & ~(size_t)3;

        dmfe_write_buf(s, d[2], buf, copy);
        if (whole > copy) {
            uint8_t pad[3] = { 0, 0, 0 };

            /* 整字写语义：只把补齐的字节写成 0，不复制 RTL 的残留数据 */
            dmfe_write_buf(s, d[2] + copy, pad, whole - copy);
        }
    }

    /*
     * rlsm.v:543-547（原文）：
     *   real_length = insert_en ? length-2 : length
     *   fstat = {1'b0(OWN=0), ff, real_length[13:0], res_c, rde,
     *            RDES0_RV[13:12], rf, mf, rfs, rls, tl, cs, ftp,
     *            RDES0_RV[4], re, db, ce, ov}
     * 一帧单描述符 ⇒ rls=1、rfs=1（末次 FSTAT 写）。
     * bit30(f): rc.v:1125-1132 在 FSM_MATCH 时置 0 ⇒ 匹配路径的帧写 0。
     * 另注：DMA 是整字写（macdata2axi.v:236 WSTRB=1111），帧尾后的
     * 残留字节不动，模型按 4 字节向上取整补 0 写。
     */
    status = RDES0_RLS | RDES0_RFS;
    status |= (uint32_t)len << RDES0_LEN_SHIFT;
    if (copy < len) {
        status |= RDES0_OV | RDES0_RDE | RDES0_ES;
    }
    dmfe_stl(s, addr + 0, status);

    s->csr5 |= CSR5_RI;
    s->rx_desc = dmfe_next_desc(s, addr, d[1], d[3], false);
    s->csr8_mfc &= ~CSR8_MFC_MASK;   /* 只读计数器：成功时不动 */

    return true;
}

static void dmfe_rx_run(ChiplabDmfeState *s)
{
    if (!s->rx_len) {
        return;
    }
    if (!(s->csr6 & CSR6_SR)) {
        s->rx_len = 0;                   /* ren=0：帧直接丢弃（rc.v:536） */
        return;
    }
    if (!dmfe_rx_desc_ready(s)) {
        /* 无可用描述符：置 RU（csr.v:823-825 rui），计入丢失帧，丢弃该帧 */
        s->csr5 |= CSR5_RU;
        s->csr8_mfc = (s->csr8_mfc + 1) & CSR8_MFC_MASK;
        s->rx_len = 0;
        dmfe_update_irq(s);
        return;
    }

    if (!dmfe_rx_filter_ok(s, s->rx_frame, s->rx_len)) {
        s->rx_len = 0;
        return;
    }

    dmfe_rx_deliver(s, s->rx_frame, s->rx_len);
    s->rx_len = 0;
    dmfe_update_irq(s);
}

/* ------------------------------------------------------------------ */
/* NIC 后端回调（QEMU 9.2：can_receive(NetClientState*) /             */
/* receive(NetClientState*, const uint8_t*, size_t)）                  */
/* ------------------------------------------------------------------ */
static bool chiplab_dmfe_can_receive(NetClientState *nc)
{
    ChiplabDmfeState *s = qemu_get_nic_opaque(nc);

    return dmfe_rx_desc_ready(s);
}

static ssize_t chiplab_dmfe_receive(NetClientState *nc, const uint8_t *buf,
                                    size_t size)
{
    ChiplabDmfeState *s = qemu_get_nic_opaque(nc);

    DMFE_TRACE("chiplab-dmfe: RX %zu bytes\n", size);
    if (size > DMFE_MAX_FRAME) {
        return -1;
    }
    memcpy(s->rx_frame, buf, size);
    s->rx_len = size;
    dmfe_rx_run(s);

    /* 无论是否交付都算"已接收"：丢弃路径已置 RU/计入 mfc */
    return size;
}

static void chiplab_dmfe_cleanup(NetClientState *nc)
{
    ChiplabDmfeState *s = qemu_get_nic_opaque(nc);

    s->nic = NULL;
}

static void chiplab_dmfe_set_link(NetClientState *nc)
{
    ChiplabDmfeState *s = qemu_get_nic_opaque(nc);

    /* 通用 MII PHY 的 BMSR.link_status 跟随后端链路 */
    if (s) {
        if (nc->link_down) {
            s->phy_regs[PHY_BMSR] &= ~(1u << 2);
        } else {
            s->phy_regs[PHY_BMSR] |= (1u << 2);
        }
    }
}

static NetClientInfo net_chiplab_dmfe_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = chiplab_dmfe_can_receive,
    .receive = chiplab_dmfe_receive,
    .cleanup = chiplab_dmfe_cleanup,
    .link_status_changed = chiplab_dmfe_set_link,
};

/* ------------------------------------------------------------------ */
/* MII 管理：CSR9 软件位拍 + 通用 MII PHY（IEEE 802.3 clause 22）      */
/* ------------------------------------------------------------------ */
static uint16_t dmfe_phy_read(ChiplabDmfeState *s, int reg)
{
    if (reg < 0 || reg >= PHY_REG_NUM) {
        return 0;
    }
    return s->phy_regs[reg];
}

static void dmfe_phy_write(ChiplabDmfeState *s, int reg, uint16_t val)
{
    if (reg < 0 || reg >= PHY_REG_NUM) {
        return;
    }

    switch (reg) {
    case PHY_BMCR:
        if (val & (1u << 15)) {                 /* 软复位：自清 */
            val &= ~(1u << 15);
            s->phy_regs[PHY_BMSR] = PHY_BMSR_VAL;
            s->phy_regs[PHY_ANAR] = 0x01e1;
            s->phy_regs[PHY_ANLPAR] = 0xc5e1;
            s->phy_regs[PHY_ANER] = 0x000f;
        }
        if (val & (1u << 9)) {                  /* 重启自协商 */
            val &= ~(1u << 9);
            s->phy_regs[PHY_BMSR] |= (1u << 5);  /* autoneg complete */
            val |= (1u << 12);
        }
        s->phy_regs[PHY_BMCR] = val;
        break;
    default:
        s->phy_regs[reg] = val;
        break;
    }
}

/*
 * 一个 MDC 上升沿：采样 MDO、按需驱动 MDI。
 * 帧格式：32 x 1 + ST(01) + OP(10 读 / 01 写) + PHYAD(5) + REGAD(5) + TA(2)
 *         + DATA(16)
 * 驱动侧实现见 u-boot/drivers/net/chiplab_dmfe.c（chiplab_dmfe_mdio_xfer）。
 */
static int dmfe_mdio_clock(ChiplabDmfeState *s, int mdo)
{
    int mdi = 1;                     /* 空闲时由上拉保持 1 */

    if (!s->mdio_in_frame) {
        if (mdo) {
            if (s->mdio_preamble < 33) {
                s->mdio_preamble++;
            }
            return 1;
        }
        if (s->mdio_preamble >= 32) {
            /* 前导结束后的第一个 0 = ST 比特 1，进入帧 */
            s->mdio_in_frame = true;
            s->mdio_nbits = 0;
            s->mdio_frame = 0;
            s->mdio_preamble = 0;
        } else {
            s->mdio_preamble = 0;
            return 1;
        }
    }

    /* 帧内：0-based 位序号 */
    if (s->mdio_nbits < 14) {
        s->mdio_frame = (s->mdio_frame << 1) | (mdo & 1);
        if (s->mdio_nbits == 13) {
            /* ST(2)+OP(2)+PHYAD(5)+REGAD(5) 收齐 */
            uint32_t op = (s->mdio_frame >> 10) & 3;

            s->mdio_phyad = (s->mdio_frame >> 5) & 0x1f;
            s->mdio_regad = s->mdio_frame & 0x1f;
            s->mdio_read = (op == 2);
            if (op != 1 && op != 2) {
                s->mdio_in_frame = false;   /* 非法 OP：丢弃 */
                s->mdio_preamble = 0;
                return 1;
            }
            s->mdio_data = 0;
        }
    } else if (s->mdio_nbits == 14) {
        /* TA 比特 1：MAC 驱动（写=1，读=0），PHY 不驱动 */
        mdi = 1;
    } else if (s->mdio_nbits == 15) {
        /* TA 比特 2：读操作由 PHY 驱动 0，随后进入数据相位（MSB first） */
        mdi = 0;
        if (s->mdio_read) {
            s->mdio_data = dmfe_phy_read(s, s->mdio_regad);
        }
    } else {
        if (s->mdio_read) {
            mdi = (s->mdio_data >> 15) & 1;
            s->mdio_data = (s->mdio_data << 1) & 0xffff;
        } else {
            s->mdio_frame = (s->mdio_frame << 1) | (mdo & 1);
        }
    }

    s->mdio_nbits++;
    if (s->mdio_nbits >= 32) {
        if (!s->mdio_read && s->mdio_phyad == DMFE_PHY_ADDR) {
            dmfe_phy_write(s, s->mdio_regad, s->mdio_frame & 0xffff);
        }
        s->mdio_in_frame = false;
        s->mdio_preamble = 0;
    }

    return mdi;
}

/* ------------------------------------------------------------------ */
/* CSR 读                                                             */
/* ------------------------------------------------------------------ */
static uint32_t dmfe_csr_read(ChiplabDmfeState *s, uint32_t off)
{
    switch (off) {
    case CSR0:
        /* csr.v:1150-1152：{RV[31:26],RV[25:21],dbo,tap,RV[16:14],pbl,ble,
         *                   dsl,bar,(rst|csr0_swr)}
         * 模型把 rst 折进 csr0_swr（写 1 读 1，写 0 读 0；RTL 的软复位
         * 握手结束后 rst 也会落 0）。 */
        return CSR0_RO_MASK | (s->csr0 & ~(CSR0_RO_MASK | CSR0_SWR)) |
               ((s->csr0_swr &&
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->swr_until_ns)
                ? CSR0_SWR : 0);
    case CSR1:
    case CSR2:
        return 0;                    /* 轮询请求寄存器：读恒 0 */
    case CSR3:
        return s->csr3;
    case CSR4:
        return s->csr4;
    case CSR5: {
        uint32_t v = CSR5_RO_MASK | (s->csr5 & ~CSR5_RO_MASK);

        /* csr.v:869-870：TS/RS 状态字段 */
        v |= (uint32_t)((s->csr6 & CSR6_ST) ? 1 : 0) << CSR5_TS_SHIFT;
        v |= (uint32_t)((s->csr6 & CSR6_SR) ? 1 : 0) << CSR5_RS_SHIFT;
        /* csr.v:840-868：NIS/AIS 不是独立锁存位，而是"状态 & 使能"的汇总 */
        if (dmfe_nis(s)) {
            v |= CSR5_NIS;
        }
        if (dmfe_ais(s)) {
            v |= CSR5_AIS;
        }
        return v;
    }
    case CSR6:
        /* csr.v:1156-1163：标准 Tulip 位序（31 位拼接左补零导致 ra 落在
         * bit29、bit31 恒 0），读回恒定 1 的位见 CSR6_RO_MASK */
        return CSR6_RO_MASK | (s->csr6 & CSR6_WR_MASK);
    case CSR7:
        return CSR7_RO_MASK | (s->csr7 & CSR7_WR_MASK);
    case CSR8:
        return CSR8_RO_MASK | s->csr8_foc | s->csr8_mfc;
    case CSR9:
        /* csr.v:1174-1176：{RV[31:20], mdi, mii(mden), mdo, mdc,
         *                   RV[15:4], sdo, sdi, sclk, scs}
         * bit19 = mdi 是**输入**（csr.v:1071 csr9_mdi <= mdi，每个时钟重采样），
         * 必须参与读回，否则软件位拍读不到 PHY 数据。 */
        return CSR9_RO_MASK | (s->csr9 & (CSR9_SCS | CSR9_SCLK | CSR9_SDI |
                                          CSR9_SDO | CSR9_MDC | CSR9_MDO |
                                          CSR9_MDEN | CSR9_MDI));
    case CSR10:
        return s->csr10 & CSR10_INSERT_EN;
    case CSR11:
        return CSR11_RO_MASK | (s->csr11 & ~CSR11_RO_MASK);
    default:
        /* 未实现寄存器读 0（csr.v 读多路 default 分支） */
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* CSR 写                                                             */
/* ------------------------------------------------------------------ */
static void dmfe_soft_reset(ChiplabDmfeState *s)
{
    /* csr.v:560-597/rstc.v：软复位清空数据通路状态 */
    s->csr0 = 0;
    s->csr3 = 0xffffffffu;           /* CSR3_RV = 0xFFFFFFFF（utility.v:94） */
    s->csr4 = 0xffffffffu;           /* CSR4_RV = 0xFFFFFFFF（utility.v:99） */
    s->csr5 = 0;
    s->csr6 = CSR6_PR;               /* CSR6_RV[6]=1 ⇒ PR 复位为 1 */
    s->csr7 = 0;
    s->csr8_mfc = 0;
    s->csr8_foc = 0;
    s->csr9 = CSR9_SCS | CSR9_SCLK | CSR9_SDI | CSR9_SDO; /* RV[3:0]=0xF */
    s->csr10 = 0;
    s->csr11 = 0;
    /* rdbadc/tdbadc 复位为 1 ⇒ CSR3/CSR4 未编程前不取描述符（csr.v:677/726） */
    s->tx_desc = 0xffffffffu;
    s->rx_desc = 0xffffffffu;
    s->rx_len = 0;
    s->ft = 0;                       /* 复位 = FT_PERFECT：ho/if/hp 全 0 */
    s->swr_until_ns = 0;
    memset(s->filter, 0, sizeof(s->filter));
    s->mdio_in_frame = false;
    s->mdio_preamble = 0;
    s->mdio_nbits = 0;
    dmfe_update_irq(s);
}

static void dmfe_csr_write(ChiplabDmfeState *s, uint32_t off, uint32_t val)
{
    switch (off) {
    case CSR0:
        /* csr.v:571-597：SWR 是"请求位"，模型保留写值供读回；
         * 其余可写位 dbo/tap/pbl/ble/dsl/bar 原样锁存。 */
        s->csr0 = val & (CSR0_DBO | CSR0_TAP_MASK | CSR0_PBL_MASK |
                         CSR0_BLE | CSR0_DSL_MASK | CSR0_BAR);
        if (val & CSR0_SWR) {
            /*
             * csr.v:1152 读回 bit0 = (rst | csr0_swr)；写 1 会拉起 rstsoft，
             * rstc.v:104-118 的握手做完后 CSR 块自己被复位（csr0_swr 清零）。
             * 模型没有时钟，用 1 µs 窗口表示这段握手：
             *   · 写 1 后立刻读 ⇒ 读到 1（与"SWR 读恒 1"口径一致）；
             *   · "写 1 然后轮询到 0" 的标准 Tulip 复位流程 ⇒ 也能结束；
             *   · 软件显式写 0 ⇒ 立即清。
             */
            s->csr0_swr = 1;
            s->swr_until_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000;
            dmfe_soft_reset(s);
        } else {
            s->csr0_swr = 0;
            s->swr_until_ns = 0;
        }
        break;
    case CSR1:
        dmfe_tx_run(s);              /* 发送轮询请求（值无关） */
        break;
    case CSR2:
        dmfe_rx_run(s);              /* 接收轮询请求 */
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case CSR3:
        s->csr3 = val;
        s->rx_desc = val;
        dmfe_rx_run(s);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case CSR4:
        s->csr4 = val;
        s->tx_desc = val;
        break;
    case CSR5:
        /* csr.v:781-798：写 1 清（TS/RS 字段不受写影响） */
        s->csr5 &= ~(val & CSR5_W1C);
        dmfe_update_irq(s);
        break;
    case CSR6:
        /* csr.v:893-950：只锁存实现的位；ho/if/hp 由过滤类型推导（模型
         * 按写值原样保存，见对齐表"过滤模式"） */
        /* ho/if/hp 不在 WR_MASK 里：它们由 SETUP 帧的 ft 驱动（csr.v:923-948） */
        s->csr6 = (s->csr6 & ~CSR6_WR_MASK) | (val & CSR6_WR_MASK);
        if (val & CSR6_ST) {
            dmfe_tx_run(s);
        }
        if (val & CSR6_SR) {
            dmfe_rx_run(s);
            if (s->nic) {
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
            }
        }
        dmfe_update_irq(s);
        break;
    case CSR7:
        s->csr7 = val & CSR7_WR_MASK;
        dmfe_update_irq(s);
        break;
    case CSR8:
        /* 只读计数器 */
        break;
    case CSR9: {
        uint32_t old = s->csr9;

        s->csr9 = val & (CSR9_SCS | CSR9_SCLK | CSR9_SDI | CSR9_SDO |
                         CSR9_MDC | CSR9_MDO | CSR9_MDEN);
        /* MDC 上升沿 ⇒ 交换一位（csr.v:2443-2447：mdo/mden/mdc 直通 PHY） */
        if ((val & CSR9_MDC) && !(old & CSR9_MDC)) {
            int mdi = dmfe_mdio_clock(s, !!(val & CSR9_MDO));

            if (mdi) {
                s->csr9 |= CSR9_MDI;
            } else {
                s->csr9 &= ~CSR9_MDI;
            }
        }
        break;
    }
    case CSR10:
        s->csr10 = val & CSR10_INSERT_EN;
        break;
    case CSR11:
        /* 定时器为占位实现：只保存可写位，不计数 */
        s->csr11 = val;
        break;
    default:
        /* 未实现寄存器写丢弃（csr.v 写路径只对 CSR0..CSR11 有分支） */
        break;
    }
}

static uint64_t chiplab_dmfe_mmio_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    ChiplabDmfeState *s = opaque;
    uint32_t off = (addr & 0xff) & ~3u;   /* maccsr2axi.v:88 csraddr[7:0] */
    uint32_t v = dmfe_csr_read(s, off);
    int lane = addr & 3;

    switch (size) {
    case 1:
        return (v >> (lane * 8)) & 0xff;
    case 2:
        return (v >> ((lane & 2) * 8)) & 0xffff;
    default:
        return v;
    }
}

static void chiplab_dmfe_mmio_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned size)
{
    ChiplabDmfeState *s = opaque;
    uint32_t off = (addr & 0xff) & ~3u;
    uint32_t v = dmfe_csr_read(s, off);
    int lane = addr & 3;

    switch (size) {
    case 1:
        v = (v & ~(0xffu << (lane * 8))) | ((uint32_t)value << (lane * 8));
        break;
    case 2:
        v = (v & ~(0xffffu << ((lane & 2) * 8))) |
            (((uint32_t)value & 0xffff) << ((lane & 2) * 8));
        break;
    default:
        v = value;
        break;
    }

    dmfe_csr_write(s, off, v);
}

static const MemoryRegionOps chiplab_dmfe_ops = {
    .read = chiplab_dmfe_mmio_read,
    .write = chiplab_dmfe_mmio_write,
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

/* ------------------------------------------------------------------ */
/* 设备生命周期                                                        */
/* ------------------------------------------------------------------ */
/* 上电/复位后的寄存器与 PHY 初值（realize 与 reset 共用） */
static void chiplab_dmfe_init_state(ChiplabDmfeState *s)
{
    int i;

    dmfe_soft_reset(s);
    /* CSR0/CSR6/CSR7/CSR8/CSR9/CSR11 的复位读值 */
    s->csr0 = 0;
    s->csr0_swr = 0;
    s->csr6 = CSR6_PR;               /* CSR6_RV[6]=1（csr.v:886） */
    s->csr7 = 0;                     /* CSR7 复位：全部使能 = 0 */
    s->csr9 = CSR9_SCS | CSR9_SCLK | CSR9_SDI | CSR9_SDO;
    s->csr11 = 0;
    s->tx_desc = 0xffffffffu;
    s->rx_desc = 0xffffffffu;

    for (i = 0; i < PHY_REG_NUM; i++) {
        s->phy_regs[i] = 0;
    }
    s->phy_regs[PHY_BMCR] = 0;
    s->phy_regs[PHY_BMSR] = PHY_BMSR_VAL;
    s->phy_regs[PHY_ID1] = PHY_ID1_VAL;
    s->phy_regs[PHY_ID2] = PHY_ID2_VAL;
    s->phy_regs[PHY_ANAR] = 0x01e1;
    s->phy_regs[PHY_ANLPAR] = 0xc5e1;
    s->phy_regs[PHY_ANER] = 0x000f;
}

static void chiplab_dmfe_reset(DeviceState *dev)
{
    chiplab_dmfe_init_state(CHIPLAB_DMFE(dev));
}

static void chiplab_dmfe_realize(DeviceState *dev, Error **errp)
{
    ChiplabDmfeState *s = CHIPLAB_DMFE(dev);

    /* 上电初值：不依赖 reset 钩子的调用时机 */
    chiplab_dmfe_init_state(s);

    if (!s->conf.peers.ncs[0]) {
        warn_report("chiplab-dmfe: no netdev attached; the MAC will not "
                    "reach the network (use -nic user,tftp=<dir>)");
    }

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_chiplab_dmfe_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void chiplab_dmfe_instance_init(Object *obj)
{
    ChiplabDmfeState *s = CHIPLAB_DMFE(obj);

    memory_region_init_io(&s->mmio, obj, &chiplab_dmfe_ops, s,
                          "chiplab.dmfe", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static Property chiplab_dmfe_properties[] = {
    DEFINE_NIC_PROPERTIES(ChiplabDmfeState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void chiplab_dmfe_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = chiplab_dmfe_realize;
    device_class_set_props(dc, chiplab_dmfe_properties);
    device_class_set_legacy_reset(dc, chiplab_dmfe_reset);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo chiplab_dmfe_type_info = {
    .name = TYPE_CHIPLAB_DMFE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ChiplabDmfeState),
    .instance_init = chiplab_dmfe_instance_init,
    .class_init = chiplab_dmfe_class_init,
};

static void chiplab_dmfe_register_types(void)
{
    type_register_static(&chiplab_dmfe_type_info);
}

type_init(chiplab_dmfe_register_types)
