/*
 * Loongson Chiplab FPGA SoC - DEC 21140 "Tulip" compatible MAC (MMIO)
 *
 * 设备模型实现在 hw/net/chiplab_dmfe.c；本头文件只暴露类型名与"板级接入"
 * 说明，供 hw/riscv/chiplab.c 使用。
 *
 * 接线口径（RTL 出处）：
 *   · MMIO 窗口 0x1ff0_0000，64 KiB（axi_mux_syn.v:859/950）；
 *   · CSR n 位于字节偏移 n*8（maccsr2axi.v:88 + csr.v:458）；
 *   · 中断 = mac_int = int_out[0]（soc_top.v:597/725）。
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_NET_CHIPLAB_DMFE_H
#define HW_NET_CHIPLAB_DMFE_H

#define TYPE_CHIPLAB_DMFE "chiplab-dmfe"

#endif /* HW_NET_CHIPLAB_DMFE_H */
