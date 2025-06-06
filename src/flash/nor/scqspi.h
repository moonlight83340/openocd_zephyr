/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 by spacecubics                                     *
 *   gaetan.perrot@spacecubics.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_SCOBC_QSPI_H
#define OPENOCD_FLASH_NOR_SCOBC_QSPI_H

#include "spi.h"

#define QSPI_DATA_MEM0_SS (0x01)
#define QSPI_CFG_MEM0		   (0u)
#define QSPI_CFG_MEM1		   (1u)
#define QSPI_ASR_IDLE (0x00)
#define QSPI_ASR_BUSY (0x01)
#define QSPI_RX_FIFO_MAX_BYTE (16u)
#define QSPI_SPI_MODE_QUAD (0x00020000)

#define SCOBCA1_FPGA_SYSREG_CFGMEMCTL (0x4F000010) /* Configuration Memory Register */

/* Offset */
#define QSPI_ACR_OFFSET	   (0x0000) /* QSPI Access Control Register */
#define QSPI_TDR_OFFSET	   (0x0004) /* QSPI TX Data Register */
#define QSPI_RDR_OFFSET	   (0x0008) /* QSPI RX Data Register */
#define QSPI_ASR_OFFSET	   (0x000C) /* QSPI Access Status Register */
#define QSPI_FIFOSR_OFFSET (0x0010) /* QSPI FIFO Status Register */
#define QSPI_FIFORR_OFFSET (0x0014) /* QSPI FIFO Reset Register */
#define QSPI_ISR_OFFSET	   (0x0020) /* QSPI Interrupt Status Register */
#define QSPI_IER_OFFSET	   (0x0024) /* QSPI Interrupt Enable Register */
#define QSPI_CCR_OFFSET	   (0x0030) /* QSPI Clock Control Register */
#define QSPI_DCMSR_OFFSET  (0x0034) /* QSPI Data Capture Mode Setting Register */
#define QSPI_FTLSR_OFFSET  (0x0038) /* QSPI FIFO Threshold Level Setting Register */
#define QSPI_VER_OFFSET	   (0xF000) /* QSPI Controller IP Version Register */

/* QSPI Control Register for Data/ Configutation Memory */
#define SCOBCA1_FPGA_NORFLASH_QSPI_ACR(base)	(base + QSPI_ACR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base)	(base + QSPI_TDR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base)	(base + QSPI_RDR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_ASR(base)	(base + QSPI_ASR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FIFOSR(base) (base + QSPI_FIFOSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FIFORR(base) (base + QSPI_FIFORR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base)	(base + QSPI_ISR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_IER(base)	(base + QSPI_IER_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_CCR(base)	(base + QSPI_CCR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_DCMSR(base)	(base + QSPI_DCMSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FTLSR(base)	(base + QSPI_FTLSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_VER(base)	(base + QSPI_VER_OFFSET)

#endif /* OPENOCD_FLASH_NOR_SCOBC_QSPI_H */