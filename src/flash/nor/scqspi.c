/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 Space Cubics                                       *
 *   gaetan.perrot@spacecubics.com                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "scqspi.h"
#include <helper/time_support.h>
#include <target/image.h>

/* Timeout in ms */
#define SPI_CMD_TIMEOUT (100)

/**
 * struct scqspi_flash_bank - Represents a NOR flash bank for SCQSPI interface.
 *
 * @target: Pointer to the target structure associated with the flash bank.
 * @probed: Boolean flag indicating whether the flash bank has been probed.
 * @io_base: Base address for I/O operations on the flash bank.
 * @spi_ss: SPI slave select identifier for the flash bank.
 * @dev: Flash device structure containing device-specific information.
 *
 * This structure is used to manage and interact with a NOR flash bank
 * connected via the SCQSPI interface. It holds essential information
 * about the target, probing status, I/O base address, SPI slave select,
 * and the flash device details.
 */
struct scqspi_flash_bank {
  struct target *target;
  bool probed;
  uint32_t io_base;
  uint8_t spi_ss;
  struct flash_device dev;
};

/* ------------------------------------------------------------------------- */
/* Internal helper functions for scqspi flash driver implementation          */
/* ------------------------------------------------------------------------- */

static bool is_qspi_control_done(struct target *target, uint32_t base) {
  uint32_t val;
  int retval;
  LOG_DEBUG("Confirm QSPI Interrupt Status is `SPI Control Done`\n");

  retval = target_read_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base), &val);

  if (val != 0x01) {
    LOG_ERROR("Confirm QSPI Interrupt Status failed %d != 0x01\n", val);
  }

  LOG_DEBUG("Clear QSPI Interrupt Status\n");

  retval = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base), 0x01);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to read QSPI Interrupt Status\n");
    return false;
  }

  retval = target_read_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base), &val);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to read QSPI Interrupt Status\n");
    return false;
  }

  if (val != 0x00) {
    LOG_ERROR("Confirm QSPI Interrupt Status failed %d != 0x00\n", val);
    return false;
  }

  return true;
}

static bool is_qspi_idle(struct target *target, uint32_t base) {
  uint32_t val;
  int retval;

  LOG_DEBUG("Confirm QSPI Access Status is `Idle`\n");

  retval = target_read_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ASR(base), &val);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to read QSPI ASR register\n");
    return false;
  }

  return (val == QSPI_ASR_IDLE);
}

static int wait_qspi_idle(struct target *target, uint32_t base,
                          int timeout_ms) {
  long long endtime = timeval_ms() + timeout_ms;

  do {
    if (is_qspi_idle(target, base)) {
      return ERROR_OK;
    }

    LOG_DEBUG("QSPI busy, waiting...");
    alive_sleep(1);

  } while (timeval_ms() < endtime);

  LOG_ERROR("Timeout waiting for QSPI to become idle");
  return ERROR_FLASH_OPERATION_FAILED;
}

static int activate_spi_ss(struct target *target, uint32_t base,
                           uint32_t spi_mode) {
  int retval;

  LOG_DEBUG("Activate SPI SS with %08x\n", spi_mode);
  retval =
      target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ACR(base), spi_mode);

  if (retval != ERROR_OK)
    return retval;

  retval = wait_qspi_idle(target, base, SPI_CMD_TIMEOUT);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to wait for QSPI to become idle");
    return retval;
  }

  return ERROR_OK;
}

static int inactivate_spi_ss(struct target *target, uint32_t base) {
  int retval;

  LOG_DEBUG("Inactivate SPI SS\n");

  retval = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ACR(base),
                            0x00000000);

  if (retval != ERROR_OK)
    return retval;

  retval = wait_qspi_idle(target, base, SPI_CMD_TIMEOUT);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to wait for QSPI to become idle");
    return retval;
  }

  return ERROR_OK;
}

/**
 * @brief Resets the QSPI FIFO registers for the specified target and base
 * address.
 *
 * This function writes specific values to the QSPI FIFO Reset Register (FIFORR)
 * and the QSPI FIFO Status Register (FIFOSR) to reset the FIFO state.
 *
 * @param target Pointer to the target structure representing the device.
 * @param base Base address of the QSPI peripheral.
 * @return ERROR_OK on
 * success, or an error code indicating the failure.
 */
static int reset_qspi_fifo(struct target *target, uint32_t base) {
  int retval;
  /* Reset QSPI FIFO */
  retval =
      target_write_u8(target, SCOBCA1_FPGA_NORFLASH_QSPI_FIFORR(base), 0x01);
  if (retval != ERROR_OK)
    return retval;

  retval =
      target_write_u8(target, SCOBCA1_FPGA_NORFLASH_QSPI_FIFOSR(base), 0x01);
  if (retval != ERROR_OK)
    return retval;

  return retval;
}

/**
 * @brief Clears the status register of the NOR flash device connected via QSPI.
 *
 * This function performs the following operations:
 * 1. Activates the SPI slave select (SS) line using SINGLE-IO mode.
 * 2. Clears all interrupt status register (ISR) flags.
 * 3. Resets the QSPI FIFO.
 * 4. Sends the "Clear Status Register" instruction (0x30) to the NOR flash
 * device.
 * 5. Deactivates the SPI slave select (SS) line.
 * 6. Confirms that the SPI control operation is completed successfully.
 *
 * @param target Pointer to the target device structure.
 * @param base Base address of the QSPI controller.
 * @param spi_ss SPI slave select identifier.
 *
 * @return ERROR_OK on success, or an error code indicating the failure reason.
 *         Possible error conditions include failure to activate/deactivate SPI
 * SS, failure to reset QSPI FIFO, or failure to confirm SPI control completion.
 */
static int clear_status_register(struct target *target, uint32_t base,
                                 uint32_t spi_ss) {
  int retval;

  /* Activate SPI SS with SINGLE-IO */
  retval = activate_spi_ss(target, base, spi_ss);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to activate SPI SS");
    return retval;
  }

  /* Clear All ISR */
  retval = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base),
                            0xFFFFFFFF);

  if (retval != ERROR_OK)
    return retval;

  /* Reset QSPI FIFO */
  retval = reset_qspi_fifo(target, base);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to reset QSPI FIFO");
    return false;
  }

  LOG_DEBUG("Clear Status Register (Instructure:0x30) \n");
  retval = target_write_u8(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x30);

  if (retval != ERROR_OK)
    return retval;

  /* Inactive SPI SS */
  retval = inactivate_spi_ss(target, base);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to inactivate SPI SS");
    return retval;
  }

  /* Confirm SPI Control is Done */
  if (!is_qspi_control_done(target, base)) {
    LOG_ERROR("Confirm SPI Control is Done failed");
    return ERROR_FAIL;
  }

  return ERROR_OK;
}

/**
 * @brief Selects the configuration memory for the target device.
 *
 * This function switches the configuration memory by writing to the
 * CFGMEMSEL register and verifies the operation by reading back the value.
 *
 * @param target Pointer to the target structure representing the device.
 * @param base Base address (unused in this function, but may be relevant for
 * future extensions).
 *
 * @return Returns ERROR_OK on success, or an error code if the operation fails.
 *
 * Currently we select the config memory 0  by default
 */
static int select_mem(struct target *target, uint32_t base) {
  uint8_t val;
  int retval;

  /* Config Memory is switched by CFGMEMSEL */
  LOG_DEBUG("Select Config Memory 0\n");
  retval = target_write_u8(target, SCOBCA1_FPGA_SYSREG_CFGMEMCTL, 0x00);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to write config memory\n");
    return retval;
  }

  retval = target_read_u8(target, SCOBCA1_FPGA_SYSREG_CFGMEMCTL, &val);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to read config memory\n");
    return retval;
  }

  if (val != 0x00) {
    LOG_ERROR("Can not select Config Memory %d != 0x00\n", val);
    return ERROR_FAIL;
  }

  return retval;
}

/**
 * @brief Initializes the SCQSPI flash bank.
 *
 * This function sets up the SCQSPI flash bank by selecting the memory region
 * and clearing the status register. It ensures the flash bank is ready for
 * further operations.
 *
 * @param bank Pointer to the flash bank structure.
 *
 * @return ERROR_OK on success, or an error code indicating the failure reason.
 */
static int scqspi_init(struct flash_bank *bank) {
  struct target *target = bank->target;
  struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
  int retval;

  LOG_DEBUG("%s", __func__);

  retval = select_mem(target, scqspi_info->io_base);
  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to select memory\n");
    return retval;
  }

  LOG_DEBUG("Clear Status Register\n");
  retval =
      clear_status_register(target, scqspi_info->io_base, scqspi_info->spi_ss);
  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to clear Status register\n");
    return retval;
  }

  return ERROR_OK;
}

/**
 * @brief Reads the ID of a NOR flash device connected via QSPI.
 *
 * This function sends the SPIFLASH_READ_ID command to the NOR flash device
 * and retrieves its ID. The ID is expected to be a 3-byte value, which is
 * returned in the `id` parameter.
 *
 * @param bank Pointer to the flash bank structure representing the NOR flash.
 * @param id Pointer to a uint32_t variable where the flash ID will be stored.
 *           The ID is constructed from the three bytes read from the device.
 *           If no flash is found, the ID will be set to 0xffffff.
 *
 * @return ERROR_OK on success, or an error code indicating the failure reason.
 *         Possible error scenarios include failure to activate SPI SS, reset
 *         the QSPI FIFO, send the SPIFLASH_READ_ID command, or read data from
 *         the device.
 *
 * @note This function assumes the flash bank structure and target are properly
 *       initialized. It also assumes the flash ID is a 3-byte value.
 *
 * @warning If the ID read from the device is 0xffffff, it indicates that no
 *          SPI flash was found.
 */
static int scqspi_read_id(struct flash_bank *bank, uint32_t *id) {
  struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
  struct target *target = bank->target;
  uint8_t din[3] = {0, 0, 0};
  uint8_t rdata;
  int retval;

  /* Activate SPI SS with SINGLE-IO */
  retval = activate_spi_ss(target, scqspi_info->io_base, scqspi_info->spi_ss);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to activate SPI SS");
    return retval;
  }

  /* Reset QSPI FIFO */
  retval = reset_qspi_fifo(target, scqspi_info->io_base);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to reset QSPI FIFO");
    return false;
  }

  LOG_DEBUG("Send SPIFLASH_READ_ID cmd\n");

  retval = target_write_u8(target,
                           SCOBCA1_FPGA_NORFLASH_QSPI_TDR(scqspi_info->io_base),
                           SPIFLASH_READ_ID);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to write SPIFLASH_READ_ID command");
    return retval;
  }

  retval = wait_qspi_idle(target, scqspi_info->io_base, SPI_CMD_TIMEOUT);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to wait for QSPI to become idle");
    return retval;
  }

  /* Reset QSPI FIFO */
  retval = reset_qspi_fifo(target, scqspi_info->io_base);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to reset QSPI FIFO");
    return false;
  }

  LOG_DEBUG("Read SPIFLASH_READ_ID \n");

  for (unsigned i = 0; i < sizeof(din); i++) {
    retval = target_write_u8(
        target, SCOBCA1_FPGA_NORFLASH_QSPI_RDR(scqspi_info->io_base), 0x00);

    if (retval != ERROR_OK)
      return retval;
  }

  retval = wait_qspi_idle(target, scqspi_info->io_base, SPI_CMD_TIMEOUT);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to wait for QSPI to become idle");
    return retval;
  }

  for (unsigned i = 0; i < sizeof(din); i++) {
    retval = target_read_u8(
        target, SCOBCA1_FPGA_NORFLASH_QSPI_RDR(scqspi_info->io_base), &rdata);

    if (retval != ERROR_OK)
      return retval;

    din[i] = rdata;
    LOG_DEBUG("Read byte %d: 0x%02x\n", i, din[i]);
  }

  /* Inactive SPI SS */
  retval = inactivate_spi_ss(target, scqspi_info->io_base);

  if (retval != ERROR_OK) {
    LOG_ERROR("Failed to inactivate SPI SS");
    return retval;
  }

  /* Not quite sure about that */
  // should read 0x00196001 (SC nor flash ID)
  *id = (din[2] << 16) | (din[1] << 8) | din[0];

  if (*id == 0xffffff) {
    LOG_ERROR("No SPI flash found");
    return ERROR_FAIL;
  }

  return ERROR_OK;
}

/* ------------------------------------------------------------------------- */
/* Command handler functions                                                 */
/* ------------------------------------------------------------------------- */

/* flash bank scqspi <base> <size> <chip_width> <bus_width> <target#>
 * <io_base> <driverPath>
 */
FLASH_BANK_COMMAND_HANDLER(scqspi_flash_bank_command) {
  struct scqspi_flash_bank *scqspi_info;

  LOG_DEBUG("%s", __func__);

  if (CMD_ARGC < 7)
    return ERROR_COMMAND_SYNTAX_ERROR;

  scqspi_info = malloc(sizeof(struct scqspi_flash_bank));
  if (!scqspi_info) {
    LOG_ERROR("not enough memory");
    return ERROR_FAIL;
  }

  bank->driver_priv = scqspi_info;
  scqspi_info->probed = false;
  COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], scqspi_info->io_base);
  /* Default to SPI SS 1 (Config memory 0) */
  scqspi_info->spi_ss = 0x01;

  return ERROR_OK;
}

static int scqspi_erase(struct flash_bank *bank, unsigned int first,
                        unsigned int last) {
  LOG_INFO("%s", __func__);
  return ERROR_OK;
}

static int scqspi_write(struct flash_bank *bank, const uint8_t *buffer,
                        uint32_t offset, uint32_t count) {
  LOG_INFO("%s", __func__);
  return ERROR_OK;
}

static int scqspi_read(struct flash_bank *bank, uint8_t *buffer,
                       uint32_t offset, uint32_t count) {
  LOG_INFO("%s", __func__);
  return ERROR_OK;
}

static int scqspi_probe(struct flash_bank *bank) {
  struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
  struct flash_sector *sectors;
  uint32_t id = 0; /* silence uninitialized warning */

  int ret;

  LOG_INFO("%s\n", __func__);

  if (scqspi_info->probed)
    free(bank->sectors);

  scqspi_info->probed = false;

  ret = scqspi_init(bank);
  if (ret != ERROR_OK)
    return ret;

  /* Check device id */
  ret = scqspi_read_id(bank, &id);
  if (ret != ERROR_OK) {
    LOG_ERROR("Failed to read flash ID");
    return ret;
  }

  memset(&scqspi_info->dev, 0, sizeof(scqspi_info->dev));
  bool found = false;
  for (const struct flash_device *p = flash_devices; p->name; p++) {
    if (p->device_id == id) {
      memcpy(&scqspi_info->dev, p, sizeof(scqspi_info->dev));
      found = true;
      break;
    }
  }

  if (!found) {
    LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
    return ERROR_FAIL;
  }

  LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
           scqspi_info->dev.name, scqspi_info->dev.device_id);

  /* Set bank size with informations from the device */
  bank->size = scqspi_info->dev.size_in_bytes;

  LOG_INFO("bank size %d\n", scqspi_info->dev.size_in_bytes);

  /* if no sectors, then treat whole flash as single sector */
  if (scqspi_info->dev.sectorsize == 0)
    scqspi_info->dev.sectorsize = scqspi_info->dev.size_in_bytes;
  /* if no page_size, then use sectorsize as page_size */
  if (scqspi_info->dev.pagesize == 0)
    scqspi_info->dev.pagesize = scqspi_info->dev.sectorsize;

  /* create and fill sectors array */
  bank->num_sectors =
      scqspi_info->dev.size_in_bytes / scqspi_info->dev.sectorsize;
  sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
  if (!sectors) {
    LOG_ERROR("not enough memory");
    return ERROR_FAIL;
  }

  for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
    sectors[sector].offset = sector * scqspi_info->dev.sectorsize;
    sectors[sector].size = scqspi_info->dev.sectorsize;
    sectors[sector].is_erased = -1;
    sectors[sector].is_protected = 0;
  }

  bank->sectors = sectors;
  scqspi_info->probed = true;

  return ERROR_OK;
}

static int scqspi_auto_probe(struct flash_bank *bank) {
  struct scqspi_flash_bank *scqspi_info = bank->driver_priv;

  LOG_DEBUG("%s\n", __func__);

  if (scqspi_info->probed)
    return ERROR_OK;

  return scqspi_probe(bank);
  ;
}

static int scqspi_info(struct flash_bank *bank,
                       struct command_invocation *cmd) {
  struct scqspi_flash_bank *scqspi_info = bank->driver_priv;

  LOG_DEBUG("%s", __func__);

  if (!(scqspi_info->probed)) {
    command_print_sameline(cmd, "\nQSPI flash bank not probed yet\n");
    return ERROR_FLASH_BANK_NOT_PROBED;
  }

  command_print_sameline(
      cmd,
      "flash \'%s\', device id = 0x%06" PRIx32 ", flash size = %" PRIu32
      "%sB\n(page size = %" PRIu32 ", read = 0x%02" PRIx8
      ", qread = 0x%02" PRIx8 ", pprog = 0x%02" PRIx8
      ", mass_erase = 0x%02" PRIx8 ", sector size = %" PRIu32
      " %sB, sector_erase = 0x%02" PRIx8 ")",
      scqspi_info->dev.name, scqspi_info->dev.device_id,
      bank->size / 4096 ? bank->size / 1024 : bank->size,
      bank->size / 4096 ? "Ki" : "", scqspi_info->dev.pagesize,
      scqspi_info->dev.read_cmd, scqspi_info->dev.qread_cmd,
      scqspi_info->dev.pprog_cmd, scqspi_info->dev.chip_erase_cmd,
      scqspi_info->dev.sectorsize / 4096 ? scqspi_info->dev.sectorsize / 1024
                                         : scqspi_info->dev.sectorsize,
      scqspi_info->dev.sectorsize / 4096 ? "Ki" : "",
      scqspi_info->dev.erase_cmd);

  return ERROR_OK;
}

static int scqspi_protect_check(struct flash_bank *bank) {
  /* Nothing to do. Protection is only handled in SW. */
  return ERROR_OK;
}

static int scqspi_flash_blank_check(struct flash_bank *bank) {
  /* Not implemented */
  return ERROR_OK;
}

static const struct command_registration scqspi_command_handlers[] = {
    {
        .name = "scqspi",
        .mode = COMMAND_ANY,
        .help = "scqspi flash command group",
        .usage = "",
    },
    COMMAND_REGISTRATION_DONE};

const struct flash_driver scqspi_flash = {
    .name = "scqspi",
    .commands = scqspi_command_handlers,
    .flash_bank_command = scqspi_flash_bank_command,
    .erase = scqspi_erase,
    .write = scqspi_write,
    .read = scqspi_read,
    .probe = scqspi_probe,
    .auto_probe = scqspi_auto_probe,
    .erase_check = scqspi_flash_blank_check,
    .protect_check = scqspi_protect_check,
    .info = scqspi_info,
    .free_driver_priv = default_flash_free_driver_priv,
};