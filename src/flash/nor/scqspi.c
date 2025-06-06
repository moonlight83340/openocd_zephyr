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

struct scqspi_flash_bank {
  struct target *target;
  bool probed;
  uint32_t io_base;
  uint8_t spi_ss;
  struct flash_device dev;
};

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
  LOG_INFO("%s", __func__);
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