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
 * <driverPath>
 */
FLASH_BANK_COMMAND_HANDLER(scqspi_flash_bank_command) {
  LOG_INFO("%s", __func__);
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
  LOG_INFO("%s", __func__);
  return ERROR_OK;
}

static int scqspi_info(struct flash_bank *bank,
                       struct command_invocation *cmd) {
  LOG_INFO("%s", __func__);
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