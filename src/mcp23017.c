/*
 * Copyright (c) 2021, Adam Boardman
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcp23017.h"
#include "mcp23017_private.h"
#define I2C_ADDR 0x21
#define I2C_ADDR_MCP23008 0x26

int output;
int last_input;

int write_register_mcp23008(uint8_t reg, uint8_t value) {
  uint8_t command[] = {reg, value};
  int result = i2c_write_blocking(i2c_default, I2C_ADDR_MCP23008, command, 2, false);
  if (result == PICO_ERROR_GENERIC) {
    return result;
  }
  return PICO_ERROR_NONE;
}

int write_register(uint8_t reg, uint8_t value) {
  uint8_t command[] = {reg, value};
  int result = i2c_write_blocking(i2c_default, I2C_ADDR, command, 2, false);
  if (result == PICO_ERROR_GENERIC) {
    return result;
  }
  return PICO_ERROR_NONE;
}

int read_register(uint8_t reg) {
  uint8_t buffer = 0;
  int result;
  result = i2c_write_blocking(i2c_default, I2C_ADDR, &reg, 1, true);
  mcp_debug("i2c_write_blocking: %d\n", result);
  if (result == PICO_ERROR_GENERIC) {
    return result;
  }

  result = i2c_read_blocking(i2c_default, I2C_ADDR, &buffer, 1, false);
  mcp_debug("i2c_read_blocking: %d, read: %d\n", result, buffer);
  if (result == PICO_ERROR_GENERIC)
    return result;

  return buffer;
}

int write_dual_registers(uint8_t reg, int value) {
  uint8_t command[] = {reg, (uint8_t)(value & 0xff),
                       (uint8_t)((value >> 8) & 0xff)};
  int result = i2c_write_blocking(i2c_default, I2C_ADDR, command, 3, false);
  if (result == PICO_ERROR_GENERIC) {
    return result;
  }
  return PICO_ERROR_NONE;
}

int read_dual_registers(uint8_t reg) {
  uint8_t buffer[2];
  int result;
  result = i2c_write_blocking(i2c_default, I2C_ADDR, &reg, 1, true);
  mcp_debug("i2c_write_blocking: %d\n", result);
  if (result == PICO_ERROR_GENERIC) {
    return result;
  }

  result = i2c_read_blocking(i2c_default, I2C_ADDR, buffer, 2, false);
  mcp_debug("i2c_read_blocking: %d, read: %d,%d\n", result, buffer[0],
            buffer[1]);
  if (result == PICO_ERROR_GENERIC)
    return result;

  return (buffer[1] << 8) + buffer[0];
}

int setup(bool mirroring, bool polarity) {
  int result;
  result = setup_bank_configuration(MCP23017_IOCONA, mirroring, polarity);
  if (result != 0)
    return PICO_ERROR_GENERIC;

  result = setup_bank_configuration(MCP23017_IOCONB, mirroring, polarity);
  return result;
}

int setup_bank_configuration(int reg, bool mirroring, bool polarity) {
  int ioConValue = 0;
  set_bit(&ioConValue, MCP23017_IOCON_BANK_BIT, false);
  set_bit(&ioConValue, MCP23017_IOCON_MIRROR_BIT, mirroring);
  set_bit(&ioConValue, MCP23017_IOCON_SEQOP_BIT, false);
  set_bit(&ioConValue, MCP23017_IOCON_DISSLW_BIT, false);
  set_bit(&ioConValue, MCP23017_IOCON_HAEN_BIT, false);
  set_bit(&ioConValue, MCP23017_IOCON_ODR_BIT, false);
  set_bit(&ioConValue, MCP23017_IOCON_INTPOL_BIT, polarity);
  return write_register(reg, ioConValue);
}

int get_last_interrupt_pin() {
  int intFlag;

  intFlag = read_dual_registers(MCP23017_INTFA); // also MCP23017_INTFB
  mcp_debug("INTF %d", intFlag);
  if (intFlag != PICO_ERROR_GENERIC) {
    for (int i = 0; i < 16; i++) {
      if (is_bit_set(intFlag, i)) {
        return i;
      }
    }
  }

  return PICO_ERROR_GENERIC;
}

int get_interrupt_values() {
  return read_dual_registers(MCP23017_INTCAPA); // will include MCP23017_INTCAPB
}

int update_input_values() {
  int result =
      read_dual_registers(MCP23017_GPIOA); // will include MCP23017_GPIOB
  if (result != PICO_ERROR_GENERIC) {
    last_input = result;
    result = PICO_ERROR_NONE;
  }
  return result;
}

bool get_input_pin_value(int pin) { return is_bit_set(last_input, pin); }

int get_address() { return I2C_ADDR; }

int set_io_direction(int direction) {
  return write_dual_registers(MCP23017_IODIRA, direction); // inc
                                                           // MCP23017_IODIRB
}

int set_pullup(int direction) {
  return write_dual_registers(
      MCP23017_GPPUA, direction); // inc MCP23017_GPPUB, direction >> 8);
}

int set_interrupt_type(int compare_to_reg) {
  return write_dual_registers(MCP23017_INTCONA,
                              compare_to_reg); // inc MCP23017_INTCONB
}

int enable_interrupt(int enabled) {
  return write_dual_registers(MCP23017_GPINTENA,
                              enabled); // inc MCP23017_GPINTENB
}

int set_all_output_bits(int all_bits) {
  output = all_bits;
  return write_dual_registers(MCP23017_GPIOA, all_bits); // inc MCP23017_GPIOB
}

void set_output_bit_for_pin(int pin, bool set) { set_bit(&output, pin, set); }

bool get_output_bit_for_pin(int pin) { return is_bit_set(output, pin); }

int flush_output() {
  return write_dual_registers(MCP23017_GPIOA, output); // inc MCP23017_GPIOB
}
