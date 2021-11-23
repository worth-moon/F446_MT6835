/*
 * MT6835.h
 *
 *  Created on: Nov 22, 2021
 *      Author: Heng
 */

#ifndef INC_MT6835_H_
#define INC_MT6835_H_

#include "stm32f4xx_hal.h"

enum {
  CMD_RD = (0b0011),  /**< user read register. */
  CMD_WR = (0b0110),  /**< user write register. */
  CMD_EEPROM=(0b1100),/**< user erase and program EEPROM. */
  CMD_ZERO = (0b0101),  /**< AUTO setting zero. */
  CMD_BURST= (0b1010),  /**< burst mode. */
};


enum {
  REG_ID=(0x001),
  REG_ANGLE3 = (0x003),
  REG_ANGLE2 = (0x004),
  REG_ANGLE1 = (0x005),
  REG_CRC = (0x006),
  REG_ABZ_RES2 = (0x007),
  REG_ABZ_RES1 = (0x008),
  REG_ZERO2 = (0x009),
  REG_ZERO1 = (0x00A),
  REG_UVW=(0x00B),
  REG_PWM=(0x00C),
  REG_HYST=(0x00D),
  REG_AUTOCAL=(0x00E),
};


uint8_t MT_rdReg(uint8_t reg);
void MT_wrReg(uint8_t reg, uint8_t data);
uint8_t MT_rdID();
void MT_wrID(uint8_t data);
uint32_t MT_rdAngle();
void MT_wrABZRES(uint16_t res);
uint16_t MT_rdABZRES();
uint8_t MT_rdUVW();
void MT_wrUVW(uint8_t UVW);
uint8_t MT_wrEEPROM();

#endif /* INC_MT6835_H_ */
