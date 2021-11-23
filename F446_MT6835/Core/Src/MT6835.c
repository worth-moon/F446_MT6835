/*
 * MT6835.c
 *
 *  Created on: Nov 22, 2021
 *      Author: Heng
 */

#include "MT6835.h"


extern SPI_HandleTypeDef hspi1;

uint32_t MT_rdAngle(){

	uint8_t Angle[3];
	Angle[0]=MT_rdReg(REG_ANGLE1);
	Angle[1]=MT_rdReg(REG_ANGLE2);
	Angle[2]=MT_rdReg(REG_ANGLE3);

	return Angle[2]<<12|Angle[1]<<4|Angle[0]>>4;
}

uint8_t MT_rdID(){

	return MT_rdReg(REG_ID);

}

void MT_wrID(uint8_t data){

	MT_wrReg(REG_ID,data);
}

void MT_wrABZRES(uint16_t res){

	uint8_t resHi,resLow;
	uint8_t temp;
	resHi=res>>6;


	temp=MT_rdReg(REG_ABZ_RES1)&0x3;

	resLow=(res& 0x003F)<<2|temp;

	MT_wrReg(REG_ABZ_RES2,resHi);
	MT_wrReg(REG_ABZ_RES1,resLow);

}

uint16_t MT_rdABZRES(){

	uint8_t resHi,resLow;

	resHi=MT_rdReg(REG_ABZ_RES2);
	resLow=MT_rdReg(REG_ABZ_RES1);

	return resHi<<6|resLow>>2;
}

uint8_t MT_rdUVW(){

	return MT_rdReg(REG_UVW)&0xF;
}

void MT_wrUVW(uint8_t UVW){

	uint8_t temp;

	temp=MT_rdReg(REG_UVW)&0xF0;
	MT_wrReg(REG_UVW,temp|(UVW & 0x0F));

}


void MT_wrReg(uint8_t reg, uint8_t data){

	uint8_t txbuf[3];
	uint16_t tx;
	HAL_StatusTypeDef status;

	tx=CMD_WR<<12|reg;

	txbuf[0]=tx>>8;
	txbuf[1]=tx & 0x00FF;
	txbuf[2]=data;

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

	status=HAL_SPI_Transmit(&hspi1,txbuf,3,10);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

}

uint8_t MT_wrEEPROM(){

	uint8_t txbuf[2],rxbuf[1];
	HAL_StatusTypeDef status;

	txbuf[0]=CMD_EEPROM<<4;
	txbuf[1]=0;

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	status=HAL_SPI_Transmit(&hspi1,txbuf,2,10);
	HAL_Delay(1);
	status=HAL_SPI_Receive(&hspi1,rxbuf,1,10);

	return rxbuf[0];

}




uint8_t MT_rdReg(uint8_t reg){

	uint8_t txbuf[2];
	uint16_t tx;
	HAL_StatusTypeDef status;
	uint8_t rx;

	tx=CMD_RD<<12|reg;

	txbuf[0]=tx>>8;
	txbuf[1]=tx & 0x00FF;

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	status=HAL_SPI_Transmit(&hspi1,txbuf,2,10);
	HAL_Delay(1);
	status=HAL_SPI_Receive(&hspi1,&rx,1,10);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

	return rx;


}
