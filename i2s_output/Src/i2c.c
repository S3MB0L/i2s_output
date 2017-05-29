/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 198;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */
		
  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, Audio_SCL_Pin|Audio_SDA_Pin);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void dac_control_i2c_init(void)
{
	uint32_t delaycount;
	uint8_t CodecCommandBuffer[5];

	uint8_t regValue = 0xFF;

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
	delaycount = 1000000;
	while (delaycount > 0)
	{
		delaycount--;
	}
	//keep codec OFF
	CodecCommandBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
	CodecCommandBuffer[1] = 0x01;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	//begin initialization sequence (p. 32)
	CodecCommandBuffer[0] = 0x00;
	CodecCommandBuffer[1] = 0x99;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x47;
	CodecCommandBuffer[1] = 0x80;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	regValue = read_codec_register(0x32);

	CodecCommandBuffer[0] = 0x32;
	CodecCommandBuffer[1] = regValue | 0x80;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	regValue = read_codec_register(0x32);

	CodecCommandBuffer[0] = 0x32;
	CodecCommandBuffer[1] = regValue & (~0x80);
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x00;
	CodecCommandBuffer[1] = 0x00;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);
	//end of initialization sequence

	CodecCommandBuffer[0] = CODEC_MAP_PWR_CTRL2;
	CodecCommandBuffer[1] = 0xAF;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_PLAYBACK_CTRL1;
	CodecCommandBuffer[1] = 0x70;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_CLK_CTRL;
	CodecCommandBuffer[1] = 0x81; //auto detect clock
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_IF_CTRL1;
	CodecCommandBuffer[1] = 0x07;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x0A;
	CodecCommandBuffer[1] = 0x00;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x27;
	CodecCommandBuffer[1] = 0x00;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = 0x1A | CODEC_MAPBYTE_INC;
	CodecCommandBuffer[1] = 0x0A;
	CodecCommandBuffer[2] = 0x0A;
	dac_control_i2c_send_data(CodecCommandBuffer, 3);

	CodecCommandBuffer[0] = 0x1F;
	CodecCommandBuffer[1] = 0x0F;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);

	CodecCommandBuffer[0] = CODEC_MAP_PWR_CTRL1;
	CodecCommandBuffer[1] = 0x9E;
	dac_control_i2c_send_data(CodecCommandBuffer, 2);


}
	
void dac_control_i2c_send_data(uint8_t controlBytes[], uint8_t numBytes)
{
		uint16_t bytesSent=0;
	//HAL_I2C_Mem_Write(&hi2c1,CODEC_I2C_ADDRESS,
	for(numBytes=0;bytesSent<numBytes;bytesSent++){
		if(HAL_I2C_Master_Transmit(&hi2c1,CODEC_I2C_ADDRESS,&controlBytes[bytesSent],1,10000)!=HAL_OK)
			_Error_Handler(__FILE__,__LINE__);
	}
}

uint8_t read_codec_register(uint8_t mapbyte)
{
//	uint8_t receivedByte = 0;

//	HAL_I2C_Master_Transmit(&hi2c1,
//	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
//	{
//		//just wait until no longer busy
//	}

//	I2C_GenerateSTART(CODEC_I2C, ENABLE);
//	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB))
//	{
//		//wait for generation of start condition
//	}

//	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Transmitter);
//	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//	{
//		//wait for end of address transmission
//	}

//	I2C_SendData(CODEC_I2C, mapbyte); //sets the transmitter address
//	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
//	{
//		//wait for transmission of byte
//	}

//	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

//	while (I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
//	{
//		//just wait until no longer busy
//	}

//	I2C_AcknowledgeConfig(CODEC_I2C, DISABLE);

//	I2C_GenerateSTART(CODEC_I2C, ENABLE);
//	while (!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_SB))
//	{
//		//wait for generation of start condition
//	}

//	I2C_Send7bitAddress(CODEC_I2C, CODEC_I2C_ADDRESS, I2C_Direction_Receiver);
//	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//	{
//		//wait for end of address transmission
//	}

//	while (!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
//	{
//		//wait until byte arrived
//	}
//	receivedByte = I2C_ReceiveData(CODEC_I2C);

//	I2C_GenerateSTOP(CODEC_I2C, ENABLE);

//	return receivedByte;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
