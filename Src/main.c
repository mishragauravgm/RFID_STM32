/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mfrc522.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

typedef enum{
	MI_OK = 0,
	MI_NOTAGERR,
	MI_ERR} mfrc522_status_t;
/* Private variables ---------------------------------------------------------*/
//The below typedef would be used in a lot of places as status so defined as an enum00
int check1,check2,check3,check4,check5,check6,check7,check8,check9,check10 = 0;
uint8_t cardDetect=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
mfrc522_status_t mfrc522_AntiColl(uint8_t*);
void mfrc522_Halt(void);
void mfrc522_antennaOn(void);
void mfrc522_antennaOff(void);
void mfrc522_Init(void);
mfrc522_status_t mfrc522_check(uint8_t*);
	
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	check1=1;
	mfrc522_Init();
	check2=1;
	uint8_t ID[5];    //={0x45,0x26,0x51,0x20,0x00};
	//char buffer[50];
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
		
		check3=1;
		
	// Check if any card is detected
	if(mfrc522_check(ID) == MI_OK)
	{
		cardDetect = 1;
		//printf("0x0%02x\n0x0%02x\n0x0%02x\n0x0%02x\n0x0%02x",ID[0],ID[1],ID[2],ID[3],ID[4]);
	}
		else{
		cardDetect = 2;
		}
		
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA8   ------> RCC_MCO
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void enableChip(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

void disableChip(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void mfrc522_writeRegister(uint8_t addr, uint8_t val)//checked
{
	check8=1;
	enableChip();
	uint8_t tobesent[2];
	tobesent[0]=(addr<<1) & 0x7E;
	tobesent[1] = val;
	HAL_SPI_Transmit(&hspi1, &tobesent[0], sizeof(tobesent[0]), 100);
	HAL_SPI_Transmit(&hspi1, &tobesent[1], sizeof(tobesent[1]),100);
	
	disableChip();
}


uint8_t mfrc522_readRegister(uint8_t addr)
{
	uint8_t val;
	addr=((addr<<1)&0x7E) | 0x80; 
	enableChip();
	HAL_SPI_Transmit(&hspi1,&addr,sizeof(addr),50);
	HAL_SPI_Receive(&hspi1,&val,sizeof(val),50);
	disableChip();
	return val;
}


void mfrc522_SetBitMask(uint8_t reg, uint8_t mask)//checked
{
	mfrc522_writeRegister(reg,mfrc522_readRegister(reg)|mask);
	
}


void mfrc522_ClearBitMask(uint8_t reg, uint8_t mask) //checked
{
	mfrc522_writeRegister(reg,mfrc522_readRegister(reg)&~mask);
}	



void mfrc522_antennaOn(void)//checked
{
	uint8_t temp;
	temp = mfrc522_readRegister(TxControlReg);
	
	if(!(temp & 0x03))
	{
		mfrc522_SetBitMask(TxControlReg,0x03);
	}
check9=1;
}


void mfrc522_antennaOff(void) //checked
{
	mfrc522_ClearBitMask(TxControlReg,0x03);
}


void mfrc522_reset(void) //checked
{
	mfrc522_writeRegister(CommandReg,SoftReset_CMD);
}


void mfrc522_Init(void)//checked
{
	check5=1;
	mfrc522_reset();
	check6=1;
	mfrc522_writeRegister(TModeReg,0x8D);
	mfrc522_writeRegister(TPrescalerReg,0x3E);
	mfrc522_writeRegister(TReloadReg_2,30);
	mfrc522_writeRegister(TReloadReg_1,0);
	check7=1;
	/* 48dB gain */
	
	mfrc522_writeRegister(RFCfgReg,0x70);
	
	mfrc522_writeRegister(TxASKReg,0x40);
	mfrc522_writeRegister(ModeReg, 0x3D);
	
	mfrc522_antennaOn();
}


mfrc522_status_t mfrc522_toCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen)//checked
{
	mfrc522_status_t status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;
	
	switch(command){
		case(MFAuthent_CMD):
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case(Transceive_CMD):
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
	}

	mfrc522_writeRegister(ComIEnReg,irqEn | 0x80);
	mfrc522_ClearBitMask(ComIrqReg,0x80);
	mfrc522_SetBitMask(FIFOLevelReg,0x80);
	
	mfrc522_writeRegister(CommandReg,Idle_CMD);
	
	//Writing data to the FIFO
	for(i=0;i< sendLen; i++)
	{
		mfrc522_writeRegister(FIFODataReg,sendData[i]);
	}
	
	//Execute the command
	mfrc522_writeRegister(CommandReg, command);
	if(command == Transceive_CMD)
	{
		mfrc522_SetBitMask(BitFramingReg, 0x80); //StartSend = 1, transmission of data starts 
	}
	
	//Waiting to receive data to complete
	i=2000; //i according to the clock frequency adjustment, the operator MI card maximum waiting time 25ms?
	do
	{
		//CommIrqReq[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = mfrc522_readRegister(ComIrqReg);
		i--;
	}while((i!=0) && !(n&0x01) && !(n & waitIRq));
	
	mfrc522_ClearBitMask(BitFramingReg, 0x80); //StartSend = 0
	
	if(i!=0)
  {
		if(!(mfrc522_readRegister(ErrorReg) & 0x1B))
		{
			status = MI_OK;
			if(n & irqEn & 0x01)
			{
				status = MI_NOTAGERR;
			}
			
			if(command == Transceive_CMD)
			{
				n=mfrc522_readRegister(FIFOLevelReg);
				lastBits = mfrc522_readRegister(ControlReg) & 0x07;
				if(lastBits){
					*backLen = (n-1)*8+lastBits;
				}
				else{
				*backLen = n*8;
				}
				
				if(n==0){
					n=1;
				}
				
				if(n>MAX_LEN){
					n = MAX_LEN;
				}
				
				//Reading the received data in FIFO
				for(i=0;i<n;i++){
					backData[i]=mfrc522_readRegister(FIFODataReg);
				}

			}
		}
			else{
				status = MI_ERR;
		}
	}
	return status;
}


mfrc522_status_t mfrc522_Request(uint8_t reqMode, uint8_t* TagType) //checked
{
	mfrc522_status_t status;
	uint16_t backBits; //The Received Data Bits
	
	mfrc522_writeRegister(BitFramingReg,0x07);
	TagType[0]=reqMode;
	status = mfrc522_toCard(Transceive_CMD, TagType, 1, TagType, &backBits);
	
	if((status != MI_OK) || (backBits != 0x10))
	{
		status = MI_ERR;
	}
	else{status = MI_OK;}
	
	return status;
}


mfrc522_status_t mfrc522_check(uint8_t* id) //checked
{
	mfrc522_status_t status;
	status = mfrc522_Request(PICC_REQIDL,id);
	if(status==MI_OK)//Implies that card is detected 
		{
		status = mfrc522_AntiColl(id); //return card serial number 4 bytes
	}
	mfrc522_Halt(); //Command Card into hibernation
	
	return status;
}


mfrc522_status_t mfrc522_compare(uint8_t* cardID, uint8_t* compareID)
{
	uint8_t i;
	for(i=0;i<5;i++)
	{
		if(cardID[i] != compareID[i])
		{
			return MI_ERR;
		}
	}
	return MI_OK;
}



mfrc522_status_t mfrc522_AntiColl(uint8_t* serNum)//checked
{
	mfrc522_status_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;
	
	mfrc522_writeRegister(BitFramingReg,0x00); // TxLastBits = BitFramingReg[2..0]
	
	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = mfrc522_toCard(Transceive_CMD, serNum, 2, serNum, &unLen);
	
	if(status == MI_OK){
		// Check Card Serial Number
		for(i=0;i<4;i++){
			serNumCheck ^=serNum[i];
		}
		
		if(serNumCheck != serNum[i]){
			status = MI_ERR;
		}
	}
	return status;
}


void mfrc522_CalculateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData) //checked
{

	uint8_t i,n;
	mfrc522_ClearBitMask(DivIrqReg,0x04);		//CRCIrq = 0
	mfrc522_SetBitMask(FIFOLevelReg,0x80);   //Clear the FIFO pointer
	// Write into CommandReg the value CMD_IDLE
	
	//Writing data to the FIFO
	for(i=0;i<len;i++){
		mfrc522_writeRegister(FIFODataReg,*(pIndata+i));
	}
	mfrc522_writeRegister(CommandReg, CalcCRC_CMD);
	
	//Wait until CRC Calculation is complete
	i=0xFF;
	
	do{
		n = mfrc522_readRegister(DivIrqReg);
		i--;
	}while((i!=0) && !(n&0x04)); //CRCIrq = 1
	//Read CRC Calculation Result
	pOutData[0] = mfrc522_readRegister(CRCResultReg_2);
	pOutData[1] = mfrc522_readRegister(CRCResultReg_1);
}	


uint8_t mfrc522_SelectTag(uint8_t* serNum) //checked
{
	uint8_t i;
	uint8_t buffer[9];
	mfrc522_status_t status;
	uint16_t recvBits;
	uint8_t size;
	
	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for(i = 0;i<5;i++){
		buffer[i+2] = *(serNum+i);
	}
	mfrc522_CalculateCRC(buffer,7,&buffer[7]);
	status = mfrc522_toCard(Transceive_CMD, buffer, 9, buffer, &recvBits);
	if((status == MI_OK) && (recvBits == 0x18)){
		size = buffer[0];
}
	else{
		size = 0;
	}
	
	return size;
}


mfrc522_status_t mfrc522_auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) //checed
{
	mfrc522_status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12];
	//Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	
	for(i=0;i<6;i++){
	buff[i+2] = *(Sectorkey+i);
	}
	
	for(i=0;i<4;i++){
		buff[i+8] = *(serNum+i);
	}
	status = mfrc522_toCard(MFAuthent_CMD, buff, 12, buff, &recvBits);
	
	if((status!=MI_OK) || (!(mfrc522_readRegister(Status2Reg) & 0x08))){
		status = MI_ERR;
	}
	
	return status;
}


mfrc522_status_t mfrc522_Read(uint8_t blockAddr, uint8_t* recvData)//checked
{
	mfrc522_status_t status;
	uint16_t unLen;
	
	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	mfrc522_CalculateCRC(recvData,2,&recvData[2]);
	status = mfrc522_toCard(Transceive_CMD,recvData,4,recvData,&unLen);
	
	if((status!=MI_OK) || (unLen != 0x90)){
		status = MI_ERR;
	}
	return status;
}


mfrc522_status_t mfrc522_Write(uint8_t blockAddr, uint8_t* writeData) //checked
{
	mfrc522_status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18];
	
	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	
	mfrc522_CalculateCRC(buff, 2, &buff[2]);
	status = mfrc522_toCard(Transceive_CMD, buff, 4, buff, &recvBits);
	if((status!= MI_OK) || (recvBits!=4) || (buff[0] & 0x0F) != 0x0A){
		status = MI_ERR;
	}
	if(status == MI_OK){
	//Data to the FIFO write 16Byte
		for(i=0; i<16;i++){
			buff[i] = *(writeData+i);
		}
		mfrc522_CalculateCRC(buff, 16, &buff[16]);
		status = mfrc522_toCard(Transceive_CMD,buff, 18, buff, &recvBits);
		
		if((status!=MI_OK)|| (recvBits!=4) || ((buff[0] & 0x0F) != 0x0A)){
			status = MI_ERR;
		}
}
	
return status;
}



void mfrc522_Halt(void) //checked
	{
	uint16_t unLen;
	uint8_t buff[4];
	
	buff[0] = PICC_HALT;
	buff[1] = 0;
	mfrc522_CalculateCRC(buff, 2, &buff[2]);
	
	mfrc522_toCard(Transceive_CMD, buff, 4, buff, &unLen);
}

	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
