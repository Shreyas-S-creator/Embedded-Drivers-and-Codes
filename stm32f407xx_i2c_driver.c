#include <stdint.h>
#include "stm32f407xx.h"
//#include "stm32f407xx_i2c_driver.h"  // as i have included i2c driver header file in the mcu specific header , i got that conflicting types error ..

uint32_t AHB_Prescalar[9]={2,4,8,16,32,64,128,256,512};
uint16_t APB1_Prescalar[4]={2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t*);//declared not in header coz just a helper funct
static void I2C_ExecuteAddressPhase(I2C_RegDef_t*, uint8_t,uint8_t);
static void I2C_ClearADDRFlag(I2C_Handle_t*);
static void I2C_GenerateStartCondition(I2C_RegDef_t*);

static void I2C_MasterHandleTxEInterrupt(I2C_Handle_t* pI2CHandle);
static void I2C_MasterHandleRxNEInterrupt(I2C_Handle_t* pI2CHandle);
// Helper Functions

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx,uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1|= (1<< I2C_CR1_START);
}


static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr,uint8_t ReadorWrite)
{
	if(ReadorWrite == I2C_READ){
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr |= I2C_READ; //SlaveAddr is Slave address + r/nw bit
		pI2Cx->DR = SlaveAddr;
	}else if (ReadorWrite == I2C_WRITE){
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr |= I2C_WRITE; //SlaveAddr is Slave address + r/nw bit
		pI2Cx->DR = SlaveAddr;
	}

}


static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageACKing(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{ 	// busy in TX
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}


void I2C_GenerateStopCondition(I2C_RegDef_t*  pI2Cx)
{
	pI2Cx->CR1 |= 1<< I2C_CR1_STOP;
}


void I2C_ManageACKing(I2C_RegDef_t* pI2Cx , uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= 1 << I2C_CR1_ACK;
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

// Peripheral Clock Control
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE){
		if(pI2Cx==I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx==I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx==I2C3){
			I2C3_PCLK_EN();
		}
	}else{
		if(pI2Cx==I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx==I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx==I2C3){ // ### DONT use ELSE because it will be run even if any random things selected.
			I2C3_PCLK_DI();		// like even if 12c4 is given this will execute if we use else ..###
		}
	}
}

// Peripheral control
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE){
		pI2Cx->CR1 |=(1<<I2C_CR1_PE);
	}else {
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}
uint32_t Get_PLLOutputClock(void){
	uint32_t PLLFreq;


	return PLLFreq;
}
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1;
	uint32_t sysclk,temp1,apb1P,ahb1P;
	temp1= (RCC->CFGR>>2 & 0x3);
	if(temp1==0){
		sysclk=16000000;
	}else if(temp1==1){
		sysclk=8000000;
	}else if(temp1==2){
		sysclk=Get_PLLOutputClock();
	}

	temp1= (RCC->CFGR>>4 & 0xF);
	if(temp1<8){
		ahb1P=1;
	}else if (temp1<16){
		ahb1P=AHB_Prescalar[temp1-8];
	}
	temp1= (RCC->CFGR>>10 & 0x7);
	if(temp1<4){
		apb1P=1;
	}else if (temp1<8){
		apb1P=APB1_Prescalar[temp1-4];
	}

	pclk1=(sysclk/ahb1P)/apb1P;

	return pclk1;
}

void I2C_Init(I2C_Handle_t* I2CHandle)
{
	// # When using one register only once for assignment , no need to use |=
	uint32_t temp =0;
	I2C_PeriClockControl(I2CHandle->pI2Cx,ENABLE);

	//ACK Bit of CR1 reg
	temp|= I2CHandle->I2C_Config.I2C_ACKControl<<I2C_CR1_ACK;
	I2CHandle->pI2Cx->CR1 =temp;							//can't set ACK until PE=1

	//CR2 FREQ in cr2
	temp=0;
	temp=RCC_GetPCLK1Value()/1000000U;
	I2CHandle->pI2Cx->CR2 =(temp & 0x3F);

	//OAR1 for slave address
	temp =0;
	temp |=I2CHandle->I2C_Config.I2C_DeviceAddress<<1; // missed also here
	temp |=(1<<14);// compulsory but don't know why    ## missed |= here
	I2CHandle->pI2Cx->OAR1 =temp ; // don't use |= because we are using separate registers

	//CCR
	temp=0;
	uint16_t ccr_value=0;
	if(I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		ccr_value = RCC_GetPCLK1Value() / (2*I2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= (ccr_value & 0xFFF);									// BY missing ccr_value and using just temp. i wouldve missed this masking !
	}
	else if(I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_FM)
	{
		//Fast mode
		temp |=(1<< I2C_CCR_FS);
		temp |= I2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY;

		if(I2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_FM_DUTY_2){
			ccr_value= (RCC_GetPCLK1Value()/(3*I2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else if (I2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_FM_DUTY_16_9)
		{
			ccr_value= (RCC_GetPCLK1Value()/(25*I2CHandle->I2C_Config.I2C_SCLSpeed));// Nothing left shifted because starts from 0th bit
		}
		temp |= (ccr_value & 0xFFF);
	}
	I2CHandle->pI2Cx->CCR = temp;

	//TRISE
	if(I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		temp = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}else	// because of else if i skipped fast mode 2
	{
		temp = ((RCC_GetPCLK1Value() *300)/1000000000U) + 1;
	}
	I2CHandle->pI2Cx->TRISE = (temp & 0x3F);
}


void I2C_MasterSendData(I2C_Handle_t* pI2CHandle,uint8_t* pTxbuffer , uint32_t Len, uint8_t SlaveAddr)
{
	//1.Generate start
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
		//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_SB)));// we are reading SR1

	// now that we've read from sr1 we should write to DR to clear SB

	//3. Send the address of the slave with r/w bit set to R(1) (total 8 bits )
	//pI2CHandle->pI2Cx->DR = SlaveAddr; missed r/w here

	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr,Write_Onto_Bus);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( !(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_ADDR)));

	//5. To clear ADDR read SR1 followed by SR2 .
	//Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)

	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send data until length becomes 0
	while(Len>0)
	{
		//while(!(pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TxE))); Better to use get flag status
		while(!(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
		//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
		//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(!(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_TXE)));
	while(!(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_BTF)));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
		//   Note: generating STOP, automatically clears the BTF

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t* pRxbuffer , uint32_t Len, uint8_t SlaveAddr)
{
	//1.Generate start
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
			//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_SB)));// we are reading SR1

	//3. Send the address of the slave with r/w bit set to R(1) (total 8 bits )
		//pI2CHandle->pI2Cx->DR = SlaveAddr; missed r/w here

	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr,Read_From_Bus); // how do we get the slave address?

	// wait until addr set to know address phase was successful

	while( !(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_ADDR)));

	if(Len == 1)
	{
		// Reset ACK
		I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);

		// stop moved from here because stop only after rxne=1 is confirmed

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// check RXNE before RX
		while( !(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_RXNE)));

		// STOP condition

		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Copy data into Buffer

		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle); // missed this .. if missed will cause clock stretching

		for( uint32_t i = Len ; i > 0; i--)
		{
			// check RXNE before RX
			while( !(I2C_GetFlagStatus((pI2CHandle->pI2Cx),I2C_FLAG_RXNE)));

			if(i == 2)
			{
				// Reset ACK
				I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);

				// STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// Copy data into Buffer

			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			pRxbuffer++;
		}
	}
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageACKing(pI2CHandle->pI2Cx, ENABLE);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVFEN);


		//Implement the code to enable ITERREN Control Bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN); // for RxNE and TxE Interrupts
	}

	return busystate;

}


/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;			//Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVFEN);


		//Implement the code to enable ITERREN Control Bit

		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN); // for RxNE and TxE Interrupts
	}

	return busystate;
}



void I2C_IRQInterruptConfig(uint8_t IRQNumber ,uint8_t EnOrDi)
{
if(EnOrDi == ENABLE){
	if(IRQNumber <= 31){

		*NVIC_ISER0 |= (1 << IRQNumber);

	}else if(IRQNumber <= 31 && IRQNumber <= 63){

		*NVIC_ISER1 |= (1 << IRQNumber%32);

	}else if(IRQNumber <= 63 && IRQNumber <=95){

		*NVIC_ISER2 |= (1 << IRQNumber%64);

	}

}else{
	if(IRQNumber <= 31){

		*NVIC_ICER0 |= (1 << IRQNumber);

	}else if(IRQNumber <= 31 && IRQNumber <= 63){

		*NVIC_ICER1 |= (1 << IRQNumber%32);

	}else if(IRQNumber <= 63 && IRQNumber <=95){

		*NVIC_ICER2 |= (1 << IRQNumber%64);

	}

}
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)  // priority from 0 to 15 , but IRQNumber 0 to 255
{
	//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;				// total 60 registers for 240 irq numbers
		uint8_t iprx_section  = IRQNumber %4 ;		// for each irq number 1 field out of 4 in a register out of 60 registers

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

static void I2C_MasterHandleRxNEInterrupt(I2C_Handle_t* pI2CHandle)
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageACKing(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}
static void I2C_MasterHandleTxEInterrupt(I2C_Handle_t* pI2CHandle)
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
		// Rest about stop condition is taken care when both DR and Shift Reg get empty , BTF INT happens



	}

}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageACKing(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVFEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = 0;
	pI2CHandle->TxLen = 0;
}


void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	// Check for control bits
	uint8_t temp1 = (pI2CHandle->pI2Cx->CR2) && (1 << I2C_CR2_ITEVFEN) ;
	uint8_t temp2 = (pI2CHandle->pI2Cx->CR2) && (1 << I2C_CR2_ITBUFEN) ;

	// for sb
	uint8_t temp3 = pI2CHandle->pI2Cx->SR1 && (1<< I2C_SR1_SB);

	if(temp1 && temp3){
		// INT is caused by setting of SB . SB can bee set only in master and not in slave
		// start is done , so lets do the next phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr ,I2C_WRITE);

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr ,I2C_READ);
		}
	}

	// for addr
	temp3 = pI2CHandle->pI2Cx->SR1 && (1<< I2C_SR1_ADDR); // read from sr1 done to clr addr

	if(temp1 && temp3){
		// means address phase is successful , clear so stretching doesnt happen
		I2C_ClearADDRFlag(pI2CHandle); // shouldnt be done here directly so modified the function #coz this should be done in RxNE INT case
	}

	// for STOPF
	temp3 = pI2CHandle->pI2Cx->SR1 && (1<< I2C_SR1_STOPF);  // to clr read from sr1 done
	if(temp1 && temp3){
		//only set in slave mode
		// INT caused by STOPF , to clr read from sr1 done , so write into cr1 to clr

		pI2CHandle->pI2Cx->CR1 |= 0X0000;  // wrote but didnt affect the contents of the regsiter

		//Notify application that sto[p signal is detected

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	// for BTF

	temp3 = pI2CHandle->pI2Cx->SR1 && (1<< I2C_SR1_BTF);
	if(temp1 && temp3){
		// INT caused by BTF
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TxE) ){

				//if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR2_TxE)) should have done
				if(pI2CHandle->TxLen == 0){

					// generate stop only if no repeated start
					if(pI2CHandle->Sr == I2C_DISABLE_SR){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//clear content of handle variables
					I2C_CloseSendData(pI2CHandle);

					//let the application know that tx ended
					I2C_ApplicationEventCallback(pI2CHandle , I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TxE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTxEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}
	// for RXNE
	temp3 = pI2CHandle->pI2Cx->SR1 && (1<< I2C_SR1_RxNE);

	if(temp1 && temp2 && temp3){
		//RxNE is set
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)){

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

				I2C_MasterHandleRxNEInterrupt(pI2CHandle);

			}
		}else {
			// check if slave in Receiver mode
			 if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))){

				 I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			 }
		}
	}
}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag

		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error

		I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_TIMEOUT);

	}
}

void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx , uint8_t Data)
{
	pI2Cx->DR = Data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx)
{
	return pI2Cx->DR;
}

