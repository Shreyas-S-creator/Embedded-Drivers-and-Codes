#include <stdint.h>
#include "stm32f407xx.h"

#define __vo    volatile

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4


#define I2C_SCL_SPEED_SM  100000
#define I2C_SCL_SPEED_FM  1000000

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9   1

#define FLAG_SET 	1
#define FLAG_RESET  0

#define I2C_ACK_ENABLE		ENABLE
#define I2C_ACK_DISABLE		DISABLE

#define I2C_ENABLE_SR 		ENABLE
#define I2C_DISABLE_SR		DISABLE

#define Read_From_Bus 	1
#define Write_Onto_Bus 	0

// Configuration structure

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
} I2C_Config_t;


//I2C Handle Structure

typedef struct{
	I2C_RegDef_t* pI2Cx;
	I2C_Config_t  I2C_Config;
	uint8_t 	  *pTxBuffer;
	uint8_t 	  *pRxBuffer;
	uint32_t 	  TxLen;
	uint32_t 	  RxLen;
	uint8_t 	  TxRxState;
	uint8_t  	  DevAddr;
	uint32_t 	  RxSize;
	uint8_t  	  Sr;
} I2C_Handle_t;

/*
* @ I2C_SCLSpeed
*/
#define  I2C_SCLSpeed_SM 100000
#define  I2C_SCLSpeed_FM4K 400000
#define  I2C_SCLSpeed_FM2K 200000

//API's

void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx,uint8_t EnOrDi);

void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2Cx);

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle,uint8_t* pTxbuffer , uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t* pRxbuffer , uint32_t Len, uint8_t SlaveAddr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle,uint8_t* pTxbuffer , uint32_t Len, uint8_t SlaveAddr ,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pRxbuffer , uint32_t Len, uint8_t SlaveAddr ,uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx , uint8_t Data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx );


void I2C_IRQInterruptConfig(uint8_t IRQNumber ,uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle);

void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx,uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx,uint32_t FlagName);

void I2C_GenerateStopCondition(I2C_RegDef_t*);

void I2C_ManageACKing(I2C_RegDef_t*, uint8_t);

void I2C_ApplicationEventCallback(I2C_Handle_t *PI2CHandle,uint8_t AppEv);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 * *@I2C related flag definitions
 */

#define I2C_FLAG_BTF  	 	(1 << I2C_SR1_BTF)
#define I2C_FLAG_TXE  	 	(1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE   	(1 << I2C_SR1_RxNE)
#define I2C_FLAG_SB 		 (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR   	(1 << I2C_SR1_ADDR)
#define I2C_FLAG_STPF   	(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR  		 (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO  		 (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF   		(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR   		(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT   	(1 << I2C_SR1_TIMEOUT)


/*
 * *@I2C states
 */
#define I2C_READY			0
#define I2C_BUSY_IN_TX		1
#define I2C_BUSY_IN_RX		2

#define I2C_WRITE 			0
#define I2C_READ			1

/*
 * @I2C Application Events Macros
 */

#define I2C_EV_TX_CMPLT 	0
#define I2C_EV_RX_CMPLT 	1
#define I2C_EV_STOP 		2
#define I2C_ER_BERR			3
#define I2C_ER_ARLO			4
#define I2C_ER_AF			5
#define I2C_ER_OVR			6
#define I2C_ER_TIMEOUT		7
#define I2C_EV_DATA_REQ 	8
#define I2C_EV_DATA_RCV 	9

