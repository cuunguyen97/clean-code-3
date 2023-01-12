/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2021 Lumi
 * Copyright (c) 2023
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: main.c
 *
 * Description:
 *
 * Author: CuuNV
 *
 * Last Changed By:  $Author: CuuNV $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $January 12, 2023
 *

 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <stm32f401re_usart.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_rcc.h>
#include <stm32f401re_adc.h>

#include <serial.h>
#include <uartcmd.h>
#include <buff.h>

#include <timer.h>
#include <eventman.h>
#include <led.h>
#include <melody.h>
#include <lightsensor.h>
#include <temhumsensor.h>

#include <button.h>
#include <eventbutton.h>
#include <Ucglib.h>
//Include libraries of NVIC
#include <misc.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
//define of USART2
#define USART_BAUDRATE					57600
#define USART_PIN_TX					GPIO_Pin_2
#define USART_PINSOURCE_TX				GPIO_PinSource2
#define USART_PIN_RX					GPIO_Pin_3
#define USART_PINSOURCE_RX				GPIO_PinSource3
#define USART_PORT						GPIOA
#define USART_GPIO_RCC					RCC_AHB1Periph_GPIOA
#define USART_RCC						RCC_APB1Periph_USART2
#define USART_AF						GPIO_AF_USART2
#define USARTX							USART2
// Define other------------------------------------------------------------------
#define CYCLE_SENT_DATA					1000 //Thoi gian quet sensor
#define PERIOD_SCAN_MULTISENSOR			5000 //ms
#define LED_LEVEL_UP					0X01u
#define LED_LEVEL_DOWN					0X00u
#define TIME_STEP_LEVEL					20 //20ms
//Buffer Data RX use to UART-----------------------------------------------------
#define SIZE_BUFF_DATA_RX				256
#define START_BYTE						0XB1
#define ACK_BYTE						0X06
#define NACK_BYTE						0X15


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static uint8_t pByBuffDataRx[SIZE_BUFF_DATA_RX] = {0};
static buffqueue_t pUartRxQueue = {0};
typedef void (* serial_handle_event)(void *);
// Cac bien dung cho qua trinh phan tich du lieu---------------------------------
#define RX_MAX_INDEX_BYTE	30
static uint8_t g_byRxState;
static uint8_t g_byRxDataByte[RX_MAX_INDEX_BYTE];
static uint8_t g_byRxCheckXor;
static uint8_t g_byRxIndexByte;
//-----------------------------------------------------------------------------//
static uint32_t g_dwTimeCurrent,g_dwTimeInitial;
static uint32_t g_dwTimeTotal;
static uint8_t g_byHumidity,g_Temperature; //Cac bien chua gia tri cua sensor quet duoc
static uint16_t g_wLight;
static ucg_t ucg;
static char srcTemp[20] = "";	// Mang str chua gia tri nhiet do
static char srcHumi[20] = "";	// Mang str chua gia tri do am
static char srcLight[20] = "";	// Mang str chua gia tri anh sang
static SSwTimer idTimerSensorScan = NO_TIMER;
uint8_t	g_byIdTimerSetLevelLed = NO_TIMER;
static uint8_t g_byState = 0; 		//trang thai dung de cap nhap hoat dong cua timer
static uint8_t g_byPwmLedLevel = 0; //Bien chua gia tri do sang cua led khi nhan giu phim


typedef enum{
	EVENT_EMPTY,
	EVENT_APP_INIT,
	EVEN_APP_FLUSHMEM_READY,
} event_api_e,*event_apt_p;
typedef enum{
	STATE_APP_STARTUP,
	STATE_APP_IDLE,
	STATE_APP_RESET
}state_app_e;
state_app_e eCurrentState = STATE_APP_STARTUP;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void appInitCommon(void);

static void setStateApp(state_app_e state);

static state_app_e getStateApp(void);

static void appStateManager(event_api_e event);

void taskMultiSensorScan(void *arg);

static void deviceStateMachine(event_button_t event);

//void delayMs(int ms);
// Set level led use timer
void setLevelLed(uint8_t byLevel);

void stopLevelChange(void);

// cac ham xu ly du lieu tu PC xuong device qua UART
void ledCmdSetState(uint8_t btLedId, \
					 uint8_t byLedColor, \
					 uint8_t byLedNumBlink, \
					 uint8_t byLedInterval, \
					 uint8_t byLedLastState);

void lcdCmdSetState(char *text);

void buttonCmdState(uint8_t byButtonId, uint8_t byButtonState);

void buzzerCmdSetState(uint8_t byNtime);

static void procUartCmd(void *arg);
//-----------------------------------------------------------------------------------------
static void usart2Init(void);

static void serialUartInit(void);

static uint8_t pollRxBuff(void);

static void processSerialUartReceiver(void);

static void processUartReceiveCommand_ControlLed(cmd_receive_p pCmd);

static void processUartReceiveCommand_ControlButton(cmd_receive_p pCmd);

static void processUartReceiveCommand_ControlLCD(cmd_receive_p pCmd);

static void processUartReceiveCommand_ControlBuzzer(cmd_receive_p pCmd);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/
int main(void)
{
	appInitCommon();
	setStateApp(STATE_APP_STARTUP);
	EventSchedulerAdd(EVENT_APP_INIT);
    /* Loop forever */
	while(1)
	{
		processTimerScheduler();
		processEventScheduler();
		processSerialUartReceiver();
		}
}
/**
 * @func   appInitCommon
 * @brief  System initialization
 * @param  None
 * @retval None
 */
void appInitCommon(void)
{

	SystemCoreClockUpdate();
	TimerInit();

	//EventSerial_Init();
	serialUartInit();

	EventSchedulerInit(appStateManager);
	EventButton_Init();
	BuzzerControl_Init();
	LedControl_Init();
	LightSensor_Init(ADC_READ_MODE_DMA);
	TemHumSensor_Init();

	//Khoi tao cai dat cho lcd
	Ucglib4WireSWSPI_begin(&ucg, UCG_FONT_MODE_SOLID);
	ucg_ClearScreen(&ucg);
	ucg_SetColor(&ucg, 0, 255, 255, 255);//khoi tao mau chu la mau trang
	ucg_SetColor(&ucg, 1, 0, 0, 0);// khoi tao mau backgroud la mau den
	ucg_SetRotate180(&ucg);
}
/**
 * @func   usart2Init
 * @brief  Usart initialization
 * @param  None
 * @retval None
 */
static void usart2Init(void)
{
	/*
	 * USART: USART2
	 * TX: PA2
	 * RX: PA3
	 */
	GPIO_InitTypeDef	GPIO_InitStruct;
	USART_InitTypeDef	USART_InitStruct;
	NVIC_InitTypeDef	NVIC_InitStruct;

	//1. Configuare GPIO & AF
	RCC_AHB1PeriphClockCmd(USART_GPIO_RCC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStruct.GPIO_Pin = USART_PIN_TX;
	GPIO_Init(USART_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = USART_PIN_RX;
	GPIO_Init(USART_PORT, &GPIO_InitStruct);

	GPIO_PinAFConfig(USART_PORT, USART_PINSOURCE_TX, USART_AF);
	GPIO_PinAFConfig(USART_PORT, USART_PINSOURCE_RX, USART_AF);
	//2. Configuare USART2: Bus APB1
	RCC_APB1PeriphClockCmd(USART_RCC, ENABLE);

	USART_InitStruct.USART_BaudRate = USART_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;

	USART_Init(USARTX,&USART_InitStruct);
	//3. Enable USART2 Receive and Transmit intterupt
	USART_ITConfig(USARTX, USART_IT_RXNE, ENABLE);

	//4. Configuare NVIC
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	//5. ENABLE USART2
	USART_Cmd(USARTX, ENABLE);
}
/**
 * @func   serialUartInit
 * @brief  Serial usart initialization
 * @param  None
 * @retval None
 */
static void serialUartInit(void)
{
	//1. Khoi tao bo dem chua du lieu gui ve
	bufInit(pByBuffDataRx, &pUartRxQueue, sizeof(pByBuffDataRx[0]), SIZE_BUFF_DATA_RX);

	//2. Khoi tao ket noi USART2
	usart2Init();

	//3. Khoi tao trang thai cua nhan du lieu
	g_byRxState = (uint8_t)RX_STATE_START_BYTE;

}
/**
 * @func   serialUartInit
 * @brief  Serial usart initialization
 * @param  None
 * @retval None
 */
void USART2x_IRQHandler(void)
{
	uint8_t data;
	if(USART_GetITStatus(USARTX, USART_IT_RXNE) == SET)
	{
		data = USART_ReceiveData(USARTX);
		bufEnDat(&pUartRxQueue,&data);
		USART_ClearITPendingBit(USARTX, USART_IT_RXNE);
	}

}
/**
 * @func   serialUartInit
 * @brief  Serial usart initialization
 * @param  None
 * @retval None
 */
static void processSerialUartReceiver(void)
{
	static uint8_t uartState;
	uartState = pollRxBuff();

	if(uartState != UART_STATE_IDLE)
	{
		switch(uartState)
		{

		case UART_STATE_ACK_RECEIVED:
			break;
		case UART_STATE_NACK_RECEIVED:
			break;
		case UART_STATE_DATA_RECEIVED:
		{
        	procUartCmd((void *)&g_byRxDataByte[2]);
			break;
		}
		case UART_STATE_ERROR:
			break;
		default:
			break;
		}
	}
}
/**
 * @func   pollRxBuff
 * @brief  Serial usart initialization
 * @param  None
 * @retval UartState, Data recerver: g_byRxDataByte[]
 */
static uint8_t pollRxBuff(void)
{
	//1. Khai bao bien chua Data du lieu lay tu hang doi
	uint8_t RxDataTemp;
	uint8_t UartState = (uint8_t)UART_STATE_IDLE;

	//2. Tao vong lap den khi het du lieu trong hang doi
	while((bufNumItems(&pUartRxQueue) !=0) && (UartState == UART_STATE_IDLE)){
		//2.0 Lay 1 phan tu trong hang doi
		bufDeDat(&pUartRxQueue, &RxDataTemp);
		//USART_SendData(USARTX, RxDataTemp);
		switch(g_byRxState)
		{
		//2.1 Uart state- Start byte: 0xb1
		case RX_STATE_START_BYTE:
		{
			//2.1.1 TH: byte du lieu lay ra la byte start 0xb1
			if(RxDataTemp == START_BYTE)
			{

				/*
				 * Neu la byte start thi se khoi tao
				 * 1. Bien check xor = 0xff
				 * 2. Khoi tao so byte data doc duoc
				 * 3. Khoi tao g_byRxState sang trang thai bat dau doc du lieu
				 */
				g_byRxCheckXor = 0xFF;
				g_byRxIndexByte = 0;
				g_byRxState = RX_STATE_DATA_BYTES;
			}
			//2.1.2 TH: byte du lieu lay ra la byte ack 0x06
			else if(RxDataTemp == ACK_BYTE)
			{
				UartState = UART_STATE_ACK_RECEIVED;
			}
			//2.1.3	TH: byte du lieu lay ra la bit nack 0x15
			else if(RxDataTemp == NACK_BYTE)
			{
				UartState = UART_STATE_NACK_RECEIVED;
			}
			//2.1.4 TH: con lai
			else
			{
				UartState = UART_STATE_ERROR;
			}
			break;
		}
		//2.2 Uart state- Data byte
		case RX_STATE_DATA_BYTES:
		{
			/*
			 * Trong qua trinh doc du lieu:
			 * 1. Luu du lieu vao  mang de xu ly tiep
			 * 2. Tinh toan check Xor de xac nhan toan ven cua data da nhan duoc
			 * 3. Neu so byte doc duoc lon hon gia tri ma mang co the luu thi reset va bao loi
			 */
			if(g_byRxIndexByte < RX_MAX_INDEX_BYTE)
			{
				g_byRxDataByte[g_byRxIndexByte] = RxDataTemp;
				if(g_byRxIndexByte >0)
				{
					g_byRxCheckXor ^= RxDataTemp;
				}
				// Neu la byte cuoi cung thi se chuyen sang check xor
				// Sau byte start la byte chua do dai du lieu
				if(++g_byRxIndexByte == (*g_byRxDataByte))
				{
					g_byRxState = RX_STATE_CXOR_BYTE;
				}
			}else
			{
				g_byRxState = RX_STATE_START_BYTE;
				UartState = UART_STATE_ERROR;
			}
			break;
		}
		//2.3 Uart state- XOR byte
		case RX_STATE_CXOR_BYTE:
			/*
			 * Neu byte check xor gui toi bang gia tri tinh duoc thi data duoc chap nhan
			 * Nguoc lai thi bao loi
			 */
			//USART_SendData(USARTX, RxDataTemp);
			//USART_SendData(USARTX, g_byRxCheckXor);
			if(RxDataTemp == g_byRxCheckXor) {
				UartState = UART_STATE_DATA_RECEIVED;

			} else {
				UartState = UART_STATE_ERROR;
			}

		//2.1 Default
		default:
			g_byRxState = (uint8_t) RX_STATE_START_BYTE;
			break;
		}

	}
	return UartState;
}

/**
 * @func   LedCmdSetState
 * @brief  Dieu khien trang thai cua led khi dieu khien tu PC.
 * @param  led_id: ID led, led_color: Mau cua led, led_num_blink: so lan bat/tat,
 * led_interval: chu ky bat/tat,led_last_state: trang thai cuoi cua led
 * @retval None
 */
void ledCmdSetState(uint8_t btLedId, \
					 uint8_t byLedColor, \
					 uint8_t byLedNumBlink, \
					 uint8_t byLedInterval, \
					 uint8_t byLedLastState)
{
	BuzzerControl_SendPacketRespond(1);
	BuzzerControl_SetMelody(pbeep);
	if((byLedLastState == 0x00)||(byLedLastState == 0x01)||(byLedLastState == 0x02))
	{
		LedControl_SetColorGeneral(btLedId, byLedColor, 100);
	}else
	{
		LedControl_SetColorGeneral(btLedId, byLedColor, 0);
	}
}
/**
 * @func   buzzerCmdSetState
 * @brief  Dieu khien trang thai cua buzzer khi dieu khien tu PC
 * @param  n: So lan bat/tat buzzer
 * @retval None
 */
void buzzerCmdSetState(uint8_t byNtime)
{
	for(uint8_t i = 0;i<byNtime;i++)
	{
		BuzzerControl_SetMelody(pbeep);
	}
}
/**
 * @func   lcdCmdSetState
 * @brief  Dieu khien trang thai cua loa khi dieu khien tu PC
 * @param  *text : doan text duoc nhap tu PC-simulator
 * @retval None
 */
void lcdCmdSetState(char *text)
{
	ucg_ClearScreen(&ucg);
	g_byState = 0;
	ucg_DrawString(&ucg, 0, 12, 0, text);
}
void buttonCmdState(uint8_t byButtonId, uint8_t byButtonState)
{
	deviceStateMachine(byButtonId);

}

/**
 * @func   LightSensor_pollingRead
 * @brief  Scan light sensor
 * @param  None
 * @retval Gia tri cua light sensor khi chuyen doi xong
 */
uint16_t LightSensor_pollingRead(void)
{
	uint16_t result = 0;
	ADC_SoftwareStartConv(ADCx_SENSOR);
	while(ADC_GetFlagStatus(ADCx_SENSOR, ADC_FLAG_EOC));
	result = ADC_GetConversionValue(ADCx_SENSOR);
	return result;
}
/**
 * @func   taskMultiSensorScan
 * @brief  Task scan multiple sensor
 * @param  None
 * @retval None
 */
void taskMultiSensorScan(void *arg)
{
	// Xoa man hinh lan dau tu khi goi ham
	if (g_byState == 0 )
	    {
	        ucg_ClearScreen(&ucg);
	        g_byState = 1;
	    }
	g_dwTimeCurrent = GetMilSecTick(); // lay thoi gian tai thoi diem quet sensor lam moc

	if(g_dwTimeCurrent >= g_dwTimeInitial)
	{
		g_dwTimeTotal += g_dwTimeCurrent - g_dwTimeInitial;
	}
	else
	{
		g_dwTimeTotal += 0xFFFFFFFFU - g_dwTimeCurrent + g_dwTimeInitial;
	}
	if(g_dwTimeTotal >= CYCLE_SENT_DATA)
	{
		g_dwTimeTotal = 0;
		//Quet gia tri do lon cua anh sang, nhiet do, do am
		g_wLight = (uint16_t)(LightSensor_pollingRead());
		g_Temperature = (uint8_t)(TemHumSensor_GetTemp()/100);
		g_byHumidity = (uint8_t)(TemHumSensor_GetHumi()/100);
		//Display output

		ucg_SetFont(&ucg, ucg_font_ncenR10_hf);

		memset(srcTemp,0,sizeof(srcTemp));
		sprintf(srcTemp,"Temp = %d%d oC",(g_Temperature/10),(g_Temperature%10));
		ucg_DrawString(&ucg, 8, 50, 0, srcTemp);

		memset(srcHumi,0,sizeof(srcHumi));
		sprintf(srcHumi,"Humi = %d%d %%",(g_byHumidity/10),(g_byHumidity%10));
		ucg_DrawString(&ucg, 8, 75, 0, srcHumi);

		memset(srcLight,0,sizeof(srcLight));
		sprintf(srcLight,"Light = %d%d%d%d lux",(g_wLight/1000),(g_wLight/100)%10,(g_wLight/10)%10,(g_wLight%10));
		ucg_DrawString(&ucg, 8, 100, 0, srcLight);
		//Update Temp,Humi,Light to PC Simulator
		HumiSensor_SendPacketRespond(g_byHumidity);
		TempSensor_SendPacketRespond(g_Temperature);
		LightSensor_SendPacketRespond(g_wLight);


	}
	g_dwTimeInitial = g_dwTimeCurrent;
}
/**
 * @func   LoadConfiguration
 * @brief  Event start up of application
 * @param  None
 * @retval None
 */
void LoadConfiguration(void)
{
	ucg_SetFont(&ucg, ucg_font_ncenR10_hf);
	ucg_DrawString(&ucg, 42,30, 0, "IOT");
	ucg_DrawString(&ucg, 6, 50, 0, "Programming by");
	ucg_DrawString(&ucg, 4, 75, 0, "Lumi Smarthome");
}

/**
 * @func   appStateManager
 * @brief  Manager state application
 * @param  event: Su kien
 * @retval None
 */
static void appStateManager(event_api_e event)
{
	switch(getStateApp())
	{
	case STATE_APP_STARTUP: //Su kien khi he thong bat dau duoc cap nguon
		if( event == EVENT_APP_INIT )
		{
			LoadConfiguration();
			setStateApp(STATE_APP_IDLE);
		}
		break;
	case STATE_APP_IDLE:
			deviceStateMachine(event);
		break;
	case STATE_APP_RESET:
		break;
	default:
		break;

	}
}
/**
 * @func   setStateApp
 * @brief  Set state of application
 * @param  state: State of application
 * @retval None
 */

static void setStateApp(state_app_e state)
{
	eCurrentState = state;
}

/**
 * @func   getStateApp
 * @brief  Get state of application
 * @param  None
 * @retval State of application
 */
static state_app_e getStateApp(void)
{
	return eCurrentState;
}

/**
 * @func   deviceStateMachine
 * @brief  State machine of the device
 * @param  event
 * @retval None
 */

void deviceStateMachine(event_button_t event)
{
	static uint8_t state_Led =0x00 ;
	static uint8_t flag = 0x00;
	static uint8_t str[][20] ={
			{"Device: Board STM32\n"},
			{"Nucleo."},
			{"Code: STM32F401RE"},
			{"NUCLEO."},
			{"Manufacturer:"},
			{"STMicroelectronics."},
			{"Kit expansion:"},
			{"Lumi Smarthome."}

	};
	switch(event)
	{
		case EVENT_OF_BUTTON_0_PRESS_5_TIMES:
		{
			//Nhay 5 lan mau green voi do sang 50%
			LedControl_BlinkStart(LED_ALL_ID, BLINK_GREEN, 10, 500, LED_COLOR_BLACK);

			//In ra man hinh dong chu tren lcd
            ucg_SetFont(&ucg, ucg_font_ncenR08_tr);
            ucg_ClearScreen(&ucg);

            ucg_DrawString(&ucg, 0, 12, 0, "Device: Board STM32");
            ucg_DrawString(&ucg, 0, 26, 0, "Nucleo.");
            ucg_DrawString(&ucg, 0, 39, 0, "Code: STM32F401RE");
            ucg_DrawString(&ucg, 0, 54, 0, "NUCLEO.");
            ucg_DrawString(&ucg, 0, 68, 0, "Manufacturer:");
            ucg_DrawString(&ucg, 0, 84, 0, "STMicroelectronics.");
            ucg_DrawString(&ucg, 0, 98, 0, "Kit expansion:");
            ucg_DrawString(&ucg, 0, 112, 0, "Lumi Smarthome.");


            //Goi chuong trinh quet sensor(taskMultiSensorScan) sau 5s
            if (idTimerSensorScan != NO_TIMER)
			{
				g_byState = 0;
				TimerStop(idTimerSensorScan);
			}

			idTimerSensorScan = TimerStart("taskMultiSensorScan", \
											PERIOD_SCAN_MULTISENSOR, \
											TIMER_REPEAT_FOREVER, \
											taskMultiSensorScan, \
											NULL);
			for(int a = 0;a<8;a++)
			{
				Serial_SendPacket(0, CMD_ID_LCD, CMD_TYPE_RES, str[a], sizeof(str[a]));
			}

		} break;
		case EVENT_OF_BUTTON_0_PRESS_LOGIC: //Nut Board
		{

		} break;
		case EVENT_OF_BUTTON_1_PRESS_LOGIC: //Nut B1
		{
			// dao trang thai cua bit 0,la bit dai dien cho trang thai cua led do
			state_Led ^= 0x01;
			BuzzerControl_SetMelody(pbeep);
			BuzzerControl_SendPacketRespond(1);


			if(state_Led & 0x01)
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_RED, 50);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_RED, 50);

				// giu trang thai cua led do, dua trang thai cua cac led con lai ve 0
				state_Led &= 0x01;
			}else
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_RED, 0);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_RED, 0);

			}

		} break;
		case EVENT_OF_BUTTON_2_PRESS_LOGIC:	//Nut B2
		{
			// dao trang thai cua bit 1,la bit dai dien cho trang thai cua led xanh la
			state_Led ^= (0x01<<1);
			BuzzerControl_SetMelody(pbeep);
			BuzzerControl_SendPacketRespond(1);
			if(state_Led & (0x01<<1))
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_GREEN, 50);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_GREEN, 50);
				// giu trang thai cua led xanh la, dua trang thai cua cac led con lai ve 0
				state_Led &= (0x01<<1);
				flag = 0;
			}else
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_GREEN, 0);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_GREEN, 0);
			}
		} break;
		case EVENT_OF_BUTTON_5_PRESS_LOGIC: //Nut B5
		{
			// dao trang thai cua bit 2,la bit dai dien cho trang thai cua led xanh duong
			state_Led ^= (0x01<<2);
			BuzzerControl_SetMelody(pbeep);
			BuzzerControl_SendPacketRespond(1);
			if(state_Led & (0x01<<2))
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_BLUE, 50);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_BLUE, 50);
				// giu trang thai cua led xanh duong, dua trang thai cua cac led con lai ve 0
				state_Led &= (0x01<<2);
			}else
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_BLUE, 0);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_BLUE, 0);
			}
		} break;
		case EVENT_OF_BUTTON_4_PRESS_LOGIC: ////Nut B4
		{
			// dao trang thai cua bit 3,la bit dai dien cho trang thai cua led trang
			state_Led ^= (0x01<<3);
			BuzzerControl_SetMelody(pbeep);
			BuzzerControl_SendPacketRespond(1);
			if(state_Led & (0x01<<3))
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_WHITE, 50);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_WHITE, 50);
				// giu trang thai cua led trang, dua trang thai cua cac led con lai ve 0
				state_Led &= (0x01<<3);
			}else
			{
				LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_WHITE, 0);
				LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_WHITE, 0);
			}
		} break;


		case EVENT_OF_BUTTON_1_HOLD_1S:
		{
			//Tang do sang cho led theo thoi gian khi van giu nut B1
			setLevelLed(LED_LEVEL_UP);
			state_Led |= (0x01<<1);
		} break;

		case EVENT_OF_BUTTON_5_HOLD_1S:
		{
			//Giam do sang cho led theo thoi gian khi van giu nut B5
			setLevelLed(LED_LEVEL_DOWN);
			state_Led |= (0x01<<1);
		} break;
		case EVENT_OF_BUTTON_0_RELEASED_1S:
		case EVENT_OF_BUTTON_1_RELEASED_1S:
		case EVENT_OF_BUTTON_2_RELEASED_1S:
		case EVENT_OF_BUTTON_3_RELEASED_1S:
		case EVENT_OF_BUTTON_4_RELEASED_1S:
		case EVENT_OF_BUTTON_5_RELEASED_1S:
		{
			//Khi tha nut B1 va B5 se cho dung chuong trinh tang hoac giam
			LedControl_SendPacketRespond(LED_KIT_ID0,LED_COLOR_GREEN,g_byPwmLedLevel);
			LedControl_SendPacketRespond(LED_KIT_ID1,LED_COLOR_GREEN,g_byPwmLedLevel);
			state_Led &= (0x01<<1);
			flag = 1;
			stopLevelChange();
		} break;
		default:
			break;
	}
	// Gui trang thai cua led len PC-simulator
	if((state_Led & 0x01)== 1)
	{
		LedControl_SendPacketRespond(LED_KIT_ID1, LED_COLOR_RED, 50);
		LedControl_SendPacketRespond(LED_KIT_ID0, LED_COLOR_RED, 50);

	}else if((state_Led & (0x01<<1)) == 2)
	{
		LedControl_SendPacketRespond(LED_KIT_ID1, LED_COLOR_GREEN, 50);
		LedControl_SendPacketRespond(LED_KIT_ID0, LED_COLOR_GREEN, 50);

	}else if((state_Led & (0x01<<2)) == 4)
	{
		LedControl_SendPacketRespond(LED_KIT_ID1, LED_COLOR_BLUE, 50);
		LedControl_SendPacketRespond(LED_KIT_ID0, LED_COLOR_BLUE, 50);
	}else if((state_Led & (0x01<<3)) == 8)
	{
		LedControl_SendPacketRespond(LED_KIT_ID1, LED_COLOR_WHITE, 50);
		LedControl_SendPacketRespond(LED_KIT_ID0, LED_COLOR_WHITE, 50);
	}else
	{
		LedControl_SendPacketRespond(LED_KIT_ID0, LED_COLOR_RED, 0);
		LedControl_SendPacketRespond(LED_KIT_ID1, LED_COLOR_RED, 0);
		LedControl_SendPacketRespond(LED_KIT_ID0, LED_COLOR_BLUE, 0);
		LedControl_SendPacketRespond(LED_KIT_ID1, LED_COLOR_BLUE, 0);
		if(flag == 0)
		{
			LedControl_SendPacketRespond(LED_KIT_ID0, LED_COLOR_GREEN, 0);
			LedControl_SendPacketRespond(LED_KIT_ID1, LED_COLOR_GREEN, 0);
		}
	}
}
/**
 * @func   SetLevelUp
 * @brief  Tac dung tang do sang cua led khi duoc goi toi
 * @param
 * @retval None
 */
void SetLevelUp(void*arg)
{
	g_byPwmLedLevel ++;
	if(g_byPwmLedLevel == 100)
	{
		stopLevelChange();
		return;
	}

	LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_GREEN, g_byPwmLedLevel);
	LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_GREEN, g_byPwmLedLevel);
}
/**
 * @func   SetLevelUp
 * @brief  Tac dung giam do sang cua led khi duoc goi toi
 * @param
 * @retval None
 */
void SetLevelDown(void*arg)
{
	g_byPwmLedLevel --;
	if(g_byPwmLedLevel == 0)
	{
		stopLevelChange();
		return;
	}

	LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_GREEN, g_byPwmLedLevel);
	LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_GREEN, g_byPwmLedLevel);
}
/**
 * @func   setLevelLed
 * @brief  Dung time de tang/giam do sang tu tu
 * @param
 * @retval None
 */
void setLevelLed(uint8_t byLevel)
{
	if(byLevel == LED_LEVEL_UP)
	{
		if (g_byIdTimerSetLevelLed != NO_TIMER)
		{
			g_byState = 0;
			TimerStop(g_byIdTimerSetLevelLed);
		}

		g_byIdTimerSetLevelLed = TimerStart("SetLevelUp", TIME_STEP_LEVEL, TIMER_REPEAT_FOREVER, SetLevelUp, NULL);
	}else if(byLevel == LED_LEVEL_DOWN)
	{
		if (g_byIdTimerSetLevelLed != NO_TIMER)
		{
			g_byState = 0;
			TimerStop(g_byIdTimerSetLevelLed);
		}
		g_byIdTimerSetLevelLed = TimerStart("SetLevelDown", TIME_STEP_LEVEL, TIMER_REPEAT_FOREVER, SetLevelDown, NULL);
	}
}

void stopLevelChange(void)
{
	if(g_byIdTimerSetLevelLed !=NO_TIMER)
	{
		TimerStop(g_byIdTimerSetLevelLed);
		g_byIdTimerSetLevelLed = NO_TIMER;
	}
}

/**
 * @func   processUartReceiveCommand_ControlLed
 * @brief  Process command led
 * @param  pCmd
 * @retval None
 */
/*
 * (uint8_t led_id, \
 uint8_t led_color, \
 uint8_t led_num_blink, \
 uint8_t led_interval, \
 uint8_t led_last_state);
 */
static void
processUartReceiveCommand_ControlLed(
    cmd_receive_p pCmd
) {

    switch (pCmd->cmdCommon.type) {
        case CMD_TYPE_SET:
        	// write code here
        	ledCmdSetState(pCmd->ledIndicator.numID,
        			pCmd->ledIndicator.color,
					pCmd->ledIndicator.counter,
					pCmd->ledIndicator.interval,
					pCmd->ledIndicator.laststate);
            break;

        default:
            /* Respond frame NACK */
            SendNACK();
            break;
    }
}
static void
processUartReceiveCommand_ControlButton(
    cmd_receive_p pCmd
)
{
    switch (pCmd->cmdCommon.type) {
        case CMD_TYPE_SET:
        	// write code here
        	buttonCmdState(pCmd->buttonState.epoint, pCmd->buttonState.state);
            break;

        default:
            /* Respond frame NACK */
            SendNACK();
            break;
    }
}
static void
processUartReceiveCommand_ControlLCD(
    cmd_receive_p pCmd
)
{
	static char BuffText[20];
	for(int i = 0; i<20;i++)
	{
		BuffText[i] = pCmd->lcdDisplay.text[i];
	}
	lcdCmdSetState(BuffText);
}
static void
processUartReceiveCommand_ControlBuzzer(
    cmd_receive_p pCmd
)
{
    switch (pCmd->cmdCommon.type) {
        case CMD_TYPE_SET:
        	// write code here
        	buzzerCmdSetState(pCmd->buzzerState.state);
            break;

        default:
            /* Respond frame NACK */
            SendNACK();
            break;
    }

}
/**
 * @func   procUartCmd
 * @brief  Process command uart
 * @param  None
 * @retval None
 */
static void
procUartCmd(
    void *arg
) {
	cmd_receive_p pCmd = (cmd_receive_p)arg;

    switch (pCmd->cmdCommon.cmdid) {
		case CMD_ID_LED:
			processUartReceiveCommand_ControlLed(pCmd);
			break;

		case CMD_ID_BUZZER:
			processUartReceiveCommand_ControlBuzzer(pCmd);
			break;

		case CMD_ID_BUTTON:
			processUartReceiveCommand_ControlButton(pCmd);
			break;

		case CMD_ID_LCD:
			processUartReceiveCommand_ControlLCD(pCmd);
			break;

        default:
            /* Respond frame NACK */
            SendNACK();
            break;
    }
}
