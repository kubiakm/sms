/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
# define LCD_COLOR_BLUE (( uint32_t ) 0 xFF0000FF )
# define LCD_COLOR_GREEN (( uint32_t ) 0 xFF00FF00 )
# define LCD_COLOR_RED (( uint32_t ) 0 xFFFF0000 )
# define LCD_COLOR_CYAN (( uint32_t ) 0 xFF00FFFF )
# define LCD_COLOR_MAGENTA (( uint32_t ) 0 xFFFF00FF )
# define LCD_COLOR_YELLOW (( uint32_t ) 0 xFFFFFF00 )
# define LCD_COLOR_LIGHTBLUE (( uint32_t ) 0 xFF8080FF )
# define LCD_COLOR_LIGHTGREEN (( uint32_t ) 0 xFF80FF80 )
# define LCD_COLOR_LIGHTRED (( uint32_t ) 0 xFFFF8080 )
# define LCD_COLOR_LIGHTCYAN (( uint32_t ) 0 xFF80FFFF )
# define LCD_COLOR_LIGHTMAGENTA (( uint32_t ) 0 xFFFF80FF )
# define LCD_COLOR_LIGHTYELLOW (( uint32_t ) 0 xFFFFFF80 )
# define LCD_COLOR_DARKBLUE (( uint32_t ) 0 xFF000080 )
# define LCD_COLOR_DARKGREEN (( uint32_t ) 0 xFF008000 )
# define LCD_COLOR_DARKRED (( uint32_t ) 0 xFF800000 )
# define LCD_COLOR_DARKCYAN (( uint32_t ) 0 xFF008080 )
# define LCD_COLOR_DARKMAGENTA (( uint32_t ) 0 xFF800080 )
# define LCD_COLOR_DARKYELLOW (( uint32_t ) 0 xFF808000 )
# define LCD_COLOR_WHITE (( uint32_t ) 0 xFFFFFFFF )
# define LCD_COLOR_LIGHTGRAY (( uint32_t ) 0 xFFD3D3D3 )
# define LCD_COLOR_GRAY (( uint32_t ) 0 xFF808080 )
# define LCD_COLOR_DARKGRAY (( uint32_t ) 0 xFF404040 )
# define LCD_COLOR_BLACK (( uint32_t ) 0 xFF000000 )
# define LCD_COLOR_BROWN (( uint32_t ) 0 xFFA52A2A )
# define LCD_COLOR_ORANGE (( uint32_t ) 0 xFFFFA500 )
# define LCD_COLOR_TRANSPARENT (( uint32_t ) 0 xFF000000 )
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

#include "modbus_conf.h"
#include "modbus.h"

/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart;
UART_HandleTypeDef huartmb;

/* Private variables ---------------------------------------------------------*/
uint8_t *resp;
uint16_t resplen;
MB_RESPONSE_STATE respstate;
uint8_t fan_on[] =     {0x00, 0x01, 0x03, 0xE8};
uint8_t fan_half[] =   {0x00, 0x00, 0x01, 0xF4};
uint8_t fan_off[] =    {0x00, 0x01, 0x00, 0x00};
uint8_t heater_on[] =  {0x00, 0x04, 0x03, 0xE8};
uint8_t heater_half[] ={0x00, 0x04, 0x01, 0xF4};
uint8_t heater_var[] = {0x00, 0x04, 0x01, 0xF4};
uint8_t heater_off[] = {0x00, 0x04, 0x00, 0x00};
uint8_t get_temp[] = {0x00, 0x00, 0x00, 0x01};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void LCD_Config(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART_PC_Init(void);
static void MX_UART_MB_Init(void);

/* Private function prototypes -----------------------------------------------*/
__IO uint32_t input = 0;
__IO uint32_t output = 0;

uint8_t UART_MB_rcvd = 0;
__IO uint8_t UART_MB_sending = 0;

uint32_t MODBUS_SLAVE_ADDRESS = 12;


char txt[200] = {0};
uint32_t temp = 0;
uint32_t ts_counter = 0;
uint32_t user_counter = 0;

void Communication_Mode(bool rx, bool tx){
	if(rx) HAL_UART_Receive_IT(&huartmb, &UART_MB_rcvd, 1);
	
	if(tx && UART_MB_sending == 0) {
		UART_MB_sending = 1;
		SetCharacterReadyToTransmit();
	} 
	if(!tx) UART_MB_sending = 0;
}
void Communication_Put(uint8_t ch){
	HAL_UART_Transmit_IT(&huartmb, &ch, 1);
}

uint8_t Communication_Get(void){
	uint8_t tmp = UART_MB_rcvd;
	UART_MB_rcvd = 0;
	SetCharacterReceived(false);
	return tmp;
}

void Enable50usTimer(void){
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Disable50usTimer(void){
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
}

//flags
int flag=0;
int flag_auto=1;
int flag_hot=0;
int flag_communication=1;
int flag_czujnik=1;
int flag_previous_auto=1,flag_previous_hot=0,flag_previous_communication=1,flag_previous_czujnik=1;
float u = 0.0f;
float u_zad = 10.0f;
int i=0;
float temp_previous=0;


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_LTDC_Init();
		
  LCD_Config(); 
	MX_UART_PC_Init();
	MX_UART_MB_Init();
	
	BSP_LED_Init(LED1);
	BSP_LED_On(LED1);
	
	BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);  
	BSP_TS_Init(0,0);
	BSP_TS_ITConfig();
	MB_Config(115200);
		
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 1);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 1);
	
	while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX);
	while(HAL_UART_GetState(&huartmb) == HAL_UART_STATE_BUSY_TX);
	
	MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();	
		
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_LTDC_ProgramLineEvent(&hltdc, 272);
	
	HAL_Delay(100);
	MODBUS_SLAVE_ADDRESS = 11;
	MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, fan_half, 4);
	respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
	if(respstate == RESPONSE_OK) ;
	
	/* setting the fan W1 on 50% of its power */
	
	/*for( MODBUS_SLAVE_ADDRESS = 10; MODBUS_SLAVE_ADDRESS < 26; ++MODBUS_SLAVE_ADDRESS){;
			fan_on[1] = 0;
			MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, fan_on, 4);
			respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
			if(respstate == RESPONSE_OK) break;
			HAL_Delay(500);		
	}
	HAL_Delay(500);		
	for(int i = 0; i < 6; ++i){
		fan_on[1] = i;
		MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, fan_off, 4);
		respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
		if(respstate != RESPONSE_OK) while(1);
		HAL_Delay(500);
	}
	
	for(int i = 0; i < 6; ++i){
		fan_off[1] = i;
		MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, fan_off, 4);
		respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
		if(respstate != RESPONSE_OK) while(1);
		HAL_Delay(500);
	}
	*/
	HAL_Delay(1000);
	
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
  while (1){			
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	if(huart->Instance == USART6){
		SetCharacterReceived(true);
		HAL_UART_Receive_IT(&huartmb, &UART_MB_rcvd, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
		UART_MB_sending = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == TS_INT_PIN){
		++ts_counter;
	} else if(GPIO_Pin == TAMPER_BUTTON_PIN){
		++user_counter;
	}
	// Wywolywane w chwili wcisniecia przycisku User
}

float u_previous=0.0f;
float temp_zadana=40.0f;
float e=0.0f;

#define D 394

float lambda = 1.0f;      //parametr lambda
			//horyzont dynamiki
int Nu= 70; 	//horyzont sterowania
int N = 390; 	//horyzont predykcji
float y_zad=5;
float u_prev1=2;
float deltaU =0.0f;
float deltaUP[D-1]={0};
float s[30];
float Ku[D-1]={0.073740, 0.076301, 0.078872, 0.081433, 0.083983, 0.086538, 0.089080, 0.091625, 0.094156, 0.095000, 0.097509,
        0.099999, 0.101036, 0.102054, 0.103015, 0.103718, 0.104633, 0.105534, 0.107852, 0.107024, 0.107848, 0.108665, 0.109449, 0.108517,
        0.109239, 0.108241, 0.107451, 0.106393, 0.105526, 0.106065, 0.104893, 0.103927, 0.105828, 0.104572, 0.103525, 0.102209, 0.101099,
        0.099735, 0.098574, 0.097138, 0.095923, 0.094439, 0.093156, 0.091613, 0.090279, 0.088682, 0.090190, 0.088780, 0.087120, 0.085663,
        0.083948, 0.082453, 0.080689, 0.079150, 0.077353, 0.075786, 0.075394, 0.073776, 0.071908, 0.071718, 0.072947, 0.071278, 0.070803,
        0.069106, 0.067157, 0.065437, 0.063461, 0.061705, 0.061396, 0.059381, 0.057576, 0.056963, 0.055125, 0.054703, 0.055714, 0.055020,
        0.053101, 0.052620, 0.051876, 0.051369, 0.050839, 0.050300, 0.049500, 0.048937, 0.048358, 0.047756, 0.046894, 0.046265, 0.047058,
        0.044951, 0.044040, 0.044806, 0.044111, 0.043412, 0.042705, 0.041737, 0.040992, 0.041692, 0.040923, 0.040142, 0.039096, 0.038280,
        0.038888, 0.038048, 0.037197, 0.036093, 0.036666, 0.035789, 0.034902, 0.034006, 0.032864, 0.031951, 0.031034, 0.030104, 0.028920,
        0.027982, 0.028468, 0.028958, 0.027999, 0.027036, 0.027520, 0.026311, 0.026787, 0.025809, 0.026273, 0.025284, 0.025739, 0.024739,
        0.023504, 0.023951, 0.022954, 0.023402, 0.023859, 0.022864, 0.023317, 0.022324, 0.021070, 0.020059, 0.019055, 0.018031, 0.018455,
        0.017190, 0.017613, 0.016582, 0.016999, 0.017399, 0.017806, 0.018208, 0.018606, 0.017534, 0.017907, 0.018267, 0.018622, 0.018969,
        0.017870, 0.018215, 0.018561, 0.018898, 0.017541, 0.017870, 0.018205, 0.018532, 0.017402, 0.017710, 0.016584, 0.016884, 0.015745,
        0.016041, 0.016331, 0.016617, 0.015216, 0.015500, 0.015774, 0.016039, 0.014854, 0.015114, 0.015364, 0.014160, 0.014404, 0.014650,
        0.013441, 0.013674, 0.013912, 0.014141, 0.012684, 0.012909, 0.013132, 0.013348, 0.012117, 0.012331, 0.011101, 0.011305, 0.011515,
        0.010288, 0.010505, 0.010724, 0.010939, 0.011150, 0.011365, 0.011581, 0.010106, 0.010314, 0.010526, 0.009287, 0.009495, 0.009700,
        0.008461, 0.008668, 0.008874, 0.009071, 0.007827, 0.008017, 0.008210, 0.008407, 0.006901, 0.007085, 0.007265, 0.007439, 0.007607,
        0.007767, 0.007930, 0.008083, 0.008232, 0.008378, 0.008520, 0.008651, 0.008789, 0.008923, 0.009056, 0.009190, 0.009317, 0.009442,
        0.009569, 0.009692, 0.008370, 0.008493, 0.008615, 0.007290, 0.007412, 0.006083, 0.006209, 0.006332, 0.006458, 0.006582, 0.005020,
        0.005146, 0.005280, 0.005405, 0.005539, 0.005669, 0.005799, 0.005936, 0.004626, 0.004759, 0.004900, 0.005041, 0.005182, 0.005325,
        0.005467, 0.005615, 0.005765, 0.004464, 0.004617, 0.004765, 0.004924, 0.006533, 0.005256, 0.005425, 0.005602, 0.005779, 0.005956,
        0.006136, 0.004869, 0.005049, 0.005230, 0.005409, 0.005588, 0.005767, 0.005945, 0.006125, 0.006309, 0.006489, 0.006676, 0.006860,
        0.007040, 0.007215, 0.005703, 0.005875, 0.006047, 0.006214, 0.006383, 0.006549, 0.006711, 0.006870, 0.007029, 0.007186, 0.007340,
        0.007493, 0.007642, 0.007784, 0.007924, 0.008058, 0.008190, 0.008316, 0.008443, 0.008567, 0.007239, 0.007355, 0.007472, 0.007584,
        0.006251, 0.006360, 0.006462, 0.006561, 0.006657, 0.006752, 0.006841, 0.006924, 0.007009, 0.005642, 0.005718, 0.005795, 0.005867,
        0.005933, 0.005995, 0.006052, 0.004419, 0.004467, 0.004519, 0.004571, 0.004618, 0.004662, 0.004710, 0.004755, 0.004799, 0.004841,
        0.004882, 0.004922, 0.004959, 0.003550, 0.003588, 0.003623, 0.002217, 0.002258, 0.002299, 0.002340, 0.002380, 0.002421, 0.002461,
        0.002500, 0.001092, 0.001129, 0.001166, 0.001202, 0.001236, 0.001268, 0.001304, 0.001339, 0.001371, 0.001405, 0.001436, 0.001465,
        0.001491, 0.001515, 0.001538, 0.001557, 0.001576, 0.001594, 0.001610, 0.001623, 0.001637, 0.001649, 0.001660, 0.001669, 0.001675,
        0.001680, 0.001683, 0.001685, 0.001688, 0.001688, 0.001688, 0.001688, 0.001688, 0.001688, 0.001688, 0.001688, 0.001688, 0.001688,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000}; //tu wyniki z matlaba
float Ke=0.964615;
float DMC()
{
	
	float Ku_deltaUp=0.0f;
	
	int i=0, j=D;

	while(i<D-1){
        Ku_deltaUp += Ku[i]*deltaUP[i];
        i++;
	}
	deltaU = Ke*e - Ku_deltaUp;

    while(j>0){
        deltaUP[j]= deltaUP[j-1];
        j--;
    }
    deltaUP[0] = deltaU;

	return u_previous +deltaU;
}

void com_error_draw(){
		if(flag_communication==0){// && flag_previous_communication!=flag_communication){
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillRect(159,172,160,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font12); //ustawienie czcionki
		BSP_LCD_DisplayStringAt(159,172+12,(uint8_t*)"BLAD KOMUNIKACJI", LEFT_MODE);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		static uint16_t raw_y = 2345;
		static uint16_t raw_u = 0;
		static float y = 0.0f;
		
		MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_READ_INPUT_REGISTER, get_temp, 4);
		respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_READ_INPUT_REGISTER, &resp, &resplen, 1000);
			if(respstate != RESPONSE_OK) {	
				flag_communication=0; 
				com_error_draw();
			}
			//while(1);
		else {
			flag_communication=1;
			raw_y = resp[1]*0x100+resp[2];
			y = raw_y/100.0f;
			temp = raw_y;
		}
		if(temp>=5500.0f && temp<=12500.0f ) flag_hot=1; //przegrzanie
		else flag_hot=0;
		if(temp>=12500.0f || temp<=-5500.0f ) flag_czujnik=0;
		else flag_czujnik=1;
		e=temp_zadana-temp/100.0f;
		/*
		IF DO OPOZNIENIA
		if (i< 5 ){
			u=-40;
			i++;}
		else
		u=0;*/
		
		//HAL_Delay(2000);
		/* przyklady tego, jak nalezy interpretowac poszczegolne wartosci sterowania */
		//u = -10.0; // grzanie z moca (-10+50)% =  40%
		//u =   0.0; // grzanie z moca (  0+50)% =  50%
		//u =  50.0; // grzanie z moca ( 50+50)% = 100%
		
		
		//STEROWANIE
		if(flag_auto==1 && flag_czujnik==1) u=DMC();
		else if(flag_czujnik==1)u=u_zad;
		else u=-50.0f;
				
		/* aplikacja ograniczen na sygnal sterujacy */
		if(u >   50.0f) u =  50.0f;
		if(u <  -50.0f) u = -50.0f;
		
		u_previous=u;
		/* skalowanie z -50..50 do 0..1000 */
		raw_u = (uint16_t)(u+50.0f)*10; // przejscie z -2048 - 2047 do 0 - 4095
		
		/* przygotowanie wiadomosci MODBUS */
		heater_var[2] = (raw_u&0xFF00)>>8; // pierwszy bajt
		heater_var[3] = (raw_u&0x00FF)>>0; // drugi bajt
		
		/* wyslanie wiadomosci */
		MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, heater_var, 4);
		
		/* odczyt odpowiedzi i sprawdzenie jej poprawnosci */
		respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
		if(respstate != RESPONSE_OK) while(1);
		
		/* komunikacja z komputerem */
		while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX);
		sprintf(txt,"U=%+8.2f;Y=%+8.2f;",u,y);
		if(HAL_UART_Transmit_IT(&huart, (uint8_t*)txt, 22)!= HAL_OK){
			Error_Handler();   
		}	
	} 
	if (htim->Instance == TIM3){ // timer odpowiedzialny za aktualizacje MB i odliczanie timeout'u
		MB();
		TimeoutTick();
	}
	if (htim->Instance == TIM4){ // timer odpowiedzialny za odliczanie kwantow 50us
		Timer50usTick();
	}
	if (htim->Instance == TIM5){ // ...
	}
	//przypisywanie poprzednich stanow
				temp_previous=temp;
				flag_previous_hot=flag_hot;
				flag_previous_communication=flag_communication;
				flag_previous_auto=flag_auto;
				flag_previous_czujnik=flag_czujnik;
}

uint8_t touch_index = 0;
int now = 0;
TS_StateTypeDef TS_State;
int previous = 1;
//static int colorflag=0;
int x = 240;
int pasek=0;

void touch() {
	now = 0;
	int turn_flag_hot_off = 0;
	int turn_flag_communication_off = 0;
	int turn_flag_czujnik_off = 0;
	int turn_flag_auto_on = -1;
	int increase_temp=0;
	int increase_u=0;
	BSP_TS_GetState(&TS_State);
	for (touch_index = 0; touch_index < TS_State.touchDetected; ++touch_index)
	{
		//przegrzanie
		if (flag_hot == 1) {
			if (TS_State.touchX[touch_index] < 160 && TS_State.touchX[touch_index]>0 && TS_State.touchY[touch_index] < 222 && TS_State.touchY[touch_index]>172)
				now = 1;
			//turn_flag_hot_off = 1;
		}
		//odlaczona komunikacja
		if (flag_communication == 0) {
			if (TS_State.touchX[touch_index] < 319 && TS_State.touchX[touch_index]>159 && TS_State.touchY[touch_index] < 222 && TS_State.touchY[touch_index]>172)
				now = 1;
			turn_flag_communication_off = 1;
		}
		//odlaczony czujnik
		if (flag_czujnik == 0) {
			if (TS_State.touchX[touch_index] < 479 && TS_State.touchX[touch_index]>319 && TS_State.touchY[touch_index] < 222 && TS_State.touchY[touch_index]>172)
				now = 1;
			turn_flag_czujnik_off = 1;
		}
		//wlacz auto
		if (TS_State.touchX[touch_index] < 479 && TS_State.touchX[touch_index]>429 && TS_State.touchY[touch_index] < 50 && TS_State.touchY[touch_index]>0) {
			now = 1;
			turn_flag_auto_on = 1;
		}
		
			//wlacz reczny
		if (TS_State.touchX[touch_index] < 479 && TS_State.touchX[touch_index]>429 && TS_State.touchY[touch_index] < 99 && TS_State.touchY[touch_index]>49) {
			now = 1;
			turn_flag_auto_on =0;
		}
			//+ temp zadana
		if (TS_State.touchX[touch_index] < 379 && TS_State.touchX[touch_index]>329 && TS_State.touchY[touch_index] < 49 && TS_State.touchY[touch_index]>0) {
			now = 1;
			increase_temp = 1;
		}
		//-temp zadana
		if (TS_State.touchX[touch_index] < 429 && TS_State.touchX[touch_index]>379 && TS_State.touchY[touch_index] < 49 && TS_State.touchY[touch_index]>0) {
			now = 1;
			increase_temp = -1;
		}
		//GDY MAMY WLACZONY TRYB RECZNY
		if(flag_auto==0){
			//+sterowanie
			if (TS_State.touchX[touch_index] < 379 && TS_State.touchX[touch_index]>329 && TS_State.touchY[touch_index] < 99 && TS_State.touchY[touch_index]>49) {
				now = 1;
				increase_u = 1;
			}
			//-sterowanie
			if (TS_State.touchX[touch_index] < 429 && TS_State.touchX[touch_index]>379 && TS_State.touchY[touch_index] < 99 && TS_State.touchY[touch_index]>49) {
				now = 1;
				increase_u = -1;
			}
		}
		//if(TS_State.touchX[touch_index]<229 ){ now=1;}
		//else {x=240; BSP_LCD_FillRect(240,20,240,250);}
	}
	if (previous == 0 && now == 1) {
		if (turn_flag_hot_off == 1) flag_hot = 0;
		if (turn_flag_communication_off == 1) flag_communication = 1;
		if (turn_flag_czujnik_off == 1) flag_czujnik = 1;
		if (turn_flag_auto_on == 1) flag_auto = 1;
		else if (turn_flag_auto_on == 0) flag_auto = 0;
		if (increase_temp==1) temp_zadana=temp_zadana+5;
		else if (increase_temp==-1) temp_zadana=temp_zadana-5;
		if (increase_u==1) u_zad=u_zad+5;
		else if (increase_u==-1) u_zad=u_zad-5;
		
		//colorflag=!colorflag;
	}
	previous = now;
}
int Displayflag=0;
void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *hltdc){
	static char buf1[100] = {0};
	static char buf2[100] = {0};
	static char buf3[100] = {0};
	static char buf4[100] = {0};
	static char tempzadanabuf[100] = {0};
	static char u_buf[100] = {0};
	
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	//sprintf(buf1, "ADDRESS: %d", MODBUS_SLAVE_ADDRESS);
	sprintf(buf2, "temperatura: %.2f ", (float)(temp/100.0f));
	//sprintf(buf3, "pasek     : %d", pasek);
	//sprintf(buf3, "TS     : %d", ts_counter);
	//sprintf(buf4, "USER   : %d", user_counter);
	sprintf(buf1, "uchyb   : %.2f  ", e);
	sprintf(tempzadanabuf, "T zadana: %d   ",(int)temp_zadana);
	sprintf(u_buf, "Sterowanie: %d   ", (int)u);
	//BSP_LCD_DisplayStringAtLine(1,(uint8_t*)buf1);
	//BSP_LCD_DisplayStringAtLine(2,(uint8_t*)buf3);

	//wyswietlanie temp aktualnej zadanej i sterowania 
	BSP_LCD_SetFont(&Font20); //ustawienie czcionki
	BSP_LCD_DisplayStringAtLine(1,(uint8_t*)buf2);
	BSP_LCD_DisplayStringAtLine(2,(uint8_t*)tempzadanabuf);
	BSP_LCD_DisplayStringAtLine(3,(uint8_t*)u_buf);
	BSP_LCD_DisplayStringAtLine(4,(uint8_t*)buf1);
	BSP_LCD_DisplayStringAtLine(5,(uint8_t*)buf3);
	

	//czyszczenie pola o przegrzaniu
	if(flag_hot==0){ // && flag_previous_hot==1){
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(0,172,160,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	}
		//czyszczenie pola o komunikacji
	if(flag_communication==1){// && flag_previous_communication==0){
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(159,172,160,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	}
		//czyszczenie pola o czujniku
	if(flag_czujnik==1){// && flag_previous_czujnik==0){
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(319,172,160,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	}
	//wyswietlanie ostrzezenia o przegrzaniu
	if(flag_hot==1) {//&& flag_previous_hot!=flag_hot){
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillRect(0,172,160,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font16); //ustawienie czcionki
		BSP_LCD_DisplayStringAt(0,172+12,(uint8_t*)"PRZEGRZANIE", LEFT_MODE);
	}
	//wyswietlanie ostrzezenia o bledzie komunikacji
	if(flag_communication==0){// && flag_previous_communication!=flag_communication){
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillRect(159,172,160,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font16); //ustawienie czcionki
		BSP_LCD_DisplayStringAt(159,172+12,(uint8_t*)"BLAD KOMUNIKACJI", LEFT_MODE);
	}
	//wyswietlanie ostrzezenia o bledzie czujnika
	if(flag_czujnik==0 ){//&& flag_previous_czujnik!=flag_czujnik){
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillRect(319,172,160,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font16); //ustawienie czcionki
		BSP_LCD_DisplayStringAt(319,172+12,(uint8_t*)"BLAD CZUJNIKA", LEFT_MODE);
	}
	//wyswietlanie przycisków auto i manual{		
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		if(flag_auto==1)BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);
		BSP_LCD_FillRect(429,0,50,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DisplayStringAt(429,0,(uint8_t*)"AUTO", LEFT_MODE);//auto
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		if(flag_auto==0)BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);
		BSP_LCD_FillRect(429,49,50,50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font12); //ustawienie czcionki
		BSP_LCD_DisplayStringAt(429,0+49,(uint8_t*)"MANUAL", LEFT_MODE);

//wyswietlanie przycisków sterowania termostatem i grzalkami

	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_FillRect(329,0,100,50);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_FillRect(329,49,100,50);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(329,0+49-12,(uint8_t*)"TEMP ZADANA", LEFT_MODE);
	BSP_LCD_DisplayStringAt(329,49+50-12,(uint8_t*)"STEROWANIE GRZALKA", LEFT_MODE);
		//+ -
		BSP_LCD_DrawLine(329+5,24,329+5+40,24);//+ temp zad
		BSP_LCD_DrawLine(329+24,0+5,329+24,49-5);
		BSP_LCD_DrawLine(329+5+50,24,329+5+40+50,24) ;//-temp zad
		BSP_LCD_DrawLine(329+5,24+50,329+5+40,24+50);//+ sterowanie grzalka
		BSP_LCD_DrawLine(329+24,0+5+50,329+24,49-5+50);
		BSP_LCD_DrawLine(329+5+50,24+50,329+5+40+50,24+50); //- sterowanie grzalka

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(0,230,479,43);
	
	//wyswietlanie paska temperatury
	if(temp/100.0f<temp_zadana+5) 						BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	else if(temp/100.0f>=temp_zadana+5 && flag_hot==0)	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	else								BSP_LCD_SetTextColor(LCD_COLOR_RED);
	if(temp<3000.0f)pasek=0;
	else pasek=(int)((temp/6000.0f)*480.0f)-240;
	
	if(flag_czujnik==1) BSP_LCD_FillRect(0,230,pasek,43);
	//wyswietlanie linii temp zadanej
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	if((int)((temp_zadana/60.0f)*480.0f)-240>=0 &&	(temp_zadana/60.0f)*480.0f-240<480)  BSP_LCD_FillRect((int)((temp_zadana/60.0f)*480.0f)-240,230,3,43);
	
	//wyswietlanie cyfr temperatury
	BSP_LCD_SetFont(&Font8); //ustawienie czcionki 
	BSP_LCD_DisplayStringAt(0,222,(uint8_t*)"30", LEFT_MODE);
	BSP_LCD_DisplayStringAt(79,222,(uint8_t*)"40", LEFT_MODE);
	BSP_LCD_DisplayStringAt(159,222,(uint8_t*)"50", LEFT_MODE);
	BSP_LCD_DisplayStringAt(239,222,(uint8_t*)"60", LEFT_MODE);
	BSP_LCD_DisplayStringAt(319,222,(uint8_t*)"70", LEFT_MODE);
	BSP_LCD_DisplayStringAt(399,222,(uint8_t*)"80", LEFT_MODE);
	BSP_LCD_DisplayStringAt(459,222,(uint8_t*)"90", LEFT_MODE);

	
	
	
	
	
	HAL_LTDC_ProgramLineEvent(hltdc, 272);
	
	
	
}

static void LCD_Config(void)
{
  /* LCD Initialization */ 
  BSP_LCD_Init();

  /* LCD Initialization */ 
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));

  /* Enable the LCD */ 
  BSP_LCD_DisplayOn(); 
  
  /* Select the LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);

  /* Clear the Foreground Layer */ 
  BSP_LCD_Clear(LCD_COLOR_WHITE);
	
  /* Configure the transparency for foreground and background :
     Increase the transparency */
  BSP_LCD_SetTransparency(1, 0xFF);
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 307;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 1);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void MX_UART_PC_Init(void){
	huart.Instance        = USART1;
  huart.Init.BaudRate   = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits   = UART_STOPBITS_1;
  huart.Init.Parity     = UART_PARITY_NONE;
  huart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart.Init.Mode       = UART_MODE_TX_RX;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&huart) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&huart) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_UART_MB_Init(void){
	huartmb.Instance        = USART6;
  huartmb.Init.BaudRate   = 115200;
  huartmb.Init.WordLength = UART_WORDLENGTH_9B;
  huartmb.Init.StopBits   = UART_STOPBITS_1;
  huartmb.Init.Parity     = UART_PARITY_EVEN;
  huartmb.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huartmb.Init.Mode       = UART_MODE_TX_RX;
  huartmb.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&huartmb) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&huartmb) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
	
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2; 
  htim2.Init.Prescaler = 10800-1; // 108 000 000 / 10 800 = 10 000
	htim2.Init.Period = 10000;    		// 10 000 / 10 000 = 1 Hz (1/1 s)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim2);     // Init timer
	HAL_TIM_Base_Start_IT(&htim2); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3; 
  htim3.Init.Prescaler = 108-1; // 108 000 000 / 108 = 1 000 000
	htim3.Init.Period = 10;       // 1 000 000 / 10 = 100 000 Hz = 100 kHz(1/100000 s = 1/100 ms = 10 us)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim3);     // Init timer
	HAL_TIM_Base_Start_IT(&htim3); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4; 
  htim4.Init.Prescaler = 108-1; // 108 000 000 / 108 = 1 000 000
	htim4.Init.Period = 50;       // 1 000 000 / 50 = 20 000 Hz = 20 kHz (1/20000 s = 1/20 ms = 50 us)
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim4);     // Init timer
	HAL_TIM_Base_Start_IT(&htim4); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5; 
  htim5.Init.Prescaler = 10800-1; // 108 000 000 / 10 800 = 10 000
	htim5.Init.Period = 10000;    // 10 000 / 10 000 = 1 Hz (1/1 s)
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim5);     // Init timer
	HAL_TIM_Base_Start_IT(&htim5); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = (RK043FN48H_HSYNC - 1);
  hltdc.Init.VerticalSync = (RK043FN48H_VSYNC - 1);
  hltdc.Init.AccumulatedHBP = (RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
  hltdc.Init.AccumulatedVBP = (RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
  hltdc.Init.AccumulatedActiveH = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
  hltdc.Init.AccumulatedActiveW = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
  hltdc.Init.TotalHeigh = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP + RK043FN48H_VFP - 1);
  hltdc.Init.TotalWidth = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP + RK043FN48H_HFP - 1);
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;

  
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 1) != HAL_OK)
  {
    Error_Handler();
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PE2   ------> QUADSPI_BK1_IO2
     PG14   ------> ETH_TXD1
     PE1   ------> FMC_NBL1
     PE0   ------> FMC_NBL0
     PB5   ------> USB_OTG_HS_ULPI_D7
     PB4   ------> S_TIM3_CH1
     PD7   ------> SPDIFRX_IN0
     PC12   ------> SDMMC1_CK
     PA15   ------> S_TIM2_CH1_ETR
     PE5   ------> DCMI_D6
     PE6   ------> DCMI_D7
     PG13   ------> ETH_TXD0
     PB7   ------> USART1_RX
     PB6   ------> QUADSPI_BK1_NCS
     PG15   ------> FMC_SDNCAS
     PG11   ------> ETH_TX_EN
     PD0   ------> FMC_D2_DA2
     PC11   ------> SDMMC1_D3
     PC10   ------> SDMMC1_D2
     PA12   ------> USB_OTG_FS_DP
     PI4   ------> SAI2_MCLK_A
     PG10   ------> SAI2_SD_B
     PD3   ------> DCMI_D5
     PD1   ------> FMC_D3_DA3
     PA11   ------> USB_OTG_FS_DM
     PF0   ------> FMC_A0
     PI5   ------> SAI2_SCK_A
     PI7   ------> SAI2_FS_A
     PI6   ------> SAI2_SD_A
     PG9   ------> DCMI_VSYNC
     PD2   ------> SDMMC1_CMD
     PI1   ------> SPI2_SCK
     PA10   ------> USB_OTG_FS_ID
     PF1   ------> FMC_A1
     PH14   ------> DCMI_D4
     PI0   ------> S_TIM5_CH4
     PA9   ------> USART1_TX
     PC9   ------> SDMMC1_D1
     PA8   ------> S_TIM1_CH1
     PF2   ------> FMC_A2
     PC8   ------> SDMMC1_D0
     PC7   ------> USART6_RX
     PF3   ------> FMC_A3
     PH4   ------> USB_OTG_HS_ULPI_NXT
     PG8   ------> FMC_SDCLK
     PC6   ------> USART6_TX
     PF4   ------> FMC_A4
     PH5   ------> FMC_SDNWE
     PH3   ------> FMC_SDNE0
     PF5   ------> FMC_A5
     PD15   ------> FMC_D1_DA1
     PB13   ------> USB_OTG_HS_ULPI_D6
     PD10   ------> FMC_D15_DA15
     PC3   ------> FMC_SDCKE0
     PD14   ------> FMC_D0_DA0
     PB12   ------> USB_OTG_HS_ULPI_D5
     PD9   ------> FMC_D14_DA14
     PD8   ------> FMC_D13_DA13
     PC0   ------> USB_OTG_HS_ULPI_STP
     PC1   ------> ETH_MDC
     PC2   ------> USB_OTG_HS_ULPI_DIR
     PB2   ------> QUADSPI_CLK
     PF12   ------> FMC_A6
     PG1   ------> FMC_A11
     PF15   ------> FMC_A9
     PD12   ------> QUADSPI_BK1_IO1
     PD13   ------> QUADSPI_BK1_IO3
     PH12   ------> DCMI_D3
     PA1   ------> ETH_REF_CLK
     PA4   ------> DCMI_HSYNC
     PC4   ------> ETH_RXD0
     PF13   ------> FMC_A7
     PG0   ------> FMC_A10
     PE8   ------> FMC_D5_DA5
     PD11   ------> QUADSPI_BK1_IO0
     PG5   ------> FMC_A15_BA1
     PG4   ------> FMC_A14_BA0
     PH7   ------> I2C3_SCL
     PH9   ------> DCMI_D0
     PH11   ------> DCMI_D2
     PA2   ------> ETH_MDIO
     PA6   ------> DCMI_PIXCLK
     PA5   ------> USB_OTG_HS_ULPI_CK
     PC5   ------> ETH_RXD1
     PF14   ------> FMC_A8
     PF11   ------> FMC_SDNRAS
     PE9   ------> FMC_D6_DA6
     PE11   ------> FMC_D8_DA8
     PE14   ------> FMC_D11_DA11
     PB10   ------> USB_OTG_HS_ULPI_D3
     PH6   ------> S_TIM12_CH1
     PH8   ------> I2C3_SDA
     PH10   ------> DCMI_D1
     PA3   ------> USB_OTG_HS_ULPI_D0
     PA7   ------> ETH_CRS_DV
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB0   ------> USB_OTG_HS_ULPI_D1
     PE7   ------> FMC_D4_DA4
     PE10   ------> FMC_D7_DA7
     PE12   ------> FMC_D9_DA9
     PE15   ------> FMC_D12_DA12
     PE13   ------> FMC_D10_DA10
     PB11   ------> USB_OTG_HS_ULPI_D4
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
	
  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE1 PE0 FMC_D5_Pin FMC_D6_Pin 
                           FMC_D8_Pin FMC_D11_Pin FMC_D4_Pin FMC_D7_Pin 
                           FMC_D9_Pin FMC_D12_Pin FMC_D10_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|FMC_D5_Pin|FMC_D6_Pin 
                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin 
                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin 
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin 
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9 
                           PC8 */
  GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG15 PG8 PG1 PG0 
                           FMC_BA1_Pin FMC_BA0_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_1|GPIO_PIN_0 
                          |FMC_BA1_Pin|FMC_BA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_D2_Pin FMC_D3_Pin FMC_D1_Pin FMC_D15_Pin 
                           FMC_D0_Pin FMC_D14_Pin FMC_D13_Pin */
  GPIO_InitStruct.Pin = FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin 
                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3 
                           PF4 PF5 PF12 PF15 
                           PF13 PF14 PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_15 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_D0_Pin */
  GPIO_InitStruct.Pin = SDMMC_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SDMMC_D0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH14 PH12 PH9 PH11 
                           PH10 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_11 
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_SDNME_Pin PH3 */
  GPIO_InitStruct.Pin = FMC_SDNME_Pin|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_SCL_Pin LCD_SDA_Pin */
  GPIO_InitStruct.Pin = LCD_SCL_Pin|LCD_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
  //GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  //HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  //GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin : PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
	/* ^                                              ^ */
	/*  ^                                            ^  */
	/* - ^                                          ^ - */
	/* -- ^                                        ^ -- */
	/* --- ^                                      ^ --- */
	/* ---- ^                                    ^ ---- */
	/* ----- ^                                  ^ ----- */
	/* ------ ^                                ^ ------ */
	/* ------- ^                              ^ ------- */
	/* -------- ^                            ^ -------- */
	/* --------- ^                          ^ --------- */
	/* ---------- ^                        ^ ---------- */
	/* ----------- ^                      ^ ----------- */
	/* ------------ ^                    ^ ------------ */
	/* ------------- ^                  ^ ------------- */
	/* -------------- ^                ^ -------------- */
	/* --------------- ^              ^ --------------- */
	/* ---------------- ^            ^ ---------------- */
	/* ----------------- ^          ^ ----------------- */
	/* ------------------ ^        ^ ------------------ */
	/* ------------------- ^      ^ ------------------- */
	/* -------------------- ^    ^ -------------------- */
	/* --------------------- ^  ^ --------------------- */
	/* ---------------------- ^^ ---------------------- */
	/* --------------------- ^^ --------------------- */
	/* -------------------- ^^ -------------------- */
	/* ------------------- ^^ ------------------- */
	/* ------------------ ^^ ------------------ */
	/* ----------------- ^^ ----------------- */
	/* ---------------- ^^ ---------------- */
	/* --------------- ^^ --------------- */
	/* -------------- ^^ -------------- */
	/* ------------- ^^ ------------- */
	/* ------------ ^^ ------------ */
	/* ----------- ^^ ----------- */
	/* ---------- ^^ ---------- */
	/* --------- ^^ --------- */
	/* -------- ^^ -------- */
	/* ------- ^^ ------- */
	/* ------ ^^ ------ */
	/* ----- ^^ ----- */
	/* ---- ^^ ---- */
	/* --- ^^ --- */
	/* -- ^^ -- */
	/* - ^^ - */
	/*  ^^  */
	/* ^^ */
