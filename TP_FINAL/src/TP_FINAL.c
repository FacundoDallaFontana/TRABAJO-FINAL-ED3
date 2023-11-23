/*
===============================================================================
 Name        : RADAR CON SALIDA GRAFICA POR DAC Y CONTROL MEDIANTE UART
 Author      : Dalla Fontana Crespillo, Facundo Nicolas
 Version     : Final
 Description : Programa que contrala un servo y recibe mediciones de un sensor ultrasonico, para graficar
			   estas con el DAC y visualizarla con la ayuda de un osciloscopio. Tambien trae la posibilidad
			   de variar la velocidad de barrido del servo mediante el uso de un potenciometro. Ademas
			   es posible mediante comunicacion UART, a traves del teclado del PC, el prendido y apagado
			   del barrido del servo, y la posibilidad de mover el servo y tomar un mediciion puntual(barrido->off).
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif
#include "lpc17xx_dac.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"
#include "stdlib.h"
#include "stdio.h"

#define To 22500
#define Tf 52500
#define DMA_SIZE 25
#define NUM_SAMPLE 25
#define FUNC_FREQ_IN_HZ 100
#define PCLK_DAC_IN_MHZ 25

#define TRIG_PIN 6  // Puerto P0.0 para el pin TRIG
#define ECHO_PIN 7  // Puerto P0.1 para el pin ECHO

GPDMA_LLI_Type LLI1;
GPDMA_Channel_CFG_Type GPDMACfg;
uint32_t valor = 0;

//Variables globales
uint32_t distanciaCm;
unsigned int distLineaRectaCm;
char distLineaRectaCmArray[16];
uint8_t muestras = 5;
uint8_t muestrasEnt = 5;
uint32_t distanciaDac;
uint32_t radarSamples[NUM_SAMPLE] = {65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472,
		65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472, 65472};
float correccion[NUM_SAMPLE] = {2, 1.743, 1.556, 1.414, 1.305, 1.221, 1.155, 1.103, 1.062, 1.0353, 1.0154, 1.004, 1,
						1.004, 1.0154, 1.0353, 1.062, 1.103, 1.155, 1.221, 1.305, 1.414, 1.556, 1.743, 2};
uint32_t cclk;
uint8_t posicion = 0;
uint8_t dirFlag = 1;
uint32_t tiempo;
uint8_t on = 1;
uint8_t command;

void PWM_Init();
void setPWM(uint32_t value);
void configDAC();
void configSensor();
void configSysTick();
void configDMA();
void configADC();
void configTimer();
void sendTriggerPulse();
uint32_t measureEchoPulse();
void medirDistancia();
void configUART();
void send_data();






int main() {
    PWM_Init();
    configADC();
    configTimer();
    configDAC();
    configSensor();
    configSysTick();
    configUART();
    configDMA();
	GPDMA_ChannelCmd(1, ENABLE);

	while(1);

    return 0;
}



/***************************************
*                                      *
*           Config Functions           *
*                                      *
****************************************/
void PWM_Init() {
    LPC_SC->PCONP |= (1 << 6); // Enciende el módulo PWM1
    LPC_PINCON->PINSEL4 |= (1 << 10); // Configura P2.5 como PWM1.6
    LPC_PINCON->PINSEL4 &= ~(1 << 11);

    // Configura el PWM
    LPC_PWM1->PR = 0; // Prescaler = 1
    LPC_PWM1->MR0 = 500000; // Frecuencia de PWM = 50 Hz
    LPC_PWM1->MR6 = 25000; // Valor inicial del pulso

    // Configura el modo PWM
    LPC_PWM1->MCR |= (1 << 1); // Resetea el contador al llegar a MR0
    LPC_PWM1->CTCR = 0; // Modo temporizador

    // Habilita el canal PWM6 y el modo PWM
    LPC_PWM1->PCR |= (1 << 6)|(1 << 14); // Habilita PWM6
    LPC_PWM1->TCR = 1; // Inicia el contador del temporizador PWM
}

void configDAC(void) {

	PINSEL_CFG_Type pinCfg;// P0.26 COMO AOUT
	pinCfg.Funcnum = PINSEL_FUNC_2;
	pinCfg.OpenDrain = 0;
	pinCfg.Pinmode = 1;//Ninguno
	pinCfg.Pinnum = PINSEL_PIN_26;
	pinCfg.Portnum = PINSEL_PORT_0;
	PINSEL_ConfigPin(&pinCfg);

	DAC_CONVERTER_CFG_Type dacCfg;
	dacCfg.CNT_ENA = SET;
	dacCfg.DMA_ENA = SET;
	DAC_Init(LPC_DAC);
	/*Set timeout*/
	uint32_t tmp;
	tmp = (PCLK_DAC_IN_MHZ * 1000000)/(FUNC_FREQ_IN_HZ * NUM_SAMPLE);
	DAC_SetDMATimeOut(LPC_DAC, tmp);
	DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
}

void configSensor(void){
	// Configurar los pines TRIG y ECHO
	LPC_GPIO0->FIODIR |= (1 << TRIG_PIN);  // Establecer el pin TRIG como salida
	LPC_GPIO0->FIODIR &= ~(1 << ECHO_PIN); // Establecer el pin ECHO como entrada
}

void configSysTick(){
	SysTick->LOAD = 2500000;//25ms
	SysTick->CTRL = (1) | (1<<1) | (1<<2);
	SysTick->VAL = 0;

}

void configDMA(){
	//GPDMA_LLI_Type LLI1;
	LLI1.SrcAddr = (uint32_t) radarSamples;
	LLI1.DstAddr = (uint32_t) &(LPC_DAC->DACR);
	LLI1.NextLLI = (uint32_t) &LLI1;
	LLI1.Control = DMA_SIZE
				   | (2<<18) //source width 32 bits
				   | (2<<21) //dest width 32 bits
				   | (1<<26); //source increment

	GPDMA_Init();

	//GPDMA_Channel_CFG_Type GPDMACfg;
	GPDMACfg.ChannelNum = 1;
	GPDMACfg.SrcMemAddr = (uint32_t) radarSamples;
	GPDMACfg.DstMemAddr = 0;
	GPDMACfg.TransferSize = DMA_SIZE;
	GPDMACfg.TransferWidth = 0;
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	GPDMACfg.SrcConn = 0;
	GPDMACfg.DstConn = GPDMA_CONN_DAC;
	GPDMACfg.DMALLI = (uint32_t)&LLI1;
	GPDMA_Setup(&GPDMACfg);



}
void configTimer(void){
	TIM_TIMERCFG_Type configTimer;
	configTimer.PrescaleOption = TIM_PRESCALE_TICKVAL;
	configTimer.PrescaleValue = 2500; //TC se incrementa cada (CCLK/4)^-1 * 2500 = 100us
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &configTimer);

	TIM_MATCHCFG_Type configMatch;
	configMatch.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
	configMatch.IntOnMatch = DISABLE;
	configMatch.MatchChannel = 1;
	configMatch.MatchValue = 1000;
	configMatch.ResetOnMatch = ENABLE;
	configMatch.StopOnMatch = DISABLE;
	TIM_ConfigMatch(LPC_TIM0, &configMatch);

	TIM_Cmd(LPC_TIM0, ENABLE);
}
void configADC(void){

	PINSEL_CFG_Type pinCfg;
	pinCfg.Funcnum = PINSEL_FUNC_1;
	pinCfg.OpenDrain = 0;
	pinCfg.Pinmode = 1;//Ninguno
	pinCfg.Pinnum = PINSEL_PIN_23;
	pinCfg.Portnum = PINSEL_PORT_0;
	PINSEL_ConfigPin(&pinCfg);

	ADC_Init(LPC_ADC, 200000);
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_FALLING);

}

void configUART(){
    //SERIAL
    PINSEL_CFG_Type pinCfg;
	pinCfg.Funcnum = 1;
	pinCfg.OpenDrain = 0;
	pinCfg.Pinmode = 0;
	pinCfg.Pinnum = 2;
	pinCfg.Portnum = 0;
	PINSEL_ConfigPin(&pinCfg);

	pinCfg.Funcnum = 1;
	pinCfg.OpenDrain = 0;
	pinCfg.Pinmode = 0;
	pinCfg.Pinnum = 3;
	pinCfg.Portnum = 0;
	PINSEL_ConfigPin(&pinCfg);

    UART_CFG_Type UARTConfigStruct;
	UART_ConfigStructInit(&UARTConfigStruct);
	UART_Init(LPC_UART0, &UARTConfigStruct);

	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UART_FIFOConfig(LPC_UART0, &UARTFIFOConfigStruct);

	UART_TxCmd(LPC_UART0, ENABLE);
    UART_IntConfig(LPC_UART0, UART_INTCFG_RBR, ENABLE);
    UART_IntConfig(LPC_UART0, UART_INTCFG_RLS, ENABLE);
	NVIC_EnableIRQ(UART0_IRQn);

}

/***************************************
*                                      *
*            GP Functions              *
*                                      *
****************************************/

void setPWM(uint32_t value) {
    LPC_PWM1->MR6 = value;
    LPC_PWM1->LER |= (1 << 6); // Habilita la carga del nuevo valor de MR2
}


void sendTriggerPulse(void) {
    LPC_GPIO0->FIOPIN |= (1 << TRIG_PIN);  // Establecer el pin TRIG en alto
    // Esperar 10us
    for (volatile int i = 0; i < 72; ++i);
    LPC_GPIO0->FIOPIN &= ~(1 << TRIG_PIN);  // Establecer el pin TRIG en bajo
}

// Función para medir el tiempo de eco
uint32_t measureEchoPulse(void) {
// Esperar hasta que el pin ECHO se vuelva alto
while (!(LPC_GPIO0->FIOPIN & (1 << ECHO_PIN)));

// Medir el tiempo que el pin ECHO permanece alto
uint32_t pulseStart = 0;
while (LPC_GPIO0->FIOPIN & (1 << ECHO_PIN)) {
    pulseStart++;
}

return pulseStart;
}

void medirDistancia(){

	if(muestras>0){
		sendTriggerPulse();
		valor += measureEchoPulse();
		muestras--;
	}
	if(muestras == 0){
		distLineaRectaCm = (valor/muestrasEnt)/160;
		distanciaCm = (valor/muestrasEnt)/(160*correccion[posicion]);
		muestras = 1 + 50/distanciaCm;
		muestrasEnt = muestras;
		if(distanciaCm > 50)
			distanciaDac = 1023;
		else
			distanciaDac = distanciaCm*(1023/50);//Valores de 0 a 1023
		distanciaDac = distanciaDac << 6;//Shifteamos para poder cargar en registro DACR
		radarSamples[posicion] = distanciaDac;//Guardamos en bloquye de memoria
		valor = 0;
		if(dirFlag){
			posicion++;
		}else{
			posicion--;
		}
	}
}

void send_data(void){
	uitoa(distLineaRectaCm, distLineaRectaCmArray, 10);
	UART_Send(LPC_UART0, "Objeto a: ", sizeof("Objeto a: "), BLOCKING);
	UART_Send(LPC_UART0, distLineaRectaCmArray, sizeof(distLineaRectaCmArray), BLOCKING);
	UART_Send(LPC_UART0, " cm.\n\r", sizeof(" cm.\n\r"), BLOCKING);

    for(size_t i = 0; i < sizeof(distLineaRectaCmArray); ++i)
    distLineaRectaCmArray[i] = 0;


}


/***************************************
*                                      *
*            IRQ Handlers              *
*                                      *
****************************************/

void SysTick_Handler(){
	if(dirFlag){//un sentido
    	int tmp= To + posicion*(Tf-To)/(NUM_SAMPLE - 1);
    	setPWM(tmp);
    	medirDistancia();
    	if(posicion == (NUM_SAMPLE - 1)){
    		dirFlag=0;
    	}

	}else{//otro sentido
    	int tmp= To + posicion*(Tf-To)/(NUM_SAMPLE - 1);
    	setPWM(tmp);
    	medirDistancia();
    	if(posicion == 0){
    		dirFlag = 1;
    	}
	}
	SysTick->CTRL &= SysTick->CTRL;
	tiempo = (uint32_t) (ADC_ChannelGetData(LPC_ADC, 0));
	tiempo = 2000000 + tiempo*1221;
	SysTick->LOAD = tiempo;

}




void UART0_IRQHandler(void) {

    // Comprobar si la interrupción es por recepción de datos
    if (UART_GetIntId(LPC_UART0) & UART_IIR_INTID_RDA) {
        command = UART_ReceiveByte(LPC_UART0); // Leer el dato recibido
        if(on){
        	if(command == 'O')
        		SysTick->CTRL &= ~(1<<1);
        		SysTick->CTRL &= SysTick->CTRL;
        		on = 0;
        }else{//Radar sin moverse
        	if(command == 'O'){
        		SysTick->CTRL |= (1<<1);
        		on = 1;
        	}
    		if (command == 'D'){
        		uint32_t tmp = LPC_PWM1->MR6;
        		tmp += (Tf-To)/(NUM_SAMPLE - 1);
        		if(tmp > Tf)
        			tmp = Tf;
        		setPWM(tmp);
    		}
    		if (command == 'A'){
        		uint32_t tmp = LPC_PWM1->MR6;
        		tmp -= (Tf-To)/(NUM_SAMPLE - 1);
        		if(tmp < To)
        			tmp = To;
        		setPWM(tmp);
    		}
    		if (command == 'W'){
    			uint32_t tmp;
    			sendTriggerPulse();
    			tmp = measureEchoPulse();
    			distLineaRectaCm = tmp/160;
    			send_data();

    		}
        }

    }
}


