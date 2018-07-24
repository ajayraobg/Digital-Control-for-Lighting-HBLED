
//*************************************************//
/*Project: Digital control for lighting			*/
/*Author: Ajay Rao                   		   	*/
/*Affiliation: NCSU                   		   	*/
/*Version: 1.0									*/
/*E-mail:abrao2@ncsu.edu                      	*/
//*************************************************//

 
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdint.h>
#include "gpio_defs.h"

#include "timers.h"
#include "delay.h"
#include "LEDS.h"

#include "HBLED.h"
#include "debug.h"


volatile int g_set_current = 0;
volatile int measured_current;
volatile int16_t duty_cycle=0;
volatile int error;
uint16_t res;
enum {BangBang, Incremental, Proportional, PID} control_mode=BangBang;

enum versions section = BasicController;
char buffer[50];

typedef struct {
	float dState; // Last position input
	float iState; // Integrator state
	float iMax, iMin; // Maximum and minimum allowable integrator state
	float iGain, // integral gain
	pGain, // proportional gain
	dGain; // derivative gain
} SPid;

SPid plantPID = {0, // dState
	0, // iState
	LIM_DUTY_CYCLE, // iMax
	-LIM_DUTY_CYCLE, // iMin
	2.3153, // iGain
	1.3712, // pGain
	1.01319  // dGain
};


float UpdatePID(SPid * pid, float error, float position)
{
	float pTerm, dTerm, iTerm;

	// calculate the proportional term
	pTerm = pid->pGain * error;
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState; // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;

	return pTerm + iTerm - dTerm;
}

extern void Control_HBLED(void) {

	PTB->PSOR = MASK(DBG_4);	
	// Start conversion
	if((section != ElimADC) & (section != Raise_Cont_Freq) & (section != ADCIntFilter))
	{
		ADC0->SC1[0] = ADC_SC1_AIEN(1) | ADC_SENSE_CHANNEL;
		while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
		; // wait until end of conversion
		res = ADC0->R[0];
	}
	measured_current = (res*V_REF*MA_SCALING_FACTOR)/(ADC_FULL_SCALE*R_SENSE);
	sprintf(buffer,"%d \n", measured_current); 

	switch (control_mode) {
		case BangBang:
			if (measured_current < g_set_current)
				duty_cycle = LIM_DUTY_CYCLE;
			else
				duty_cycle = 0;
			break;
		case Incremental:
			if (measured_current < g_set_current)
				duty_cycle+=10;
			else
				duty_cycle-=10;
			break;
		case Proportional:
			duty_cycle += g_set_current - measured_current;
			break;
		case PID:
			duty_cycle = UpdatePID(&plantPID, g_set_current - measured_current, measured_current);
			break;
		default:
			break;
	}

	if (duty_cycle < 0)
		duty_cycle = 0;
	else if (duty_cycle > LIM_DUTY_CYCLE)
		duty_cycle = LIM_DUTY_CYCLE;
	PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, duty_cycle);
	PTB->PCOR = MASK(DBG_4);
}

void Set_DAC_mA(unsigned int current) {
	unsigned int code = MA_TO_DAC_CODE(current);
	DAC0->DAT[0].DATL = DAC_DATL_DATA0(code);
	DAC0->DAT[0].DATH = DAC_DATH_DATA1(code>>8);
}


void Set_DAC(unsigned int code) {
	DAC0->DAT[0].DATL = DAC_DATL_DATA0(code);
	DAC0->DAT[0].DATH = DAC_DATH_DATA1(code>>8);
}

void Init_DAC(void) {
  // Enable clock to DAC and Port E
	SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Select analog for pin
	PORTE->PCR[DAC_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[DAC_POS] |= PORT_PCR_MUX(0);	
		
	// Disable buffer mode
	DAC0->C1 = 0;
	DAC0->C2 = 0;
	
	// Enable DAC, select VDDA as reference voltage
	DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
	Set_DAC(0);
}

void Init_ADC(void) {
	// Configure ADC to read Ch 8 (PTB 0)
	
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x0C; // 16 bit
	if((section == ElimADC) | (section == Raise_Cont_Freq)) //Set Hardware trigger for ADC conversion
	{
		ADC0->SC2 = ADC_SC2_REFSEL(0)| ADC_SC2_ADTRG(1);
	}
	else if (section == ADCIntFilter) // Set Hardware trigger for ADC conversion along with enabling automatic compare
	{
		ADC0->SC2 = (ADC_SC2_REFSEL(0)| ADC_SC2_ADTRG(1)| ADC_SC2_ACFE(1)| ADC_SC2_ACREN(1) | ADC_SC2_ACFGT(0));
	}
	else 
	{
		ADC0->SC2 = ADC_SC2_REFSEL(0);
	}
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB->PCR[0]  &= PORT_PCR_MUX(7); 

	if((section == ElimADC) | (section ==Raise_Cont_Freq) | (section ==ADCIntFilter)) //Enable Interrupt for ADC and select TPM0 overfow as trigger to start conversion
	{
		SIM->SOPT7 |=  SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN(1);	
		ADC0->SC1[0] = ADC_SC1_AIEN(1) | ADC_SENSE_CHANNEL;
	}
}

void ADC0_IRQHandler()
{ 
	static int counter1 = 1;
	PTB->PSOR = MASK(DBG_3);
	res = ADC0->R[0];
	if(section == ElimADC)
	{
		if(counter1 % 2 == 0)
		{

			Control_HBLED();	
		}
	}
	else 
	{
		Control_HBLED();
	}
	counter1++;
	PTB->PCOR = MASK(DBG_3);
}

/*----------------------------------------------------------------------------
  MAIN function	
 *----------------------------------------------------------------------------*/
int main (void) 
	{

	Init_Debug_Signals();
	Init_DAC();
	Init_RGB_LEDs();
	Control_RGB_LEDs(1,1,0);
	Delay(100);
	Control_RGB_LEDs(0,0,1);
	Init_ADC();
	
	// Configure PTE31 for TPM0 Ch 4
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[31]  &= PORT_PCR_MUX(7);
	PORTE->PCR[31]  |= PORT_PCR_MUX(3);
	
	switch(section)
	{
		case BasicController:
				PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, 0);
				PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, 0);
				PIT_Init(4769);
				PIT_Start();
				break;
		case SynchSampling:
				PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, 0);
				PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, 0);
				break;
		case ElimADC:
		case Raise_Cont_Freq:
		case ADCIntFilter:
				PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, 0);
				PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, 0);
				NVIC_SetPriority(ADC0_IRQn, 5);
				NVIC_ClearPendingIRQ(ADC0_IRQn);
				NVIC_EnableIRQ(ADC0_IRQn);
				break;
		default:
				break;
	}

	while (1) 
	{
		g_set_current = 0;
		for (g_set_current = 10; g_set_current < 50; g_set_current += 10) 
		{
				if (section==ADCIntFilter)
					{
						float x = ((g_set_current-(1)) * ADC_FULL_SCALE*R_SENSE)/(V_REF*MA_SCALING_FACTOR);
						float y = ((g_set_current+(1)) * ADC_FULL_SCALE*R_SENSE)/(V_REF*MA_SCALING_FACTOR);
						ADC0_CV1=x;
						ADC0_CV2=y;
					}
					PTB->PSOR = MASK(DBG_1);
					Set_DAC_mA(g_set_current);
					switch(control_mode) //Corrected delay for 1ms based on type of controller
						{		
							case(BangBang): 
								ShortDelay(4090);
								break;
							case(Proportional): 
								ShortDelay(4097);
								break;
							case(PID): 
								ShortDelay(4097);
								break;	
							default:
								break;
						}
		}
		for (g_set_current = 50; g_set_current >= 0; g_set_current -= 10) 
			{				
				if (section==ADCIntFilter)
					{
						float a = ((g_set_current-(1)) * ADC_FULL_SCALE*R_SENSE)/(V_REF*MA_SCALING_FACTOR);
						float b = ((g_set_current+(1)) * ADC_FULL_SCALE*R_SENSE)/(V_REF*MA_SCALING_FACTOR);
						ADC0_CV1=a;
						ADC0_CV2=b;
					}
					if(g_set_current == 0) 
					{
						PTB->PCOR = MASK(DBG_1);
					}
					Set_DAC_mA(g_set_current); //Corrected delay for 1ms based on type of controller
					switch(control_mode)
						{		
							case(BangBang): 
								ShortDelay(4090);
								break;
							case(Proportional): 
								ShortDelay(4097);
								break;
							case(PID): 
								ShortDelay(4097);
								break;
							default:
								break;				
						}
			}
		g_set_current = 0;
		switch(control_mode) //Corrected delay for 1s based on type of controller
			{		
				case(BangBang): 
					Delay(476);
					break;
				case(Proportional): 
					Delay(404);
					break;
				case(PID): 
					Delay(396);
					break;
				default:
					break;				
			}		
	}
}


