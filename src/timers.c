#include "timers.h"
#include <MKL25Z4.h>
#include "HBLED.h"
#include "GPIO_defs.h"
#include "debug.h"


void PIT_IRQHandler() {

	static int counter=0;
		//PTB->PSOR = MASK(DBG_4);
	if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
		// clear status flag for timer channel 0
		PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;
		
		// Do ISR work here
		
			Control_HBLED();	

	} 

}

void TPM0_IRQHandler()
{
	static long int counter=1;
	PTB->PSOR = MASK(DBG_2);
	TPM0->SC |= TPM_SC_TOIE_MASK; // clear status flag
	//ISR work
	if(counter % 2 == 0)
	{
		Control_HBLED();	
	}
	counter++;
	PTB->PCOR = MASK(DBG_2);
}
	

void PIT_Init(unsigned period) {
	// Enable clock to PIT module
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable module, freeze timers in debug mode
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	PIT->MCR |= PIT_MCR_FRZ_MASK;
	
	// Initialize PIT0 to count down from argument 
	PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(period);

	// No chaining
	PIT->CHANNEL[0].TCTRL &= PIT_TCTRL_CHN_MASK;
	
	// Generate interrupts
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;

	/* Enable Interrupts */
	NVIC_SetPriority(PIT_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(PIT_IRQn); 
	NVIC_EnableIRQ(PIT_IRQn);	
}


void PIT_Start(void) {
// Enable counter
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void PWM_Init(TPM_Type * TPM, uint8_t channel_num, uint16_t period, uint16_t duty)
{
	//turn on clock to TPM 
	switch ((int) TPM) {
		case (int) TPM0:
			SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
			break;
		case (int) TPM1:
			SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
			break;
		case (int) TPM2:
			SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
			break;
		default:
			break;
	}
	//set clock source for tpm
	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	//load the counter and mod
	TPM->MOD = period;	
	//set channel to center-aligned low-true PWM
	TPM->CONTROLS[channel_num].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	//set TPM to up-down and divide by 1 prescaler and clock mode
	if(section == SynchSampling) 
	{
		TPM->SC = (TPM_SC_CPWMS_MASK | TPM_SC_PS(0)) | TPM_SC_TOIE_MASK;
	}
	else
	{
		TPM->SC = (TPM_SC_CPWMS_MASK | TPM_SC_PS(0));
	}
	//set trigger mode and keep running in debug mode
	TPM->CONF |= TPM_CONF_TRGSEL(0xA) | TPM_CONF_DBGMODE(3);
	// Set initial duty cycle
	TPM->CONTROLS[channel_num].CnV = duty;
 // Start the timer counting
	TPM->SC |= TPM_SC_CMOD(1);
	if(section == SynchSampling)
	{
		NVIC_SetPriority(TPM0_IRQn, 3);
	  NVIC_ClearPendingIRQ(TPM0_IRQn);
	  NVIC_EnableIRQ(TPM0_IRQn);
	}
}

void PWM_Set_Value(TPM_Type * TPM, uint8_t channel_num, uint16_t value) {
	TPM->CONTROLS[channel_num].CnV = value;
}
