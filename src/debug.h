#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

#define MASK(x) (1UL << (x))

// Debug Signals on port B
#define DBG_1 1 	
#define DBG_2 2	  
#define DBG_3 3		
#define DBG_4 8		
#define DBG_5 9		
#define DBG_6 10  
#define DBG_7 11

#define DBG_LED_ON 	DBG_1  
#define DBG_CONTROLLER	DBG_2 
#define DBG_IRQ_TPM	DBG_3 
#define DBG_IRQ_ADC	DBG_4 
//#define DBG_	DBG_4 
//#define DBG_	DBG_5 
//#define DBG_	DBG_6
//#define DBG_	DBG_7   

void Init_Debug_Signals(void);


typedef enum versions {BasicController, SynchSampling, ElimADC, Raise_Cont_Freq, ADCIntFilter};
extern enum versions section;
#endif // DEBUG_H
