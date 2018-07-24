# Digital-Control-for-Lighting-HBLED

This project controls the brightness of a high-brightness light-emitting diode (HBLED) by modulating the buck converter which drives it. Hardware used in this project are FRDM KL25Z Dev board and a shield for the board with buck converter circuitry and HBLED.

We can determine the LED current by measuring the voltage across R10. This signal is brought out to the S- signal on J13 (on the shield).
• Connect the scope’s channel 1 positive differential input (1+, orange) to S- on J13.
• Connect the scope’s ground and channel 1 negative differential input (1-, orange and white) to a 0V signal on J13 or J12.

We will use the digital to analog converter (DAC0) to indicate the setpoint (desired current output) of the system. Use 2.2 mV to represent each mA of current. For example, the DAC should indicate a setpoint of 100 mA with a voltage of 220 mV. This will allow us to compare the voltage across R10 directly with the DAC output.
• Connect the scope’s channel 2 positive differential input (2+, blue) to DAC on J8.
• Connect the scope’s channel 2 negative differential input (2-, blue and white) to 0V on J8.

The goal is to drive the HBLED roughly once per second with the following current profile. The current should be 0 mA before and after each flash.

Start time  End Time  HBLED Current
0 ms        1 ms      10 mA
1 ms        2 ms      20 mA
2 ms        3 ms      30 mA
3 ms        4 ms      40 mA
4 ms        5 ms      50 mA
5 ms        6 ms      40 mA
6 ms        7 ms      30 mA
7 ms        8 ms      20 mA
8 ms        9 ms      10 mA
9 ms        -          0 mA

The program implements 4 types of controller with each controller incresing the CPU efficiency. The type of controller can be selected by setting the 'section' to BasicController, SynchSampling, ElimADC, Raise_Cont_Freq, or ADCIntFilter. The program also implements 3 types of feddback control logic BangBang, Proportional and PID.

In BasicController the PIT ISR calls the controller function Control_HBLED at a frequency of roughly 5 kHz. Control_HBLED will be responsible for sampling the ADC output, calculating the new set point as per the control feedback logic and the difference between the previous set point and actual output.

In SynchSampling PWM timer overflow interrupt service routine calls Control_HBLED at 8 kHz. The timer overflows at 16 kHz, so make sure the ISR only calls Control_HBLED every other call.

In ElimADC, TPM0 directly triggers the ADC to start a conversion. Only the ADC ISR will call Control_HBLED.

In Raise_Cont_Freq, we raise the control loop frequency to 16 kHz but keep the switching frequency at 16 kHz.

In ADCIntFilter, we use the ADC’s automatic compare function to eliminate ADC interrupts for conversions which indicate a current error of less than 1 mA, and therefore require little (if any) change to the duty cycle. This will save CPU time for other processing activities.
