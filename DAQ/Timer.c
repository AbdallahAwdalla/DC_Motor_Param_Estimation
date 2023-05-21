#include "common.h"




/**
 * Function TPM0_Init
 * \brief  Initialize TPM0 
 * REG[0  -  N] MOD 
 * Number of counts = MOD + 1 
 * Time of each count Tc 
 * What's Tc 
 * The Clk is illustrated below 
 * ___________           |___________           |___________           
 *            |          |           |          |           |           
 *            |__________|           |__________|           |__________
 *                       | <--------Tc -------->|
 * Tc = 1/fc --> fc is the freq of the Clk 
 * Total time of period = Tc * (MOD + 1 ) = (MOD +1)/fc
 * 
 * 
 */

void TPM0_Init() {


    /**
     *  ______________         ______________         ______________          
     *                |       |              |       |              |        
     *                |_______|              |_______|              |_______
     * 
     *  0             x      200   (uS)
     * 
     * (MOD + 1 ) /fc = 200us 
     * 
     * MOD = (200 * 24 ) -1
     * CnV = Depending on the duty cycle 
     * 
     * 
     * 
     * 
     * PS = 0 (no pre-scaller)
     * 
     * period = 177 uS  = (MOD + 1)/ fc
     * MOD + 1 = 4248
     * MOD = 4247
     * 
     * CnV = 150 * 24 = 3600
     * 
     */

    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // Enable TPM0 clock
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // Select MCGFLLCLK clock as TPM0 clock source

    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; // Enable Port D clock
    PORTD->PCR[3] = PORT_PCR_MUX(4); //Configure PTD3 as TPM0_CH3

    // TPM0->SC = 0; // Disable TPM0 before configuring
    TPM0->CONTROLS[3].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Set TPM0 channel 3 to edge-aligned PWM mode
    TPM0->CONTROLS[3].CnV = (uint32_t)(0);
    TPM0->MOD = (uint16_t)((1U<<16U) - 1); // Set TPM0 counter period to the pulse width (150+27) microseconds

    TPM0->SC = TPM_SC_PS(0) | TPM_SC_CMOD(01); // Set TPM0 prescaler to divide by 1 and enable TPM0 counter
}



void set_pwm(uint16_t in)
{
    TPM0->CONTROLS[3U].CnV = (uint32_t)(in);
}
