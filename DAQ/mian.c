#include "common.h"
#define ENCODER_A_PIN 4U
#define ENCODER_B_PIN 5U

volatile int encoderCount = 0;
volatile int lastEncoderCount = 0;
volatile int encoderDirection = 0;
volatile uint32_t lastTickTime = 0U;
volatile float rpm;
volatile uint32_t counts ;
uint16_t SelPeriod;

uint16_t periodArray[5U] = {
    0U,
    1U<<14,
    1U<<15,
    49151,
    (1U<<16)-1U,
};

float getRPM(void);

void vPeriodicTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(2);

    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        rpm = getRPM(); // get the RPM value

        char buffer[15u];
        sprintf(buffer, "%f\t%d\n", rpm ,SelPeriod);
        for (int i = 0; buffer[i] != '\0'; i++) {
            while (!(UART0->S1 & UART_S1_TDRE_MASK)); // Wait until the transmit data register is empty
            UART0->D = buffer[i]; // Send the character over UART
        }
        

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
void vPeriodicTask2(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(5000);
    static uint8_t idx = 0u;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        SelPeriod = periodArray[idx] ;
        set_pwm(SelPeriod);
        idx++; 
        idx%= 5U;

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void encoderISR(void) {
    uint32_t currentTime = SysTick->VAL;
    uint32_t elapsedTime = lastTickTime - currentTime;

    if (elapsedTime > 100U) { // ignore very short pulses
        counts++;
        uint8_t encoderB = (PTA->PDIR >> ENCODER_B_PIN) & 1U;
        int encoderDelta = (encoderB) ? 1 : -1; // determine direction
        encoderCount += encoderDelta;
        encoderDirection = encoderDelta;
    }
}

void initEncoder(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; // enable GPIOA clock
    PORTA->PCR[ENCODER_A_PIN] = (PORT_PCR_MUX(1U) | PORT_PCR_PE_MASK | PORT_PCR_IRQC(9U)); // configure pins as GPIO and pull down input also interrup on rising edge 
    PORTA->PCR[ENCODER_B_PIN] = (PORT_PCR_MUX(1U) | PORT_PCR_PE_MASK);
    PTA->PDDR = 0U; // configure pins as input

    // configure SysTick timer for time measurements
    SysTick->LOAD = 0xFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    NVIC_EnableIRQ(PORTA_IRQn);
}

// void UART0_Init(uint32_t baud_rate) {
//     SIM->SCGC4 |= SIM_SCGC4_UART0_MASK; // Enable UART0 clock
//     SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1); // Select the clock source for UART0
//     UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK); // Disable transmitter and receiver before configuring
//     UART0->BDH = UART_BDH_SBR((SystemCoreClock / 16) / baud_rate >> 8); // Set baud rate
//     UART0->BDL = UART_BDL_SBR((SystemCoreClock / 16) / baud_rate); // Set baud rate
//     UART0->C1 = 0; // Configure control register 1
//     UART0->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK); // Enable transmitter and receiver
// }

void UART0_Init(uint32_t baud_rate) {
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK; // Enable UART0 clock
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1); // Select the MCGFLLCLK clock as the UART0 clock source

    // Configure PTA1 as UART0_TX
    PORTA->PCR[1] = PORT_PCR_MUX(2); // Select UART0_TX function

    // Configure PTA2 as UART0_RX
    PORTA->PCR[2] = PORT_PCR_MUX(2); // Select UART0_RX function

    UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK); // Disable transmitter and receiver before configuring
    UART0->BDH = 0;
    UART0->BDL = (uint8_t)((SystemCoreClock / (16 * baud_rate)) & 0xFF); // Set baud rate
    UART0->C1 = 0; // Configure control register 1
    UART0->C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK); // Enable transmitter and receiver
}







void initFreeRTOS(void)
{
    xTaskCreate(vPeriodicTask, "Periodic Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1u, NULL);
    xTaskCreate(vPeriodicTask2, "Periodic Task2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1u, NULL);
    // Start FreeRTOS scheduler
    vTaskStartScheduler();

}

void PORTA_IRQHandler(void) {
    encoderISR();
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    PORTA->ISFR |= (1 << ENCODER_A_PIN); // clear interrupt flag
}

float getRPM(void) {
    uint32_t now = SysTick->VAL;
    uint32_t elapsedMicros = (now - lastTickTime) / (SystemCoreClock / 1000000);
    lastTickTime = now;
    float rpm = (float)encoderCount * 150.0F;
    encoderCount = 0u;
    return rpm;
}

int main(void) {
	SystemCoreClockUpdate();
    initEncoder(); // initialize the encoder
    UART0_Init(9600U);
    TPM0_Init();
    initFreeRTOS(); // initialize the encoder

    while (1) {
			counts++;
        // do something with the RPM value, such as printing it to a display or sending it over a communication interface
    }
    return 0;
}
