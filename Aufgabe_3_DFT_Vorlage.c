#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"

// Praepozessor-Makros
#define SAMPLERATE 44000
#define FENSTERBREITE 440

//Funktionen-Deklarationen
void adcIntHandler(void);
void setup(void);
// hier nach Bedarf noch weitere Funktionsdeklarationen einfuegen
void discreteFourierTransfomation(void) {

    uint32_t real[FENSTERBREITE];
    uint32_t imaginary[FENSTERBREITE];

    for (k = 0 ; k < FENSTERBREITE ; k++)
    {
        for (n = 0 ; n < FENSTERBREITE ; n++) {
            real[k] += bufferSample[n] * cosf(n * k * DoublePi / FENSTERBREITE);
        }

        for (n = 0 ; n < FENSTERBREITE ; n++) {
            imaginary[k] += bufferSample[n] * sinf(n * k * DoublePi / FENSTERBREITE);
        }

        freqBereich[k * frequenzaufloesung] += sqrt(real[k]^2 + imaginary[k]^2);
    }
}



// globale Variablen
const float DoublePi = 6.283185308;
int32_t bufferSample[FENSTERBREITE];
uint32_t sampleIndex = 0;
uint32_t frequenzaufloesung = SAMPLERATE / FENSTERBREITE;
float freqBereich[44000];
// hier nach Bedarf noch weitere globale Variablen einfuegen

void main(void){ // nicht veraendern!! Bitte Code in adcIntHandler einfuegen
    setup();
    while(1){}
}

void setup(void){//konfiguriert den Mikrocontroller

    // konfiguriere SystemClock
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    uint32_t period = SysCtlClockGet()/SAMPLERATE;

    // aktiviere Peripherie
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // aktiviere Gleitkommazahlen-Modul
    FPUEnable();
    FPUStackingEnable();
    FPULazyStackingEnable();
    FPUFlushToZeroModeSet(FPU_FLUSH_TO_ZERO_EN);

    // konfiguriere GPIO
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    // konfiguriere Timer
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, period - 1);
    TimerControlTrigger(TIMER0_BASE,TIMER_A,true);
    TimerEnable(TIMER0_BASE,TIMER_A);

    // konfiguriere ADC
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_RATE_FULL,1);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE,3);
    ADCIntRegister(ADC0_BASE,3,adcIntHandler);
    ADCIntEnable(ADC0_BASE,3);

}

void adcIntHandler(void){

    ADCSequenceDataGet(ADC0_BASE,3,&bufferSample[i]);

    i++;

    if (sampleIndex == 339) {
        i = 0;
    }

    discreteFourierTransfomation();

    // am Ende von adcIntHandler, Interrupt-Flag loeschen
    ADCIntClear(ADC0_BASE,3);
}
