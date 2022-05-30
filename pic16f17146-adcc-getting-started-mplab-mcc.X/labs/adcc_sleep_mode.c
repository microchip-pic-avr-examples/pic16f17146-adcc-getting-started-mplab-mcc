/**
  adc accumulate mode Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    adc_accumulate_mode.c

  Summary:
    This is the source file for the ADCC accumulate mode lab

  Description:
    This source file contains the code to demonstrate the ADCC in accumulate mode.
 */

/*
© [2022] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
 */

/**
  Section: Included Files
 */
#include "../mcc_generated_files/system/system.h"
#include "adcc_labs.h"

//#defines for setting ADC threshold interrupts in sleep mode
#define INTERRUPT_ON_LOWER_THR_SOI (0x59)
#define INTERRUPT_ON_UPPER_THR_SOI (0x5E)

/*
                             Application    
 */

/**
  @Summary
    Performs the ADCC operation in Sleep mode Lab.
  @Description
 Read the analog channel connected to Ambient Light sensor when MCU is in sleep mode.
ADC works in sleep mode and will not display any result on terminal window if ADC results are greater than lower threshold and less than upper threshold
if ADC results are above Upper threshold then MCU wakes up and displays the warning that the light intensity is above upper threshold of 5000 lx
if ADC results are below Lower threshold then MCU wakes up and displays the warning that the light intensity is below lower threshold of 200 lx  
 * @Preconditions
    SYSTEM_Initialize() functions should have been called before calling this function.
  @Param
    none
  @Returns
    None
 */
void AdccInSleepMode(bool initRequired)
{
    uint16_t adcFilter = 0;

    if (initRequired == true)
    {
        printf("\r\n\n\nLab 6: ADCC in Sleep mode");
        printf("\r\n\nPress switch S1 to go to the next lab.");
        CPUDOZEbits.IDLEN = CLEAR; // Clear idle enable bit so than MCU will go to sleep mode when SLEEP instruction is executed
        ADCC_Initialize_Sleep_Mode();
    }

    printf("\r\n\nEntering Sleep mode");
    while (!(EUSART1_IsTxDone())); // Wait till UART finishes printing the message 

    PMD3bits.UART1MD = SET; // Disable UART setting power mode disable bit    
    ADCC_StartConversion(Ambient_AN); // Start ADC conversion in continuous sampling mode
    LED0_SetHigh(); // Turning OFF LED before going to sleep mode
    SLEEP();
    NOP();
    LED0_SetLow(); // Turning ON LED after waking up
    PMD3bits.UART1MD = CLEAR; // Enable UART clearing power mode disable bit 
    EUSART1_Initialize(); // Initialize UART peripheral after waking up from sleep mode

    if (adcThresholdFlag == true)
    {
        if (ADCC_HasErrorCrossedUpperThreshold()) // Check if ADC readings are above upper threshold
        {
            printf("\r\n\nLight intensity is more than upper threshold 500 ADC count.");
            printf("\r\nADC will check for the lower threshold 25 ADC count now.");

            // Set the ADC threshold interrupt for checking if the light intensity goes below lower threshold
            // ADC error calculation:Filtered value vs setpoint;
            // Threshold interrupt if ADC error voltage is less than lower threshold,
            // Stop ADC continuous mode on interrupt  
            ADCON3 = INTERRUPT_ON_LOWER_THR_SOI;
        }
        if (ADCC_HasErrorCrossedLowerThreshold()) // Check if ADC readings are below lower threshold
        {
            printf("\r\n\nLight intensity is less than lower threshold 25 ADC count.");
            printf("\r\nADC will check for the upper threshold 500 ADC count now.");

            // Set the ADC threshold interrupt for checking if the light intensity goes above upper threshold
            // ADC error calculation:Filtered value vs setpoint;
            // Threshold interrupt if ADC error voltage is more than upper threshold,
            // Stop ADC continuous mode on interrupt
            ADCON3 = INTERRUPT_ON_UPPER_THR_SOI;
        }
        adcFilter = ADCC_GetFilterValue(); // Read average ADC value        
        CalculateLightIntensity(adcFilter); // Calculate light intensity from average ADC reading
        printf("\r\n\nADC filtered value is %d", adcFilter);
        printf("\r\nLight Intensity = %d lx.", lightIntensity);
        adcThresholdFlag = false;
    }
}

/**
This function initializes ADC for ADC conversion in Sleep mode,
 *  ADCC mode burst average, average of 32 ADC results, Voltage reference for ADC = VDD,
 *Continuous sampling enabled, Stop on Interrupt, Enable ADC Threshold interrupt, 
 * lower Threshold=205=500lx, upper threshold =820=2000lx
@param none 
\returns none 
 */
void ADCC_Initialize_Sleep_Mode(void)
{
    // set the ADCC to the options selected in the User Interface
    // ADLTH 25; 
    ADLTHL = 0x19;
    // ADLTH 0; 
    ADLTHH = 0x00;
    // ADUTH 244; 
    ADUTHL = 0xF4;
    // ADUTH 1; 
    ADUTHH = 0x01;
    // ADSTPT 0; 
    ADSTPTL = 0x00;
    // ADSTPT 0; 
    ADSTPTH = 0x00;
    // ADACC 0; 
    ADACCU = 0x00;
    // ADRPT 32; 
    ADRPT = 0x20;
    // PCH ANC2; 
    ADPCH = 0x12;
    //NCH ANA0;
    ADNCH = 0x00;
    // ADACQ 0; 
    ADACQL = 0x00;
    // ADACQ 0; 
    ADACQH = 0x00;
    // CAP Additional uC disabled; 
    ADCAP = 0x00;
    // ADPRE 0; 
    ADPREL = 0x00;
    // ADPRE 0; 
    ADPREH = 0x00;
    // ADDSEN disabled; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss; 
    ADCON1 = 0x00;
    // ADCRS 5; ADMD Burst_average_mode; ADACLR disabled; ADPSIS RES; 
    ADCON2 = 0x53;
    // ADCALC Filtered value vs setpoint; ADTMD ADERR < ADLTH or ADERR > ADUTH; ADSOI ADGO is cleared; 
    ADCON3 = 0x5C;
    // ADMATH registers not updated; 
    ADSTAT = 0x00;
    // ADNREF VSS; ADPREF VDD; 
    ADREF = 0x00;
    // ADACT disabled; 
    ADACT = 0x00;
    // ADCS FOSC/2; 
    ADCLK = 0x00;
    // ADGO stop; ADFM right; ADON enabled; ADCS Frc; ADCONT enabled; 
    ADCON0 = 0xD4;

    // Clear the ADC interrupt flag
    PIR6bits.ADIF = 0;
    // Disabling ADCC interrupt.
    PIE6bits.ADIE = 0;
    // Clear the ADC Threshold interrupt flag
    PIR6bits.ADTIF = 0;
    // Enabling ADCC threshold interrupt.
    PIE6bits.ADTIE = 1;
}

/**
 End of File
 */