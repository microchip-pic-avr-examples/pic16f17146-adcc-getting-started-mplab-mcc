/**
  ADCC Average mode Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    adcc_average_mode.c

  Summary:
    This is the source file for the ADCC Average mode lab

  Description:
    This source file contains the code to demonstrate the ADCC in Average mode.
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

// Comment below #define for terminal view of the results in Average modes
#define GRAPH_AVERAGE

#ifdef GRAPH_AVERAGE 
#define  TIMER_PERIOD  (24)
#else
#define  TIMER_PERIOD  (242)
#endif
/*
                             Application    
 */

/**
  @Summary
    Performs the ADCC Average mode Lab.
  @Description
 * Read the analog channel connected to Ambient Light sensor using ADCC in Average mode.
 * Display the ADCC conversion count, ADC results, ADC accumulator, ADCC filtered value and 
 * corresponding Light intensity on graph or terminal window. * 
  @Preconditions
    SYSTEM_Initialize() functions should have been called before calling this function.
  @Param
    none
  @Returns
    None
 */
void AdccAverageMode(bool initRequired)
{
    uint16_t adcResult = 0;
    uint24_t adcAccumulator = 0;
    uint16_t adcFilter = 0;

    if (initRequired == true)
    {
        printf("\r\n\nLab 3: ADCC in Average mode");
        printf("\r\n\nPress switch SW0 to go to the next lab.");
        Timer2_PeriodCountSet(TIMER_PERIOD); // Set ADC auto trigger interval for terminal/graphical view
        Timer2_Start();
        ADCC_Initialize_Average_Mode();
        ADPCH = Ambient_AN; // Select the analog channel to read ambient light
    }
    if (adcConversionDoneFlag == true)
    {
#ifdef GRAPH_AVERAGE           
        while (!(UART1.IsTxReady()));
        UART1.Write(START_OF_FRAME); // Command sent to the Data Visualizer, 0x5F = Start                                              
        while (!(UART1.IsTxReady()));
        UART1.Write(ADCNT); // ADCC conversion Count
        while (!(UART1.IsTxReady()));
        UART1.Write(ADRESL); // ADCC Result low byte as visualizer reads low byte first 
        while (!(UART1.IsTxReady()));
        UART1.Write(ADRESH); // ADCC Result high byte
        while (!(UART1.IsTxReady()));
        UART1.Write(ADACCL); // ADCC Accumulator low byte 
        while (!(UART1.IsTxReady()));
        UART1.Write(ADACCH); // ADCC Accumulator high byte
        while (!(UART1.IsTxReady()));
        UART1.Write(ADFLTRL); // ADCC Filter low byte  
        while (!(UART1.IsTxReady()));
        UART1.Write(ADFLTRH); // ADCC Filter high byte  
        while (!(UART1.IsTxReady()));
        UART1.Write(lightIntensityLowByte); // Light Intensity low byte
        while (!(UART1.IsTxReady()));
        UART1.Write(lightIntensityHighByte); // Light Intensity high byte
        while (!(UART1.IsTxReady()));
        UART1.Write(END_OF_FRAME); // Command sent to the Data Visualizer, 0xA0 = End 
#else
        adcResult = ADCC_GetConversionResult();
        adcAccumulator = ADCC_GetAccumulatorValue();
        adcFilter = ADCC_GetFilterValue();
        printf("\r\n\nADC Count= %d", ADCNT);
        printf("\r\nADC Result=%d", adcResult);
        printf("\r\nADC Accumulator= %lu", (uint32_t) adcAccumulator);
        printf("\r\nADC Filter=%d", adcFilter);
#endif   
        if (ADCNT == ADRPT)// If ADCC conversion count is equal to number of repetitions, use ADCC filter value (average) for calculating the light intensity  
        {
            adcFilter = ADCC_GetFilterValue(); // Read average ADCC value
            CalculateLightIntensity(adcFilter); // Calculate light intensity from average ADCC reading
#ifndef GRAPH_AVERAGE
            printf("\r\n\n\nLab3: Average mode. Average of %d ADC results is %d", ADRPT, adcFilter);
            printf("\r\nLight Intensity = %d lx", lightIntensity);
#endif
        }
        adcConversionDoneFlag = false;
    }
}

/**
  @Summary
    This function initializes ADCC in Average mode
  @Description
 * ADCC mode: Average, ADC result right shift count = 4, ADC repeat count=16 for average of 16 ADCC result counts, 
 * Voltage reference for ADCC = FVR (2.048V),
 * auto trigger using timer 2 overflow, Enable ADCC conversion done interrupt
  @Preconditions
    none
  @Param
    none
  @Returns
    None
 */
void ADCC_Initialize_Average_Mode(void)
{
    // set the ADCC to the options selected in the User Interface
    // ADLTH 0; 
    ADLTHL = 0x00;
    // ADLTH 0; 
    ADLTHH = 0x00;
    // ADUTH 0; 
    ADUTHL = 0x00;
    // ADUTH 0; 
    ADUTHH = 0x00;
    // ADSTPT 0; 
    ADSTPTL = 0x00;
    // ADSTPT 0; 
    ADSTPTH = 0x00;
    // ADACC 0; 
    ADACCU = 0x00;
    // ADRPT 16; 
    ADRPT = 0x10;
    // PCH ANA0; 
    ADPCH = 0x00;
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
    // ADCRS 4; ADMD Average_mode; ADACLR disabled; ADPSIS RES; 
    ADCON2 = 0x42;
    // ADCALC Filtered value vs setpoint; ADTMD ADERR >= ADLTH; ADSOI ADGO not cleared; 
    ADCON3 = 0x52;
    // ADMATH registers not updated; 
    ADSTAT = 0x00;
    //ADPREF FVR; 
    ADREF = 0x3;
    // ADACT TMR2; 
    ADACT = 0x04;
    //ADCCS FOSC/4; 
    ADCLK = 0x1;
    //GO_nDONE undefined; ADIC single-ended mode; ADFM right justified; ADCS FOSC; ADCONT disabled; ADON enabled; 
    ADCON0 = 0x84;

    // Clear the ADC interrupt flag
    PIR6bits.ADIF = 0;
    // Enabling ADCC interrupt.
    PIE6bits.ADIE = 1;

    // Clear the ADC Threshold interrupt flag
    PIR6bits.ADTIF = 0;
    //  Disabling ADCC threshold interrupt.
    PIE6bits.ADTIE = 0;
}
/**
 End of File
 */