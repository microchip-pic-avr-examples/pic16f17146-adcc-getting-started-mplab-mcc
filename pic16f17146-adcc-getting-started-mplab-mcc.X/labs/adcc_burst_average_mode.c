/**
  ADCC Burst Average mode Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    adcc_burst_average_mode.c

  Summary:
    This is the source file for the ADCC Burst Average mode lab

  Description:
    This source file contains the code to demonstrate the ADCC in Burst Average mode.
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

// Comment below #defines for graphical view of the results in Burst Average mode
#define GRAPH_BURST


#ifdef GRAPH_BURST 
#define  TIMER_PERIOD (24)
#else
#define  TIMER_PERIOD  (242)
#endif
/*
                             Application    
 */

/**
  @Summary
    Performs the ADCC Burst Average mode Lab.
  @Description
 Read the analog channel connected to Ambient Light sensor using ADCC in Burst Average mode.
 * Display the ADC filtered value and corresponding Light intensity on graph or terminal window. * 
  @Preconditions
    SYSTEM_Initialize() functions should have been called before calling this function.
  @Param
    none
  @Returns
    None
 */
void AdccBurstAverageMode(bool initRequired)
{
    uint16_t adcFilter = 0;

    if (initRequired == true)
    {
        printf("\r\n\n\nLab 4: ADCC in Burst Average mode");
        printf("\r\n\nPress switch SW0 to go to the next lab.");
        Timer2_PeriodCountSet(TIMER_PERIOD); // Set ADC auto trigger interval for terminal/graphical view
        Timer2_Start();
        ADCC_Initialize_Burst_Average_Mode();
        ADPCH = Ambient_AN; // Select the analog channel to read ambient light
    }

    if (adcThresholdFlag == true)
    {
        adcFilter = ADCC_GetFilterValue(); // Read average ADC value
        CalculateLightIntensity(adcFilter); // Calculate light intensity from average ADC reading
#ifdef GRAPH_BURST 
        while (!(UART1.IsTxReady()));
        UART1.Write(START_OF_FRAME); // Command sent to the Data Visualizer, 0x5F = Start                                              
        while (!(UART1.IsTxReady()));
        UART1.Write(ZERO); // ADC conversion Count
        while (!(UART1.IsTxReady()));
        UART1.Write(ZERO); // ADC Result low byte as visualizer reads low byte first 
        while (!(UART1.IsTxReady()));
        UART1.Write(ZERO); // ADC Result high byte
        while (!(UART1.IsTxReady()));
        UART1.Write(ZERO); // ADC Accumulator low byte 
        while (!(UART1.IsTxReady()));
        UART1.Write(ZERO); // ADC Accumulator high byte
        while (!(UART1.IsTxReady()));
        UART1.Write(ADFLTRL); // ADC Filter low byte  
        while (!(UART1.IsTxReady()));
        UART1.Write(ADFLTRH); // ADC Filter high byte  
        while (!(UART1.IsTxReady()));
        UART1.Write(lightIntensityLowByte); // Light Intensity low byte
        while (!(UART1.IsTxReady()));
        UART1.Write(lightIntensityHighByte); // Light Intensity high byte
        while (!(UART1.IsTxReady()));
        UART1.Write(END_OF_FRAME); // Command sent to the Data Visualizer, 0xA0 = End 
#else //Terminal view
        printf("\r\n\nBurst Average value of %d ADC result counts is %d", ADRPT, adcFilter);
        printf("\r\nLight Intensity = %d lx", lightIntensity);
#endif
        adcThresholdFlag = false;
    }

}

/**
This function initializes ADCC in Burst Average mode
 * ADCC mode: burst average, 
 * average of 16 ADC results, 
 * Voltage reference for ADC = VDD,
 * ADC auto trigger using timer 2 overflow, 
 * Enable ADC conversion done interrupt
@param none 
\returns none 
 */
void ADCC_Initialize_Burst_Average_Mode(void)
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
    // ADCRS 4; ADMD Burst_average_mode; ADACLR disabled; ADPSIS RES; 
    ADCON2 = 0x43;
    // ADCALC Filtered value vs setpoint; ADTMD enabled; ADSOI ADGO not cleared; 
    ADCON3 = 0x57;
    // ADMATH registers not updated; 
    ADSTAT = 0x00;
    // ADNREF VSS; ADPREF VDD; 
    ADREF = 0x00;
    // ADACT TMR2; 
    ADACT = 0x04;
    // ADCS FOSC/2; 
    ADCLK = 0x00;
    //GO_nDONE undefined; ADIC single-ended mode; ADFM right justified; ADCS FOSC; ADCONT disabled; ADON enabled; 
    ADCON0 = 0x84;

    // Clear the ADC interrupt flag
    PIR6bits.ADIF = 0;
    //Enabling ADCC interrupt.
    PIE6bits.ADIE = 0;

    // Clear the ADC Threshold interrupt flag
    PIR6bits.ADTIF = 0;
    //  Disabling ADCC threshold interrupt.
    PIE6bits.ADTIE = 1;

}
/**
 End of File
 */