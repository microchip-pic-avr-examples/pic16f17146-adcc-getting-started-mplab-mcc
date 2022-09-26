/**
  adc basic mode Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    adcc_basic_mode.c

  Summary:
    This is the source file for the ADC basic mode lab

  Description:
    This source file contains the code to demonstrate the ADC in basic mode.
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

//Comment below #defines for terminal view of the results in Basic mode
#define GRAPH_BASIC

#define DELAY_100MS (100) // Wait before reading ADC till update interval of 100 ms in graph view
#define DELAY_1S (1000) // Wait before reading ADC till update interval of 1 Second in terminal view
/*
                             Application    
 */

/**
  @Summary
    Performs the Basic Analog to Digital Conversion Lab.
  @Description
 Read the analog channel connected to Ambient Light sensor using ADC in basic mode.
 * Display the ADC results and corresponding Light intensity on the Graph or terminal window.
  @Preconditions
    SYSTEM_Initialize() functions should have been called before calling this function.
  @Param
    none 
  @Returns
    None
 */
void AdccBasicMode(bool initRequired)
{
    uint16_t adcResult;

    if (initRequired == true)
    {
        printf("\r\n\n\nLab 1: ADCC in Basic mode");
        printf("\r\n\nPress switch SW0 to go to the next lab.");
        ADCC_Initialize_Basic_mode();
    }
    adcResult = ADCC_GetSingleConversion(Ambient_AN); // Read the ADC result from the analog channel corresponding to ambient light 
    CalculateLightIntensity(adcResult); // Calculate light intensity from ADC result

#ifdef GRAPH_BASIC           
    while (!(UART1.IsTxReady()));
    UART1.Write(START_OF_FRAME); // Command sent to the Data Visualizer, 0x5F = Start                                              
    while (!(UART1.IsTxReady()));
    UART1.Write(ZERO); // ADCC conversion Count
    while (!(UART1.IsTxReady()));
    UART1.Write(ADRESL); // ADCC Result low byte as visualizer reads low byte first 
    while (!(UART1.IsTxReady()));
    UART1.Write(ADRESH); // ADCC Result high byte
    while (!(UART1.IsTxReady()));
    UART1.Write(ZERO); // ADCC Accumulator low byte 
    while (!(UART1.IsTxReady()));
    UART1.Write(ZERO); // ADCC Accumulator high byte
    while (!(UART1.IsTxReady()));
    UART1.Write(ZERO); // ADCC Filter low byte  
    while (!(UART1.IsTxReady()));
    UART1.Write(ZERO); // ADCC Filter high byte  
    while (!(UART1.IsTxReady()));
    UART1.Write(lightIntensityLowByte); // Light Intensity low byte
    while (!(UART1.IsTxReady()));
    UART1.Write(lightIntensityHighByte); // Light Intensity high byte
    while (!(UART1.IsTxReady()));
    UART1.Write(END_OF_FRAME); // Command sent to the Data Visualizer, 0xA0 = End      
    __delay_ms(DELAY_100MS); // Wait before reading ADCC till update interval of 100 ms in graph view
#else
    printf("\r\n\nADCC result in basic mode is %d", adcResult);
    printf("\r\nLight Intensity = %d lx", lightIntensity);
    __delay_ms(DELAY_1S); //wait before reading ADCC till update interval of 1 Second in terminal view
#endif
}

/**
  @Summary
    This function initializes ADCC in basic mode
  @Description
 * ADCC mode: basic, 
 * ADCC Clock = FOSC/4, 
 * Voltage reference for ADCC = FVR (2.048V)
  @Preconditions
    none
  @Param
    none
  @Returns
    None
 */
void ADCC_Initialize_Basic_mode(void)
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
    // ADRPT 0; 
    ADRPT = 0x00;
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
    // ADCRS 1; ADMD Basic_mode; ADACLR disabled; ADPSIS RES; 
    ADCON2 = 0x10;
    // ADCALC First derivative of Single measurement; ADTMD disabled; ADSOI ADGO not cleared; 
    ADCON3 = 0x00;
    // ADMATH registers not updated; 
    ADSTAT = 0x00;
    //ADPREF FVR; 
    ADREF = 0x3;
    // ADACT disabled; 
    ADACT = 0x00;
    //ADCCS FOSC/4; 
    ADCLK = 0x1;
    //GO_nDONE undefined; ADIC single-ended mode; ADFM right justified; ADCS FOSC; ADCONT disabled; ADON enabled; 
    ADCON0 = 0x84;

    // Clear the ADC interrupt flag
    PIR6bits.ADIF = 0;
    // Disabling ADCC interrupt.
    PIE6bits.ADIE = 0;

    // Clear the ADC Threshold interrupt flag
    PIR6bits.ADTIF = 0;
    // Disabling ADCC threshold interrupt.
    PIE6bits.ADTIE = 0;
}
/**
 End of File
 */