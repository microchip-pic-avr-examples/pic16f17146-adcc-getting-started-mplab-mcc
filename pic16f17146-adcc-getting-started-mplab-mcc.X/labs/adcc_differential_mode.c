/**
  adc differential mode Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    adcc_differential_mode.c

  Summary:
    This is the source file for the ADC Differential Mode lab

  Description:
    This source file contains the code to demonstrate the ADC in differential mode.
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

// Comment below #defines for terminal view of the results in Differential mode
#define GRAPH_DIFFERENTIAL

#define DELAY_100MS (100) // Wait before reading ADC till update interval of 100 ms in graph view
#define DELAY_1S (1000) // Wait before reading ADC till update interval of 1 Second in terminal view
/*
                             Application    
 */

/**
  @Summary
    Performs the Differential mode Analog to Digital Conversion Lab.
  @Description
 * Read the analog channel connected to Ambient Light and FVR in single ended mode as well as differential mode.
 * Display the ADCC results  on the Graph or terminal window.
  @Preconditions
    SYSTEM_Initialize() functions should have been called before calling this function.
  @Param
    none 
  @Returns
    None
 */
void AdccDifferentialMode(bool initRequired)
{
    int16_t adcResultDiff = 0;
    uint16_t adcResultSingle_pos, adcResultSingle_neg;

    if (initRequired == true)
    {
        printf("\r\n\n\nLab 6: ADCC in Differential mode");
        printf("\r\n\nPress switch SW0 to go to the next lab.");
    }
    
    ADCC_Initialize_Basic_mode();
    adcResultSingle_pos = ADCC_GetSingleConversion(Ambient_AN); // Read the ADCC result from the analog channel corresponding to ambient light 
    adcResultSingle_neg = ADCC_GetSingleConversion(channel_FVR_Buffer2); // Read the ADCC result from the analog channel corresponding to FVR Buffer 2 
    ADCC_Initialize_Differential_mode();
    adcResultDiff = ADCC_GetSingleConversion_Differential(Ambient_AN,channel_FVR_Buffer2);
    
#ifdef GRAPH_DIFFERENTIAL           
    while (!(UART1.IsTxReady()));
    UART1.Write(START_OF_FRAME); // Command sent to the Data Visualizer, 0x5F = Start                                              
    while (!(UART1.IsTxReady()));
    UART1.Write((uint8_t)(adcResultSingle_pos)); // Send low byte
    while (!(UART1.IsTxReady()));
    UART1.Write((uint8_t)(adcResultSingle_pos >> 8)); // Send high byte
    while (!(UART1.IsTxReady()));
    UART1.Write((uint8_t)(adcResultSingle_neg)); // Send low byte
    while (!(UART1.IsTxReady()));
    UART1.Write((uint8_t)(adcResultSingle_neg >> 8)); // Send high byte
    while (!(UART1.IsTxReady()));
    UART1.Write((uint8_t)adcResultDiff); // Send low byte
    while (!(UART1.IsTxReady()));
    UART1.Write((uint8_t)(adcResultDiff >> 8)); // Send high byte
    while (!(UART1.IsTxReady()));
    UART1.Write(END_OF_FRAME); // Command sent to the Data Visualizer, 0xA0 = End   
    __delay_ms(DELAY_100MS); // Wait before reading ADC till update interval of 100 ms in graph view
#else
    printf("\r\n\nADCC Single-ended result of Positive Channel (Ambient light sensor) is %u", adcResultSingle_pos);
    printf("\r\nADCC Single-ended result of Negative Channel (FVR) is %u", adcResultSingle_neg);
    printf("\r\nADCC Differential result is %d", adcResultDiff);
    __delay_ms(DELAY_1S); //wait before reading ADC till update interval of 1 Second in terminal view
#endif
}

/**
  @Summary
    This function initializes ADCC in Differential mode with basic mode of computation
  @Description
 * ADCC mode: Differential, 
 * ADCC Clock = FOSC/4, 
 * Voltage reference for ADCC = FVR (2.048V)
  @Preconditions
    none
  @Param
    none
  @Returns
    None
 */
void ADCC_Initialize_Differential_mode(void)
{
  // set the ADCC to the options selected in the User Interface
    //ADLTHL 0; 
    ADLTHL = 0x0;
    //ADLTHH 0; 
    ADLTHH = 0x0;
    //ADUTHL 0; 
    ADUTHL = 0x0;
    //ADUTHH 0; 
    ADUTHH = 0x0;
    //ADSTPTL 0; 
    ADSTPTL = 0x0;
    //ADSTPTH 0; 
    ADSTPTH = 0x0;
    //ADACCU 0x0; 
    ADACCU = 0x0;
    //ADRPT 0; 
    ADRPT = 0x0;
    //ADCHS ANA0; 
    ADPCH = 0x0;
    //ADCHS ANA0; 
    ADNCH = 0x0;
    //ADACQL 0; 
    ADACQL = 0x0;
    //ADACQH 0; 
    ADACQH = 0x0;
    //ADCAP Additional uC disabled; 
    ADCAP = 0x0;
    //ADPREL 0; 
    ADPREL = 0x0;
    //ADPREH 0; 
    ADPREH = 0x0;
    //CGA0 disabled; CGA1 disabled; CGA2 disabled; CGA4 disabled; CGA5 disabled; 
    ADCG1A = 0x0;
    //CGB4 disabled; CGB5 disabled; CGB6 disabled; CGB7 disabled; 
    ADCG1B = 0x0;
    //CGC0 disabled; CGC1 disabled; CGC2 disabled; CGC3 disabled; CGC4 disabled; CGC5 disabled; CGC6 disabled; CGC7 disabled; 
    ADCG1C = 0x0;
    //ADDSEN disabled; ADPCSC internal sampling capacitor and ext i/o pin; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss; 
    ADCON1 = 0x0;
    // ADCRS 1; ADMD Basic_mode; ADACLR disabled; ADPSIS RES; 
    ADCON2 = 0x10;
   // ADCALC First derivative of Single measurement; ADTMD disabled; ADSOI ADGO not cleared; 
    ADCON3 = 0x00;
    //ADMATH registers not updated; 
    ADSTAT = 0x0;
    //ADPREF FVR; 
    ADREF = 0x3;
    // ADACT disabled; 
    ADACT = 0x00;
    //ADCCS FOSC/4; 
    ADCLK = 0x1;
    //GO_nDONE undefined; ADIC differential mode; ADFM right justified, two's compliment; ADCS FOSC; ADCONT disabled; ADON enabled; 
    ADCON0 = 0x86;
    
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
  @Summary
    This function returns ADCC single conversion with differential mode
  @Description
 * ADCC mode: Differential, 
 * ADCC Clock = FOSC/4, 
 * Voltage reference for ADCC = FVR (2.048)
  @Preconditions
    none
  @Param
    posChannel - ADCC positive channel
    negChannel - ADCC negative channel
  @Returns
    ADCC result
 */
int16_t ADCC_GetSingleConversion_Differential(adcc_channel_t posChannel,adcc_channel_t negChannel)
{
    // Select the A/D channels
    ADPCH = posChannel;
    ADNCH = negChannel;

    // Turn on the ADC module
    ADCON0bits.ADON = 1;
    
    // Disable the continuous mode.
    ADCON0bits.ADCONT = 0;

    // Start the conversion
    ADCON0bits.ADGO = 1;

    // Wait for the conversion to finish
    while (ADCON0bits.ADGO)
    {
    }
    
    // Conversion finished, return the result
    return ((int16_t)((ADRESH << 8) + ADRESL));
}
/**
 End of File
 */