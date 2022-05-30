/**
  ADCC Read Temperature Indicator Lab Source File

  Company:
    Microchip Technology Inc.

  File Name:
    adcc_read_temperature.c

  Summary:
    This is the source file for the ADCC Read Temperature Indicator lab

  Description:
     This source file contains the code to demonstrate the ADCC reading Temperature Indicator.

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
//Flash address where temperature indicator parameters are stored

#define ADDR_TSHR1 (0x8115) // Gain
#define ADDR_TSHR2 (0x8116) // Adc reading at 90 dec C
#define ADDR_TSHR3 (0x8117) // Offset

// Constants for C to F conversion
#define C_TO_F_MULTIPLIER (1.8)
#define C_TO_F_CONSTANT (32)

// Uncomment to print the gain, offset and ADC reading at 90 degree during debugging
#define DEBUG_TEMPERATURE 

int24_t volatile celsiusValue = 0;
int16_t volatile fahrenheitValue = 0;
static int16_t gain = 0;
static int16_t offset = 0;
static uint16_t adcReading90deg = 0;

/**
  Section: Function Declarations
 */
void CalculateTemperature(uint16_t adcAverageResult);
uint16_t DIA_ReadWord(uint16_t diaAddr);

/*
                             Application    
 */

/**
  @Summary
    Performs the ADCC Read Temperature Indicator module Lab.
  @Description
 Read the analog channel connected to internal temperature indicator module using ADCC in Burst Average mode.
 * Display the Temperature in degree Celsius and Fahrenheit on the terminal window.
  @Preconditions
    SYSTEM_Initialize() functions should have been called before calling this function.
  @Param
    none
  @Returns
    None
 */
void AdccReadTemperatureIndicator(bool initRequired)
{
    uint16_t adcFilter = 0;


    if (initRequired == true)
    {
        LED0_SetHigh(); // Turn OFF LED
        printf("\r\n\n\nLab 7: ADCC read temperature indicator");
        printf("\r\n\nPress switch S1 to go to the next lab.");
        // Read Flash locations where temperature indicator parameters are stored        
        gain = (int16_t) (DIA_ReadWord(ADDR_TSHR1));
        adcReading90deg = DIA_ReadWord(ADDR_TSHR2); // ADC reading for 90 degree C
        offset = (int16_t) (DIA_ReadWord(ADDR_TSHR3));

#ifdef DEBUG_TEMPERATURE
        printf("\r\n Gain = %d", gain);
        printf("\r\n offset = %d", offset);
        printf("\r\n adcReading90deg = %d", adcReading90deg);
#endif

        // Check if there is ADC conversion done from previous lab if so ignore the results
        if (adcConversionDoneFlag == true)
        {
            adcConversionDoneFlag = false;
        }

        ADCC_Initialize_TemperatureIndicator();

        ADPCH = channel_Temp; // To select the temperature Indicator channel 
        Timer2_PeriodCountSet(PERIOD_1S); // Set auto trigger interval as 1s for terminal view
        Timer2_Start();
    }

    if (adcConversionDoneFlag == true)
    {
        adcFilter = ADCC_GetFilterValue(); // Read average ADC value
        CalculateTemperature(adcFilter); // Calculate temperature from average ADC reading   

#ifdef DEBUG_TEMPERATURE
        printf("\r\n\n adcFilter=%d", adcFilter);
#endif
        printf("\r\n\nTemperature in Celsius = %ld degree C", (int32_t) celsiusValue);
        printf("\r\nTemperature in Fahrenheit = %d degree F", fahrenheitValue);

        adcConversionDoneFlag = false;
    }
}

/**
This function initializes ADC for reading temperature indicator module,
 *  ADCC mode burst average, average of 16 ADC results, Voltage reference for ADC = FVR 2.048V,
 * ADC acquisition time = 25 us, auto trigger using timer 2 overflow, Enable ADC conversion done interrupt
@param none 
\returns none 
 */
void ADCC_Initialize_TemperatureIndicator(void)
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
    ADPCH = channel_Temp;
    //NCH ANA0;
    ADNCH = 0x00;
    // ADACQ 25; 
    ADACQL = 0x19;
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
    // ADCALC Filtered value vs setpoint; ADTMD ADERR >= ADLTH; ADSOI ADGO not cleared; 
    ADCON3 = 0x52;
    // ADMATH registers not updated; 
    ADSTAT = 0x00;
    // ADNREF VSS; ADPREF FVR; 
    ADREF = 0x03;
    // ADACT TMR2; 
    ADACT = 0x04;
    // ADCS FOSC/2; 
    ADCLK = 0x00;
    //GO_nDONE undefined; ADIC single-ended mode; ADFM right justified; ADCS FOSC; ADCONT disabled; ADON enabled; 
    ADCON0 = 0x84;


    // Clear the ADC interrupt flag
    PIR6bits.ADIF = 0;
    // Enabling ADCC interrupt.
    PIE6bits.ADIE = 1;

    // Clear the ADC Threshold interrupt flag
    PIR6bits.ADTIF = 0;
    // Disabling ADCC threshold interrupt.
    PIE6bits.ADTIE = 0;


}

/**
This function converts measured ADC result into equivalent temperature in Celsius and Fahrenheit
 * temperature=(((ADC result*gain)/256)+offset)/10
@param adcAverageResult - measured ADC result by reading temperature indicator module 
\returns none 
 */
void CalculateTemperature(uint16_t adcAverageResult)
{
    celsiusValue = adcAverageResult;
    celsiusValue = (int24_t) (celsiusValue) * (int24_t) (gain); // Multiply the ADC Result by Gain and store the result in a signed variable
    celsiusValue = celsiusValue / 256; // Divide (ADC Result * Gain) by 256
    celsiusValue = celsiusValue + offset; // Add (Offset) to the result
    celsiusValue = celsiusValue / 10; // Divide the result by 10
    fahrenheitValue = (int16_t) ((celsiusValue * C_TO_F_MULTIPLIER) + C_TO_F_CONSTANT); // Convert to Fahrenheit
}
/**
 End of File
 */