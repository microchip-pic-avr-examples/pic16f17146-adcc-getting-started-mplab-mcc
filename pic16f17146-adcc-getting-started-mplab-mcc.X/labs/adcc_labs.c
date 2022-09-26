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
#include <xc.h>
#include "adcc_labs.h"
#include "../mcc_generated_files/system/system.h"

#define MASK_LOWER_BYTE (0x00FF)
#define SIGN_BIT (0x2000) // Sign bit position for 14 bit data
#define EXTEND_SIGN_BIT (0xC000)

/**
  Section: Variable Definitions
 */
lab_t lab = NONE;
volatile swState_t switchEvent = NOT_PRESSED;

uint16_t lightIntensity = 0;
uint8_t lightIntensityLowByte = 0;
uint8_t lightIntensityHighByte = 0;

volatile bool initializationRequired = false;
volatile bool adcConversionDoneFlag = false;
volatile bool adcThresholdFlag = false;

/**
This function converts measured ADC count into equivalent light intensity
@param lightCount - measured ADC count for ambient light 
\returns none 
 */
void CalculateLightIntensity(uint16_t adccCount)
{
    float lightIntensityFloat;
    lightIntensityFloat = (float) (INTENSITY_CONVERSION_FACTOR * adccCount);
    lightIntensity = (uint16_t) lightIntensityFloat;
    lightIntensityLowByte = MASK_LOWER_BYTE & lightIntensity;
    lightIntensityHighByte = MASK_LOWER_BYTE & (lightIntensity >> 8);
}

/**
* User interrupt handler for ADC conversion complete interrupt. 
 * Interrupt is generated when ADC conversion is complete.
 */
void ADCC_UserInterruptHandler(void)
{
    adcConversionDoneFlag = true;
}

/**
User interrupt handler for ADC threshold interrupt. 
 * Interrupt is generated when ADC result is out of range.
 * Lower threshold = 82 corresponding light intensity 200 lx
 * upper Threshold = 802 corresponding light intensity 5000 lx.
 */
void ADCC_UserThresholdInterruptHandler(void)
{
    adcThresholdFlag = true;
}

/**
* User interrupt handler for Timer 4 overflow interrupt. 
 * Interrupt is generated when timer overflows after switch press event.
 */
void TMR4_UserInterruptHandler(void)
{
    if(SW0_GetValue() == 0)
    {
        switchEvent = PRESSED;
    }  
}

void AdccLabsApplication(void)
{
    initializationRequired = false;
    if (switchEvent == PRESSED) // Check for switch press event if switch press then go to next lab
    {
        Timer2_Stop();
        lab++; // Increment labName to execute next lab
        if (lab == MAX_LAB_COUNT)
        {
            lab = BASIC;
        }
        initializationRequired = true;
        switchEvent = NOT_PRESSED;
    }

    switch (lab)
    {
    case BASIC: AdccBasicMode(initializationRequired);
        break;
    case ACCUMULATE: AdccAccumulateMode(initializationRequired);
        break;
    case AVERAGE: AdccAverageMode(initializationRequired);
        break;
    case BURST_AVERAGE: AdccBurstAverageMode(initializationRequired);
        break;
    case LOW_PASS_FILTER: AdccLowPassFilterMode(initializationRequired);
        break;
    case DIFFERENTIAL: AdccDifferentialMode(initializationRequired);
        break;
    case SLEEP_MODE: AdccInSleepMode(initializationRequired);
        break;
    case READ_TEMPERATURE: AdccReadTemperatureIndicator(initializationRequired);
        break;
    default:
        break;
    }
}

/**
 * Function to read Device information area
 * The data read from DIA is 14-bit in length and sign bit extension is taken care
 */
uint16_t DIA_ReadWord(uint16_t diaAddr)
{
    uint16_t dia14bitData = 0;
    uint16_t dia16bitData = 0;
    uint8_t GIEBitValue = INTCONbits.GIE; // Save interrupt enable

    INTCONbits.GIE = 0; // Disable interrupts
    NVMADRL = (diaAddr & 0x00FF);
    NVMADRH = ((diaAddr & 0xFF00) >> 8);

    NVMCON1bits.NVMREGS = 1; // Select Device information area
    NVMCON1bits.RD = 1; // Initiate Read
    NOP();
    NOP();
    INTCONbits.GIE = GIEBitValue; // Restore interrupt enable

    dia14bitData = ((uint16_t) ((NVMDATH << 8) | NVMDATL));

    // Check if the 14 bit data is negative i.e. if 14th bit is '1' 
    if (dia14bitData & SIGN_BIT)
    {
        dia16bitData = dia14bitData | EXTEND_SIGN_BIT; // Extend the sign bit to form 16 bit number
    }
    else
    {
        dia16bitData = dia14bitData;
    }
    return (dia16bitData);
}

