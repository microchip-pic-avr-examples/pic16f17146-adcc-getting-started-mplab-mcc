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

#ifndef ADCC_LABS_H
#define	ADCC_LABS_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>
#include "../mcc_generated_files/adcc/adcc.h"

/**
  Section: Macro Declarations
 */

//for flag settings
#define SET (1)
#define CLEAR (0)
#define ZERO (0)

//Auto trigger intervals timer period value
#define PERIOD_1S (242) //Timer period  1 second for terminal view
#define PERIOD_100MS (24) //Timer period 100 ms for graph view

//constants for light intensity calculation
#define INTENSITY_CONVERSION_FACTOR (2.4414)

//defines for sending data to data visualizer graph
#define START_OF_FRAME (0x5F)
#define END_OF_FRAME (0xA0)

//enum for switch state
typedef enum
{
    NOT_PRESSED,
    PRESSED
} swState_t;

//enum for lab number
typedef enum
{
    NONE,
    BASIC,
    ACCUMULATE,
    AVERAGE,
    BURST_AVERAGE,
    LOW_PASS_FILTER,
    DIFFERENTIAL,
    SLEEP_MODE,
    READ_TEMPERATURE,
    MAX_LAB_COUNT
} lab_t;

/**
  Section: Variable Declaration
 */
extern uint16_t lightIntensity;
extern uint8_t lightIntensityLowByte;
extern uint8_t lightIntensityHighByte;

extern volatile bool adcConversionDoneFlag;
extern volatile bool adcThresholdFlag;

/**
  Section: Function Declarations
 */
void CalculateLightIntensity(uint16_t lightCount);

void ADCC_UserInterruptHandler(void);
void ADCC_UserThresholdInterruptHandler(void);
void TMR4_UserInterruptHandler(void);
void AdccLabsApplication(void);
void AdccBasicMode(bool initRequired);
void AdccAccumulateMode(bool initRequired);
void AdccAverageMode(bool initRequired);
void AdccBurstAverageMode(bool initRequired);
void AdccLowPassFilterMode(bool initRequired);
void AdccReadTemperatureIndicator(bool initRequired);
void AdccInSleepMode(bool initRequired);
void AdccDifferentialMode(bool initRequired);
    
void ADCC_Initialize_Basic_mode(void);
void ADCC_Initialize_Accumulate_Mode(void);
void ADCC_Initialize_Average_Mode(void);
void ADCC_Initialize_Burst_Average_Mode(void);
void ADCC_Initialize_Continuous_Sampling(void);
void ADCC_Initialize_TemperatureIndicator(void);
void ADCC_Initialize_Low_Pass_Filter_Mode(void);
void ADCC_Initialize_Sleep_Mode(void);   
void ADCC_Initialize_Differential_mode(void);     

int16_t ADCC_GetSingleConversion_Differential(adcc_channel_t posChannel,adcc_channel_t negChannel);
 
#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* ADCC_LABS_H */

