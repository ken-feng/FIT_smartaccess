/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2020 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */


#include "GPIO_Adapter.h"
#include "gpio_pins.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

gpioInputPinConfig_t switchPins[] = {
  
  
//Modify (Ken):NXP-V0001 NO.1 -20240325
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H
    {
        .gpioPort = gpioPort_B_c,       
        #ifdef __HW_BLE_UWB_FOB_H
        .gpioPin = 0,
        #else
        .gpioPin = 3,
        #endif
        .pullSelect = pinPull_Up_c,
        .interruptSelect = pinInt_FallingEdge_c
    },
#elif defined  __HW_BLE_UWB_EVT1_H && defined __FIT_BUTTON_SETTING_HIGH_TRIGGER_H
    {
        .gpioPort = gpioPort_B_c,       
        .gpioPin = 0,
        .pullSelect = pinPull_Down_c,
        .interruptSelect = pinInt_RisingEdge_c
    },
    {
        .gpioPort = gpioPort_B_c,       
        .gpioPin = 2,
        .pullSelect = pinPull_Down_c,
        .interruptSelect = pinInt_RisingEdge_c
    },
    {
        .gpioPort = gpioPort_B_c,       
        .gpioPin = 1,
        .pullSelect = pinPull_Down_c,
        .interruptSelect = pinInt_RisingEdge_c
    }
    #else
    {
        .gpioPort = gpioPort_B_c,
        .gpioPin = 18,
        .pullSelect = pinPull_Up_c,
        .interruptSelect = pinInt_FallingEdge_c
    },
    {
        .gpioPort = gpioPort_C_c,
        .gpioPin = 2,
        .pullSelect = pinPull_Up_c,
        .interruptSelect = pinInt_FallingEdge_c
    }
    #endif
};

/* Declare Output GPIO pins */
gpioOutputPinConfig_t ledPins[] = {
    {
        .gpioPort = gpioPort_B_c,
        .gpioPin = 3,
        .outputLogic = 0,
        .slewRate = pinSlewRate_Slow_c,
        .driveStrength = pinDriveStrength_Low_c
    },
    {
        .gpioPort = gpioPort_C_c,
        .gpioPin = 1,
        .outputLogic = 0,
        .slewRate = pinSlewRate_Slow_c,
        .driveStrength = pinDriveStrength_Low_c
    },
    {
        .gpioPort = gpioPort_A_c,
        .gpioPin = 16,
        .outputLogic = 0,
        .slewRate = pinSlewRate_Slow_c,
        .driveStrength = pinDriveStrength_Low_c
    },
    {
        .gpioPort = gpioPort_B_c,
        .gpioPin = 2,
        .outputLogic = 0,
        .slewRate = pinSlewRate_Slow_c,
        .driveStrength = pinDriveStrength_Low_c
    }
};

//Modify (Ken):NXP-V0001 NO.4 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
gpioInputPinConfig_t Ranger4InputPins[] = 
{
    /***** Interrupt pin: PORTC6 (pin 42)  *****/
    {
        #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
        .gpioPort = gpioPort_B_c,
        .gpioPin = 18,
        .pullSelect = pinPull_Up_c,
        .interruptSelect = pinInt_FallingEdge_c
        #else
        .gpioPort = gpioPort_C_c,
        .gpioPin = 6,
        .pullSelect = pinPull_Up_c,
        .interruptSelect = pinInt_FallingEdge_c
        #endif
    }
};
#endif

