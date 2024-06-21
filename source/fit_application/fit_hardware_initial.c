//***************************************************** 
// The KW38 serial head file define 
// Define the main program Ram section  
//***************************************************** 
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fit_hardware_initial.h"

//****************************************************
//Modify (Ken):NXP-V0001 NO.2 -20240326
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __NXP_EVB_TEST_PIN_H || defined __HW_VENUS_EVT1_H


/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_Init
 * Description   : Calls debug console initialization functions
 * 
 *
 * END ****************************************************************************************************************/
void Init_APP_BOARD(void)
{
    //==========================================================================
    // [ Init GPIO ] - 
    //==========================================================================
    #ifdef __HW_BLE_UWB_FOB_H
    Init_APP_BOARD_GPIO();
    #elif defined   __NXP_EVB_TEST_PIN_H
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortC);
    //Test Pin PTC7
    PORT_SetPinMux(PORTC, 7u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 7u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});
    //Test Pin PTA18
    PORT_SetPinMux(PORTA, 18u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 18u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});
	#elif defined __HW_VENUS_EVT1_H
    CLOCK_EnableClock(kCLOCK_PortB);
	#if defined __HW_TEST_PIN_H
    //To Trigger RFIC (output)
    PORT_SetPinMux(PORTB, 1u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 1u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U});
    //To Trigger vehicle (output)
    PORT_SetPinMux(PORTB, 2u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 2u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});
    //To Trigger vehicle (input)
    PORT_SetPinMux(PORTB, 3u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 3u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});
	#else
    //To Trigger RFIC (output)
    PORT_SetPinMux(PORTB, 1u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 1u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U});
    //To Trigger vehicle (output)
    PORT_SetPinMux(PORTB, 2u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 2u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});
    //To Trigger vehicle (input)
//    PORT_SetPinMux(PORTB, 3u, kPORT_MuxAsGpio);
//    GPIO_PinInit(GPIOB, 3u, &(gpio_pin_config_t){kGPIO_DigitalInput, 0U});
	#endif

    #endif
    //==========================================================================
    // [ Init RFIC UART ] -
    //==========================================================================
    #ifdef __FIT_RFIC_UART_H

    #endif
    //==========================================================================
    // [ Init SE ] - 
    //==========================================================================
    //Modify (Ken):NXP-V0001 NO.5 -20240319
    #ifdef __FIT_SECURITY_CHIP_H
    Init_APP_SECURITY_CHIP();
    #endif
    //==========================================================================
    // [ Init CAN ] -
    //==========================================================================
	#ifdef  __FIT_CAN_H
    BOARD_InitFlexcan();
	#endif
}
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : Init_APP_BOARD_GPIO
 * Description   : Calls debug console initialization functions
 * 
 *
 * END ****************************************************************************************************************/
void Init_APP_BOARD_GPIO(void)
{
    //==========================================================================
    // [ ID GPIO ] - Output
    //==========================================================================
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    
    #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H
    port_pin_config_t ledPinconfig =
    {
        kPORT_PullDisable,
        kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
//Modify (Ken):NXP-V0001 NO.6 -20240327
//    PORT_SetPinMux(PORTB, 0u, kPORT_MuxAsGpio);                                 // LED_R/Button
//    PORT_SetPinConfig(PORTB, 0u, &ledPinconfig);
//    GPIO_PinInit(GPIOB, 0u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U});     

    PORT_SetPinMux(PORTB, 1u, kPORT_MuxAsGpio);                                 // LED_G/Button
    PORT_SetPinConfig(PORTB, 1u, &ledPinconfig);
    GPIO_PinInit(GPIOB, 1u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});

    PORT_SetPinMux(PORTB, 2u, kPORT_MuxAsGpio);                                 // LED_B/Button
    PORT_SetPinConfig(PORTB, 2u, &ledPinconfig);
    GPIO_PinInit(GPIOB, 2u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U});


	#elif defined __HW_VENUS_EVT1_H


    #else
    PORT_SetPinMux(PORTB, 2u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 2u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U});
    #endif
    
    //==========================================================================
    // [ ID GPIO ] - Output
    //==========================================================================
    #if (gKBD_KeysCount_c==0)
    
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    
    PORT_SetPinMux(PORTB, 3u, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 3u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U});
    #endif
}


/* FUNCTION ************************************************************************************************************
 *
 * Function Name : Init_APP_SECURITY_CHIP
 * Description   : Calls debug console initialization functions
 * 
 *
 * END ****************************************************************************************************************/
#ifdef __FIT_SECURITY_CHIP_H
void Init_APP_SECURITY_CHIP(void)
{
    //==========================================================================
    // [ SPI0 ] - Init
    //==========================================================================
    BOARD_InitSPI0Pins();
    
    //==========================================================================
    // [ GPIO ] - Enable Pin for SE chip
    //==========================================================================
    port_pin_config_t enPinconfig =
    {
        kPORT_PullDisable,
        kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinMux(BOARD_SECURITY_EN_PORT, BOARD_SECURITY_EN_PIN, kPORT_MuxAsGpio);
    PORT_SetPinConfig(BOARD_SECURITY_EN_PORT, BOARD_SECURITY_EN_PIN, &enPinconfig);
    GPIO_PinInit(BOARD_SECURITY_EN_GPIO, BOARD_SECURITY_EN_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U});         //turn off
}
#endif

#endif




