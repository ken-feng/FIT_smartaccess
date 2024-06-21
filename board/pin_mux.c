/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2020 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */


/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v5.0
processor: MKW38A512xxx4
package_id: MKW38A512VFT4
mcu_data: ksdk2_0
processor_version: 0.0.0
pin_labels:
- {pin_num: '37', pin_signal: PTC1/I2C0_SDA/LPUART0_RTS_b/TPM0_CH2/SPI1_SCK, label: LED_R, identifier: LED_R}
- {pin_num: '4', pin_signal: PTA16/LLWU_P4/SPI1_SOUT/LPUART1_RTS_b/TPM0_CH0, label: LED_G, identifier: LED_G}
- {pin_num: '18', pin_signal: ADC0_SE3/CMP0_IN3/PTB2/LLWU_P9/TPM0_CH0/TPM1_CH0/TPM2_CH0, label: LED_B, identifier: LED_B}
- {pin_num: '19', pin_signal: ADC0_SE2/CMP0_IN4/PTB3/ERCLK32K/LPUART1_RTS_b/TPM0_CH1/CLKOUT/TPM1_CH1/RTC_CLKOUT/TPM2_CH1, label: LED, identifier: LED}
- {pin_num: '23', pin_signal: ADC0_SE4/CMP0_IN2/PTB18/LPUART1_CTS_b/I2C1_SCL/TPM_CLKIN0/TPM0_CH0/NMI_b, label: SW2, identifier: SW2}
- {pin_num: '38', pin_signal: PTC2/LLWU_P10/I2C1_SCL/LPUART0_RX/CMT_IRO/SPI1_SOUT, label: SW3, identifier: SW3}
- {pin_num: '42', pin_signal: PTC6/LLWU_P14/I2C1_SCL/LPUART0_RX/TPM2_CH0, label: LPUART0_RX, identifier: LPUART0_RX}
- {pin_num: '43', pin_signal: PTC7/LLWU_P15/SPI0_PCS2/I2C1_SDA/LPUART0_TX/TPM2_CH1, label: LPUART0_TX, identifier: LPUART0_TX}
- {pin_num: '6', pin_signal: PTA18/LLWU_P6/SPI1_SCK/LPUART1_TX/CAN0_RX/TPM2_CH0, label: LPUART1_TX, identifier: LPUART1_TX}
- {pin_num: '5', pin_signal: PTA17/LLWU_P5/SPI1_SIN/LPUART1_RX/CAN0_TX/TPM_CLKIN1, label: LPUART1_RX, identifier: LPUART1_RX}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
/* for Debug Console Only */
#include "board.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
    BOARD_InitLPUART();
    BOARD_InitLEDs();
    BOARD_InitButtons();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '6', peripheral: LPUART1, signal: TX, pin_signal: PTA18/LLWU_P6/SPI1_SCK/LPUART1_TX/CAN0_RX/TPM2_CH0}
  - {pin_num: '5', peripheral: LPUART1, signal: RX, pin_signal: PTA17/LLWU_P5/SPI1_SIN/LPUART1_RX/CAN0_TX/TPM_CLKIN1}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H

#elif defined __HW_VENUS_EVT1_H
	BOARD_InitFlexcan();
#else
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);

    /* PORTA17 (pin 5) is configured as LPUART1_RX */
    PORT_SetPinMux(BOARD_INITPINS_LPUART1_RX_PORT, BOARD_INITPINS_LPUART1_RX_PIN, kPORT_MuxAlt3);

    /* PORTA18 (pin 6) is configured as LPUART1_TX */
    PORT_SetPinMux(BOARD_INITPINS_LPUART1_TX_PORT, BOARD_INITPINS_LPUART1_TX_PIN, kPORT_MuxAlt3);

    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_LPUART1TXSRC_MASK | SIM_SOPT5_LPUART1RXSRC_MASK)))

                  /* LPUART1 Transmit Data Source Select: LPUART1_TX pin. */
                  | SIM_SOPT5_LPUART1TXSRC(SOPT5_LPUART1TXSRC_0b00)

                  /* LPUART1 Receive Data Source Select: LPUART1_RX pin. */
                  | SIM_SOPT5_LPUART1RXSRC(SOPT5_LPUART1RXSRC_0b0));
 #endif
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitLPUART:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '42', peripheral: LPUART0, signal: RX, pin_signal: PTC6/LLWU_P14/I2C1_SCL/LPUART0_RX/TPM2_CH0}
  - {pin_num: '43', peripheral: LPUART0, signal: TX, pin_signal: PTC7/LLWU_P15/SPI0_PCS2/I2C1_SDA/LPUART0_TX/TPM2_CH1}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLPUART
 * Description   : Configures pin routing for UART Serial Manager and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLPUART(void)
{
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);


	#if defined __HW_VENUS_EVT1_H
    /* PORTC17 (pin 46) is configured as LPUART1_RX */
    PORT_SetPinMux(BOARD_INITLPUART_LPUART1_RX_PORT, BOARD_INITLPUART_LPUART1_RX_PIN, kPORT_MuxAlt8);

    /* PORTC18 (pin 47) is configured as LPUART1_TX */
    PORT_SetPinMux(BOARD_INITLPUART_LPUART1_TX_PORT, BOARD_INITLPUART_LPUART1_TX_PIN, kPORT_MuxAlt8);

    /* PORTC6 (pin 42) is configured as LPUART0_RX */
    PORT_SetPinMux(BOARD_INITLPUART_LPUART0_RX_PORT, BOARD_INITLPUART_LPUART0_RX_PIN, kPORT_MuxAlt4);

    /* PORTC7 (pin 43) is configured as LPUART0_TX */
    PORT_SetPinMux(BOARD_INITLPUART_LPUART0_TX_PORT, BOARD_INITLPUART_LPUART0_TX_PIN, kPORT_MuxAlt4);

    SIM->SOPT5 =
        ((SIM->SOPT5 &
          /* Mask bits to zero which are setting */
          (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK | SIM_SOPT5_LPUART1TXSRC_MASK | SIM_SOPT5_LPUART1RXSRC_MASK)))

         /* LPUART0 Transmit Data Source Select: LPUART0_TX pin. */
         | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_0b00)

         /* LPUART0 Receive Data Source Select: LPUART_RX pin. */
         | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_0b0)

         /* LPUART1 Transmit Data Source Select: LPUART1_TX pin. */
         | SIM_SOPT5_LPUART1TXSRC(SOPT5_LPUART1TXSRC_0b00)

         /* LPUART1 Receive Data Source Select: LPUART1_RX pin. */
         | SIM_SOPT5_LPUART1RXSRC(SOPT5_LPUART1RXSRC_0b0));
	#else
    /* PORTC6 (pin 42) is configured as LPUART0_RX */
    PORT_SetPinMux(BOARD_INITLPUART_LPUART0_RX_PORT, BOARD_INITLPUART_LPUART0_RX_PIN, kPORT_MuxAlt4);

    /* PORTC7 (pin 43) is configured as LPUART0_TX */
    PORT_SetPinMux(BOARD_INITLPUART_LPUART0_TX_PORT, BOARD_INITLPUART_LPUART0_TX_PIN, kPORT_MuxAlt4);

    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK)))

                  /* LPUART0 Transmit Data Source Select: LPUART0_TX pin. */
                  | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_0b00)

                  /* LPUART0 Receive Data Source Select: LPUART_RX pin. */
                  | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_0b0));
	#endif
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitLEDs:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '19', peripheral: GPIOB, signal: 'GPIO, 3', pin_signal: ADC0_SE2/CMP0_IN4/PTB3/ERCLK32K/LPUART1_RTS_b/TPM0_CH1/CLKOUT/TPM1_CH1/RTC_CLKOUT/TPM2_CH1,
    direction: OUTPUT}
  - {pin_num: '37', peripheral: GPIOC, signal: 'GPIO, 1', pin_signal: PTC1/I2C0_SDA/LPUART0_RTS_b/TPM0_CH2/SPI1_SCK, direction: OUTPUT}
  - {pin_num: '18', peripheral: GPIOB, signal: 'GPIO, 2', pin_signal: ADC0_SE3/CMP0_IN3/PTB2/LLWU_P9/TPM0_CH0/TPM1_CH0/TPM2_CH0, direction: OUTPUT}
  - {pin_num: '4', peripheral: GPIOA, signal: 'GPIO, 16', pin_signal: PTA16/LLWU_P4/SPI1_SOUT/LPUART1_RTS_b/TPM0_CH0, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLEDs
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLEDs(void)
{
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
    
#else
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);

    gpio_pin_config_t LED_G_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA16 (pin 4)  */
    GPIO_PinInit(BOARD_INITLEDS_LED_G_GPIO, BOARD_INITLEDS_LED_G_PIN, &LED_G_config);

    gpio_pin_config_t LED_B_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB2 (pin 18)  */
    GPIO_PinInit(BOARD_INITLEDS_LED_B_GPIO, BOARD_INITLEDS_LED_B_PIN, &LED_B_config);

    gpio_pin_config_t LED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB3 (pin 19)  */
    GPIO_PinInit(BOARD_INITLEDS_LED_GPIO, BOARD_INITLEDS_LED_PIN, &LED_config);

    gpio_pin_config_t LED_R_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC1 (pin 37)  */
    GPIO_PinInit(BOARD_INITLEDS_LED_R_GPIO, BOARD_INITLEDS_LED_R_PIN, &LED_R_config);

    /* PORTA16 (pin 4) is configured as PTA16 */
    PORT_SetPinMux(BOARD_INITLEDS_LED_G_PORT, BOARD_INITLEDS_LED_G_PIN, kPORT_MuxAsGpio);

    /* PORTB2 (pin 18) is configured as PTB2 */
    PORT_SetPinMux(BOARD_INITLEDS_LED_B_PORT, BOARD_INITLEDS_LED_B_PIN, kPORT_MuxAsGpio);

    /* PORTB3 (pin 19) is configured as PTB3 */
    PORT_SetPinMux(BOARD_INITLEDS_LED_PORT, BOARD_INITLEDS_LED_PIN, kPORT_MuxAsGpio);

    /* PORTC1 (pin 37) is configured as PTC1 */
    PORT_SetPinMux(BOARD_INITLEDS_LED_R_PORT, BOARD_INITLEDS_LED_R_PIN, kPORT_MuxAsGpio);
#endif
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitButtons:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '23', peripheral: GPIOB, signal: 'GPIO, 18', pin_signal: ADC0_SE4/CMP0_IN2/PTB18/LPUART1_CTS_b/I2C1_SCL/TPM_CLKIN0/TPM0_CH0/NMI_b, direction: INPUT,
    pull_enable: enable}
  - {pin_num: '38', peripheral: GPIOC, signal: 'GPIO, 2', pin_signal: PTC2/LLWU_P10/I2C1_SCL/LPUART0_RX/CMT_IRO/SPI1_SOUT, direction: INPUT, pull_enable: enable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitButtons
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitButtons(void)
{
    //==========================================================================
    // Init GPIO
    //==========================================================================
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port C Clock Gate Control: Clock enabled */
    #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
    
    #else
    CLOCK_EnableClock(kCLOCK_PortC);
    #endif

    //==========================================================================
    // SW1 - Setting
    //==========================================================================
    #ifdef	__HW_BLE_UWB_EVT1_H
    //--------------------------------------------------------------------------
    // Config
    //--------------------------------------------------------------------------
    gpio_pin_config_t SW1_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };

    GPIO_PinInit(GPIOB, 1, &SW1_config);                // Initialize GPIO functionality on pin PTB1
    //--------------------------------------------------------------------------
    // Pin Setting
    //--------------------------------------------------------------------------
    PORT_SetPinMux(PORTB, 1, kPORT_MuxAsGpio);          // PORTB2 is configured as PTB1

    PORTC->PCR[1] = ((PORTC->PCR[1] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin. */
                     | (uint32_t)(PORT_PCR_PE_MASK));
    #endif

    
    //==========================================================================
    // SW2 - Setting
    //==========================================================================
    #ifdef	__HW_BLE_UWB_EVT1_H
    //--------------------------------------------------------------------------
    // Config
    //--------------------------------------------------------------------------
    gpio_pin_config_t SW2_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };

    GPIO_PinInit(GPIOB, 2, &SW2_config);                // Initialize GPIO functionality on pin PTB2
    //--------------------------------------------------------------------------
    // Pin Setting
    //--------------------------------------------------------------------------
    PORT_SetPinMux(PORTB, 2, kPORT_MuxAsGpio);          // PORTB2 is configured as PTB2

    PORTB->PCR[2] = ((PORTB->PCR[2] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin. */
                      | (uint32_t)(PORT_PCR_PE_MASK));
    
    #elif defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H

    #else
    //--------------------------------------------------------------------------
    // Config
    //--------------------------------------------------------------------------
    gpio_pin_config_t SW2_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB18 (pin 23)  */
    GPIO_PinInit(BOARD_INITBUTTONS_SW2_GPIO, BOARD_INITBUTTONS_SW2_PIN, &SW2_config);
    //--------------------------------------------------------------------------
    // Pin Setting
    //--------------------------------------------------------------------------
    /* PORTB18 (pin 23) is configured as PTB18 */
    PORT_SetPinMux(BOARD_INITBUTTONS_SW2_PORT, BOARD_INITBUTTONS_SW2_PIN, kPORT_MuxAsGpio);

    PORTB->PCR[18] = ((PORTB->PCR[18] &
                       /* Mask bits to zero which are setting */
                       (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                      /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin. */
                      | (uint32_t)(PORT_PCR_PE_MASK));

    #endif

    //==========================================================================
    // SW3 - Setting
    //==========================================================================
	#if defined __HW_VENUS_EVT1_H

	#else
    //--------------------------------------------------------------------------
    // Config
    //--------------------------------------------------------------------------
    gpio_pin_config_t SW3_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC2 (pin 38)  */
    GPIO_PinInit(BOARD_INITBUTTONS_SW3_GPIO, BOARD_INITBUTTONS_SW3_PIN, &SW3_config);
    //--------------------------------------------------------------------------
    // Pin Setting
    //--------------------------------------------------------------------------
    /* PORTC2 (pin 38) is configured as PTC2 */
    PORT_SetPinMux(BOARD_INITBUTTONS_SW3_PORT, BOARD_INITBUTTONS_SW3_PIN, kPORT_MuxAsGpio);

    //Modify (Ken):NXP-V0001 NO.1 -20240304
    #ifdef __HW_BLE_UWB_FOB_H
    PORTB->PCR[BOARD_INITBUTTONS_SW3_PIN] = ((PORTB->PCR[BOARD_INITBUTTONS_SW3_PIN] &
    #else
    PORTC->PCR[2] = ((PORTC->PCR[2] &
    #endif
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin. */
                     | (uint32_t)(PORT_PCR_PE_MASK));
	#endif
}
                      
                      

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI0Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI .
 *
 * END ****************************************************************************************************************/
//Modify (Ken):NXP-V0001 NO.1 -20240304
void BOARD_InitSPI0Pins(void)
{
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_VENUS_EVT1_H

#else
    /* Enable Clocks: Port C Gate Control */
    CLOCK_EnableClock(kCLOCK_PortC);
    
    /* SPI0_SCK: PORTC16 (pin 45) */
    PORT_SetPinMux(BOARD_SPI0_CLK_PORT, BOARD_SPI0_CLK_PIN, kPORT_MuxAlt2);
    /* SPI0_SOUT: PORTC17 (pin 46) */
    PORT_SetPinMux(BOARD_SPI0_MISO_PORT, BOARD_SPI0_MISO_PIN, kPORT_MuxAlt2);
    /* SPI0_SIN: PORTC18 (pin 47) */
    PORT_SetPinMux(BOARD_SPI0_MOSI_PORT, BOARD_SPI0_MOSI_PIN, kPORT_MuxAlt2);
    /* SPI0_CS: PORTC19 (pin 48) */
    PORT_SetPinMux(BOARD_SPI0_CS_PORT, BOARD_SPI0_CS_PIN, kPORT_MuxAlt2);      
#endif
}


/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI1Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI.
 *                 
 * END ****************************************************************************************************************/
//Modify (Ken):NXP-V0001 NO.1 -20240304
void BOARD_InitSPI1Pins(void)
{
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_VENUS_EVT1_H
    /* Enable Clocks: Port A Gate Control */
    CLOCK_EnableClock(kCLOCK_PortA);
    
    /* SPI1_SOUT: PORTA16 (pin 4) */
    PORT_SetPinMux(BOARD_SPI1_MISO_PORT, BOARD_SPI1_MISO_PIN, kPORT_MuxAlt2);
    /* SPI1_SIN: PORTA17 (pin 5) */
    PORT_SetPinMux(BOARD_SPI1_MOSI_PORT, BOARD_SPI1_MOSI_PIN, kPORT_MuxAlt2);
    /* SPI1_SCK: PORTA18 (pin 6) */
    PORT_SetPinMux(BOARD_SPI1_CLK_PORT, BOARD_SPI1_CLK_PIN, kPORT_MuxAlt2);
    /* SPI0_CS: PORTA19 (pin 7) */
    //PORT_SetPinMux(PORTC, PIN19_IDX, kPORT_MuxAlt2);      
#endif
}

                      
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitFlexcan
 * Description   : FlexCAN Initialize Pins for CAN Communication
 *
 * END ****************************************************************************************************************/
//Modify (Ken):NXP-V0001 NO.1 -20240304
void BOARD_InitFlexcan(void)
{
#if FLEXCAN_SUPPORT
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* PORTC3 (pin 39) is configured as CAN0_TX */
    PORT_SetPinMux(BOARD_INITFLEXCAN_CAN0_TX_PORT, BOARD_INITFLEXCAN_CAN0_TX_PIN, kPORT_MuxAlt9);

    /* PORTC4 (pin 40) is configured as CAN0_RX */
    PORT_SetPinMux(BOARD_INITFLEXCAN_CAN0_RX_PORT, BOARD_INITFLEXCAN_CAN0_RX_PIN, kPORT_MuxAlt9);
    
    //=================================================================================================================
    // Add STB pin (Normal mode=low; Sleep mode=high)
    //=================================================================================================================
    #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(BOARD_INITFLEXCAN_CAN0_STB_PORT, BOARD_INITFLEXCAN_CAN0_STB_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(BOARD_INITFLEXCAN_CAN0_STB_GPIO, BOARD_INITFLEXCAN_CAN0_STB_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});
//    PORT_SetPinMux(BOARD_INITFLEXCAN_CAN0_STB_PORT, BOARD_INITFLEXCAN_CAN0_STB_PIN, kPORT_MuxAsGpio);
//    port_pin_config_t stbPinconfig =
//    {
//        kPORT_PullDown,         //Frank add STB : Normal = PullUp;Standby = PullDown
//        kPORT_FastSlewRate,
//        kPORT_PassiveFilterEnable,
//        kPORT_LowDriveStrength,
//        kPORT_MuxAsGpio
//    };
//    PORT_SetPinConfig(BOARD_INITFLEXCAN_CAN0_STB_PORT, BOARD_INITFLEXCAN_CAN0_STB_PIN, &stbPinconfig);
//
//    gpio_pin_config_t stbPinGPIOConfig =
//    {
//        .outputLogic = 0u,
//        .pinDirection = kGPIO_DigitalInput
//    };
//    GPIO_PinInit(BOARD_INITFLEXCAN_CAN0_STB_GPIO, BOARD_INITFLEXCAN_CAN0_STB_PIN, &stbPinGPIOConfig);
    #endif
#endif
}
                                              
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitRanger4Pins
 * Description   : Init UWB module connected Pins
 *
 * END ****************************************************************************************************************/
void BOARD_InitRanger4Pins(void)
{
#if UWB_FEATURE_SUPPORT    
    //==========================================================================
    // SPI Init
    //==========================================================================
    #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
    BOARD_InitSPI1Pins();
    #else
    BOARD_InitSPI0Pins();
    #endif
    
    //==========================================================================
    // Enable Clocks
    //==========================================================================
    /* Enable Clocks: Port A Gate Control */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Enable Clocks: Port C Gate Control */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Enable Clocks: Port C Gate Control */
    CLOCK_EnableClock(kCLOCK_PortC);
    //==========================================================================
    /*** Manual chip select (CS): PORTA19 output***/
    //==========================================================================
    PORT_SetPinMux(BOARD_UWB_CS_PORT, BOARD_UWB_CS_PIN, kPORT_MuxAsGpio);
    port_pin_config_t csPinconfig =
    {
        kPORT_PullDisable,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_CS_PORT, BOARD_UWB_CS_PIN, &csPinconfig);
    gpio_pin_config_t csPinGPIOConfig =
    {
        .outputLogic = 1u, 
        .pinDirection = kGPIO_DigitalOutput
    };
    GPIO_PinInit(BOARD_UWB_CS_GPIO, BOARD_UWB_CS_PIN, &csPinGPIOConfig);
    
    //==========================================================================
    /*** Ready pin(RDY): PORTC5 input***/
    //==========================================================================
    PORT_SetPinMux(BOARD_UWB_RDY_PORT, BOARD_UWB_RDY_PIN, kPORT_MuxAsGpio);
    port_pin_config_t readyPinconfig =
    {
        kPORT_PullUp,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterEnable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_RDY_PORT, BOARD_UWB_RDY_PIN, &readyPinconfig);
    
    gpio_pin_config_t readyPinGPIOConfig =
    {
        .outputLogic = 1u,
        .pinDirection = kGPIO_DigitalInput
    };
    GPIO_PinInit(BOARD_UWB_RDY_GPIO, BOARD_UWB_RDY_PIN, &readyPinGPIOConfig);
    
    //==========================================================================
    /*** Reset pin(RST): PORTC19 output***/
    //==========================================================================
    PORT_SetPinMux(BOARD_UWB_RST_PORT, BOARD_UWB_RST_PIN, kPORT_MuxAsGpio);
    port_pin_config_t rstPinconfig =
    {
        #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
        kPORT_PullUp,
        #else
        kPORT_PullDisable,
        #endif
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_RST_PORT, BOARD_UWB_RST_PIN, &rstPinconfig);
    gpio_pin_config_t rstPinGPIOConfig =
    {
        .outputLogic = 1u,
        .pinDirection = kGPIO_DigitalOutput
    };
    GPIO_PinInit(BOARD_UWB_RST_GPIO, BOARD_UWB_RST_PIN, &rstPinGPIOConfig);
    
    //==========================================================================
    /*** Interrupt pin: PORTC6 input***/
    //==========================================================================
    // This port not use now. Because this port also be used by LpUart0, so 
    // it should be initlized after LpUart0 was initlized.  
    PORT_SetPinMux(BOARD_UWB_INt_PORT, BOARD_UWB_INT_PIN, kPORT_MuxAsGpio);
    port_pin_config_t intPinconfig =
    {
        kPORT_PullUp,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterEnable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_INt_PORT, BOARD_UWB_INT_PIN, &intPinconfig);

    gpio_pin_config_t intPinGPIOConfig =
    {
        .outputLogic = 1u,
        .pinDirection = kGPIO_DigitalInput
    };
    GPIO_PinInit(BOARD_UWB_INT_GPIO, BOARD_UWB_INT_PIN, &intPinGPIOConfig);
#endif
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
