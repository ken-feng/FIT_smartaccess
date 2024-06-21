/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2020 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */


#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void);

//=====================================================================================================================
// [ Configuration UART ] - UART1
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H


#elif defined __HW_VENUS_EVT1_H
#define SOPT5_LPUART1RXSRC_0b0 0x00u  /*!<@brief LPUART1 Receive Data Source Select: LPUART1_RX pin */
#define SOPT5_LPUART1TXSRC_0b00 0x00u /*!<@brief LPUART1 Transmit Data Source Select: LPUART1_TX pin */

/*! @name PORTA18 (number 6), LPUART1_TX
  @{ */
#define BOARD_INITLPUART_LPUART1_TX_PORT PORTC /*!<@brief PORT device name: PORTA */
#define BOARD_INITLPUART_LPUART1_TX_PIN 18U    /*!<@brief PORTA pin index: 18 */
                                             /* @} */

/*! @name PORTA17 (number 5), LPUART1_RX
  @{ */
#define BOARD_INITLPUART_LPUART1_RX_PORT PORTC /*!<@brief PORT device name: PORTA */
#define BOARD_INITLPUART_LPUART1_RX_PIN 17U    /*!<@brief PORTA pin index: 17 */

#else
#define SOPT5_LPUART1RXSRC_0b0 0x00u  /*!<@brief LPUART1 Receive Data Source Select: LPUART1_RX pin */
#define SOPT5_LPUART1TXSRC_0b00 0x00u /*!<@brief LPUART1 Transmit Data Source Select: LPUART1_TX pin */

/*! @name PORTA18 (number 6), LPUART1_TX
  @{ */
#define BOARD_INITPINS_LPUART1_TX_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_LPUART1_TX_PIN 18U    /*!<@brief PORTA pin index: 18 */
                                             /* @} */

/*! @name PORTA17 (number 5), LPUART1_RX
  @{ */
#define BOARD_INITPINS_LPUART1_RX_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_LPUART1_RX_PIN 17U    /*!<@brief PORTA pin index: 17 */
                                             /* @} */
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void);

//=====================================================================================================================
// [ Configuration UART ] - UART0
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
      #define SOPT5_LPUART0RXSRC_0b0 0x00u  /*!<@brief LPUART0 Receive Data Source Select: LPUART_RX pin */
      #define SOPT5_LPUART0TXSRC_0b00 0x00u /*!<@brief LPUART0 Transmit Data Source Select: LPUART0_TX pin */

      /*! @name PORTC6 (number 42), LPUART0_RX
        @{ */
      #define BOARD_INITLPUART_LPUART0_RX_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_INITLPUART_LPUART0_RX_PIN 6U     /*!<@brief PORTC pin index: 6 */
                                                     /* @} */

      /*! @name PORTC7 (number 43), LPUART0_TX
        @{ */
      #define BOARD_INITLPUART_LPUART0_TX_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_INITLPUART_LPUART0_TX_PIN 7U     /*!<@brief PORTC pin index: 7 */
                                                     /* @} */
#else
#define SOPT5_LPUART0RXSRC_0b0 0x00u  /*!<@brief LPUART0 Receive Data Source Select: LPUART_RX pin */
#define SOPT5_LPUART0TXSRC_0b00 0x00u /*!<@brief LPUART0 Transmit Data Source Select: LPUART0_TX pin */

/*! @name PORTC6 (number 42), LPUART0_RX
  @{ */
#define BOARD_INITLPUART_LPUART0_RX_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITLPUART_LPUART0_RX_PIN 6U     /*!<@brief PORTC pin index: 6 */
                                               /* @} */

/*! @name PORTC7 (number 43), LPUART0_TX
  @{ */
#define BOARD_INITLPUART_LPUART0_TX_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITLPUART_LPUART0_TX_PIN 7U     /*!<@brief PORTC pin index: 7 */
                                               /* @} */
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLPUART
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLPUART(void);

//=====================================================================================================================
// [ Configuration LED ] - 
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H

#else
/*! @name PORTB3 (number 19), LED
  @{ */
#define BOARD_INITLEDS_LED_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITLEDS_LED_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITLEDS_LED_PIN 3U     /*!<@brief PORTB pin index: 3 */
                                      /* @} */

/*! @name PORTC1 (number 37), LED_R
  @{ */
#define BOARD_INITLEDS_LED_R_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITLEDS_LED_R_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITLEDS_LED_R_PIN 1U     /*!<@brief PORTC pin index: 1 */
                                        /* @} */

/*! @name PORTB2 (number 18), LED_B
  @{ */
#define BOARD_INITLEDS_LED_B_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITLEDS_LED_B_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITLEDS_LED_B_PIN 2U     /*!<@brief PORTB pin index: 2 */
                                        /* @} */

/*! @name PORTA16 (number 4), LED_G
  @{ */
#define BOARD_INITLEDS_LED_G_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITLEDS_LED_G_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITLEDS_LED_G_PIN 16U    /*!<@brief PORTA pin index: 16 */
                                        /* @} */
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLEDs
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLEDs(void);

//=====================================================================================================================
// [ Configuration Switch ] - 
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_FOB_H
      /*! @name PORTB3 (number xx), SW3
        @{ */
      #define BOARD_INITBUTTONS_SW3_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_INITBUTTONS_SW3_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_INITBUTTONS_SW3_PIN  0U     /*!<@brief PORTB pin index: 3 */
                                               /* @} */
#elif defined __HW_BLE_UWB_G2_H
      /*! @name PORTB3 (number xx), SW3
        @{ */
      #define BOARD_INITBUTTONS_SW3_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_INITBUTTONS_SW3_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_INITBUTTONS_SW3_PIN  3U     /*!<@brief PORTB pin index: 3 */
                                               /* @} */
#elif defined __HW_BLE_UWB_EVT1_H
      /*! @name PORTB3 (number xx), SW1
        @{ */
      #define BOARD_INITBUTTONS_SW3_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_INITBUTTONS_SW3_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_INITBUTTONS_SW3_PIN  0U     /*!<@brief PORTB pin index: 3 */
                                               /* @} */
      /*! @name PORTB3 (number xx), SW2
        @{ */
      #define BOARD_INITBUTTONS_SW3_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_INITBUTTONS_SW3_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_INITBUTTONS_SW3_PIN  1U     /*!<@brief PORTB pin index: 3 */
                                               /* @} */
      /*! @name PORTB3 (number xx), SW3
        @{ */
      #define BOARD_INITBUTTONS_SW3_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_INITBUTTONS_SW3_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_INITBUTTONS_SW3_PIN  2U     /*!<@brief PORTB pin index: 3 */
                                               /* @} */

#elif defined __HW_VENUS_EVT1_H


#else
/*! @name PORTB18 (number 23), SW2
  @{ */
#define BOARD_INITBUTTONS_SW2_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITBUTTONS_SW2_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITBUTTONS_SW2_PIN 18U    /*!<@brief PORTB pin index: 18 */
                                         /* @} */

/*! @name PORTC2 (number 38), SW3
  @{ */
#define BOARD_INITBUTTONS_SW3_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITBUTTONS_SW3_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITBUTTONS_SW3_PIN 2U     /*!<@brief PORTC pin index: 2 */
                                         /* @} */
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitButtons
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitButtons(void);

//=====================================================================================================================
// [ Configuration SPI0 ] - 
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_EVT1_H || defined __HW_VENUS_EVT1_H

#else
/*! @name PORTC16 (number 45), SPI0_CLK */
#define BOARD_SPI0_CLK_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_SPI0_CLK_PIN  16U    /*!<@brief PORTC pin index: 16 */

/*! @name PORTC17 (number 46), SPI0_MISO */
#define BOARD_SPI0_MISO_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_SPI0_MISO_PIN 17U     /*!<@brief PORTC pin index: 17 */

/*! @name PORTC18 (number 47), SPI0_MOSI */
#define BOARD_SPI0_MOSI_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_SPI0_MOSI_PIN 18U     /*!<@brief PORTC pin index: 18 */

/*! @name PORTC18 (number 47), SPI0_MOSI */
#define BOARD_SPI0_CS_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_SPI0_CS_PIN  19U     /*!<@brief PORTC pin index: 18 */
#endif
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI0Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI split mode
 *
 * END ****************************************************************************************************************/      
void BOARD_InitSPI0Pins(void);

//=====================================================================================================================
// [ Configuration SPI1 ] - 
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
/*! @name PORTA16 (number 4), SPI1_SOUT */
#define BOARD_SPI1_MISO_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_SPI1_MISO_PIN  16U    /*!<@brief PORTA pin index: 16 */

/*! @name PORTA17 (number 5), SPI1_SIN */
#define BOARD_SPI1_MOSI_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_SPI1_MOSI_PIN 17U     /*!<@brief PORTA pin index: 17 */

/*! @name PORTA18 (number 6), SPI1_CLK */
#define BOARD_SPI1_CLK_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_SPI1_CLK_PIN 18U     /*!<@brief PORTA pin index: 18 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI1Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI split mode
 *
 * END ****************************************************************************************************************/
void BOARD_InitSPI1Pins(void);

//=====================================================================================================================
// [ Configuration CAN0 ] - 
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_FOB_H

#elif defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_EVT1_H || defined __HW_VENUS_EVT1_H
    #define BOARD_INITFLEXCAN_CAN0_TX_PORT PORTC
    #define BOARD_INITFLEXCAN_CAN0_TX_PIN 3U

    #define BOARD_INITFLEXCAN_CAN0_RX_PORT PORTC
    #define BOARD_INITFLEXCAN_CAN0_RX_PIN 4U

	#define BOARD_INITFLEXCAN_CAN0_STB_GPIO GPIOC
	#define BOARD_INITFLEXCAN_CAN0_STB_PORT PORTC
	#define BOARD_INITFLEXCAN_CAN0_STB_PIN 2U

	/* Symbols to be used with PORT driver */
	#define BOARD_INITPINS_CAN_RX_PORT PORTC               /*!<@brief PORT peripheral base pointer */
	#define BOARD_INITPINS_CAN_RX_PIN 4U                   /*!<@brief PORT pin number */
	#define BOARD_INITPINS_CAN_RX_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
														   /* @} */

	/*! @name PORTC3 (number 39), J1[4]/U19[1]/U20[4]/D3/CAN_TX
	  @{ */

	/* Symbols to be used with PORT driver */
	#define BOARD_INITPINS_CAN_TX_PORT PORTC               /*!<@brief PORT peripheral base pointer */
	#define BOARD_INITPINS_CAN_TX_PIN 3U                   /*!<@brief PORT pin number */
	#define BOARD_INITPINS_CAN_TX_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
														   /* @} */
#else
//    #define BOARD_INITFLEXCAN_CAN0_TX_PORT PORTC
//    #define BOARD_INITFLEXCAN_CAN0_TX_PIN 3U
//
//    #define BOARD_INITFLEXCAN_CAN0_RX_PORT PORTC
//    #define BOARD_INITFLEXCAN_CAN0_RX_PIN 4U
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitFlexcan
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitFlexcan(void);

//=====================================================================================================================
// [ Configuration UWB ] - 
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.3 -20240319
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_EVT1_H || defined __HW_VENUS_EVT1_H
      /*! @name PORTA19 (number 7), UWB_CS output*/
      #define BOARD_UWB_CS_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
      #define BOARD_UWB_CS_PORT PORTA /*!<@brief PORT device name: PORTA */
      #define BOARD_UWB_CS_PIN  19U     /*!<@brief PORTA pin index: 19 */

      #if defined __HW_BLE_UWB_FOB_H
      /*! @name PORTC4 (number xx), UWB_RDY input*/
      #define BOARD_UWB_RDY_GPIO GPIOC /*!<@brief GPIO device name: GPIOB */
      #define BOARD_UWB_RDY_PORT PORTC /*!<@brief PORT device name: PORTB */
      #define BOARD_UWB_RDY_PIN  4U     /*!<@brief PORTC pin index: 0 */
      #else
      /*! @name PORTB0 (number xx), UWB_RDY input*/
      #define BOARD_UWB_RDY_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_UWB_RDY_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_UWB_RDY_PIN  0U     /*!<@brief PORTC pin index: 0 */
      #endif

      /*! @name PORTC1 (number xx), UWB_RST output */
      #define BOARD_UWB_RST_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_RST_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_RST_PIN  1U     /*!<@brief PORTC pin index: 1 */

      /*! @name PORTB18 (number xx), UWB_INT input */
      #define BOARD_UWB_INT_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_UWB_INt_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_UWB_INT_PIN  18U     /*!<@brief PORTC pin index: 18 */
#else
      /*! @name PORTA19 (number 7), UWB_CS output*/
      #define BOARD_UWB_CS_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
      #define BOARD_UWB_CS_PORT PORTA /*!<@brief PORT device name: PORTA */
      #define BOARD_UWB_CS_PIN 19U     /*!<@brief PORTA pin index: 19 */

      /*! @name PORTC5 (number 41), UWB_RDY input*/
      #define BOARD_UWB_RDY_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_RDY_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_RDY_PIN 5U     /*!<@brief PORTC pin index: 5 */

      /*! @name PORTC19 (number 48), UWB_RST output */
      #define BOARD_UWB_RST_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_RST_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_RST_PIN 19U     /*!<@brief PORTC pin index: 19 */

      /*! @name PORTC6 (number 42), UWB_INT input */
      #define BOARD_UWB_INT_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_INt_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_INT_PIN 6U     /*!<@brief PORTC pin index: 6 */
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitRanger4Pins
 * Description   : Init UWB connected Pins
 *
 * END ****************************************************************************************************************/
//Modify (Ken):NXP-V0001 NO.3 -20240319
void BOARD_InitRanger4Pins(void);

//=====================================================================================================================
// [ Configuration Security Chip ] - 
//=====================================================================================================================
#define BOARD_SECURITY_EN_GPIO  GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_SECURITY_EN_PORT  PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_SECURITY_EN_PIN   5U     /*!<@brief PORTC pin index: 5 */


#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
