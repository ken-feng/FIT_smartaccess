/*
 (c) NXP B.V. 2019-2020. All rights reserved.

Disclaimer
1. The NXP Software/Source Code is provided to Licensee "AS IS" without any
   warranties of any kind. NXP makes no warranties to Licensee and shall not
   indemnify Licensee or hold it harmless for any reason related to the NXP
   Software/Source Code or otherwise be liable to the NXP customer. The NXP
   customer acknowledges and agrees that the NXP Software/Source Code is
   provided AS-IS and accepts all risks of utilizing the NXP Software under
   the conditions set forth according to this disclaimer.

2. NXP EXPRESSLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING,
   BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
   FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT OF INTELLECTUAL PROPERTY
   RIGHTS. NXP SHALL HAVE NO LIABILITY TO THE NXP CUSTOMER, OR ITS
   SUBSIDIARIES, AFFILIATES, OR ANY OTHER THIRD PARTY FOR ANY DAMAGES,
   INCLUDING WITHOUT LIMITATION, DAMAGES RESULTING OR ALLEGED TO HAVE
   RESULTED FROM ANY DEFECT, ERROR OR OMISSION IN THE NXP SOFTWARE/SOURCE
   CODE, THIRD PARTY APPLICATION SOFTWARE AND/OR DOCUMENTATION, OR AS A
   RESULT OF ANY INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHT OF ANY
   THIRD PARTY. IN NO EVENT SHALL NXP BE LIABLE FOR ANY INCIDENTAL,
   INDIRECT, SPECIAL, EXEMPLARY, PUNITIVE, OR CONSEQUENTIAL DAMAGES
   (INCLUDING LOST PROFITS) SUFFERED BY NXP CUSTOMER OR ITS SUBSIDIARIES,
   AFFILIATES, OR ANY OTHER THIRD PARTY ARISING OUT OF OR RELATED TO THE NXP
   SOFTWARE/SOURCE CODE EVEN IF NXP HAS BEEN ADVISED OF THE POSSIBILITY OF
   SUCH DAMAGES.

3. NXP reserves the right to make changes to the NXP Software/Sourcecode any
   time, also without informing customer.

4. Licensee agrees to indemnify and hold harmless NXP and its affiliated
   companies from and against any claims, suits, losses, damages,
   liabilities, costs and expenses (including reasonable attorney's fees)
   resulting from Licensee's and/or Licensee customer's/licensee's use of the
   NXP Software/Source Code.
 */

/*
 * $URL: $
 * $Revision: $ */

/*
 *   @File: phscaR4DriverHw.c
 *   @Brief: Configuration of driver for Ranger4 UWB module
 */
//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
/* =============================================================================
 * External Includes
 * ========================================================================== */
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "GPIO_Adapter.h"
#include "pin_mux.h"
#include "phscaR4DriverHw.h"
/* =============================================================================
 * Internal Includes
 * ========================================================================== */


/* =============================================================================
 * Private Symbol Defines
 * ========================================================================== */
#define gRanger4_IsrPrio_c (0x80)

/* =============================================================================
 * Private Function-like Macros
 * ========================================================================== */

/* =============================================================================
 * Private Type Definitions
 * ========================================================================== */

/* =============================================================================
 * Private Function Prototypes
 * ========================================================================== */

/* =============================================================================
 * Private Module-wide Visible Variables
 * ========================================================================== */
extern gpioInputPinConfig_t Ranger4InputPins[];

pRanger4IntCallBack_t RangerIntcallback = NULL;
void * pRanger4IntParam;
/* =============================================================================
 * Private Function Definitions
 * ========================================================================== */
static void Ranger_Int_ISR(void)
{
    if(GpioIsPinIntPending(Ranger4InputPins) != 0u)
    {
        /* set the local variable to mark that the interrupt is caused by INT*/
        GpioClearPinIntFlag(Ranger4InputPins);
        
        if(RangerIntcallback != NULL)
        {
            RangerIntcallback(pRanger4IntParam);
        }
    }
}
/* =============================================================================
 * Public Function Definitions
 * ========================================================================== */
void phscaR4Driver_InitSixWireInterface(void)
{
    BOARD_InitRanger4Pins();
    //Set pin interrupt 
    GpioInputPinInit(Ranger4InputPins,1);
    GpioInstallIsr( Ranger_Int_ISR, gGpioIsrPrioLow_c, gRanger4_IsrPrio_c, &Ranger4InputPins[0]);

}

void phscaR4Driver_DisableINTInterrupt(void)
{
    (void)GpioUninstallIsr( &Ranger4InputPins[0] );
    GpioSetPinInterrupt(Ranger4InputPins[0].gpioPort, Ranger4InputPins[0].gpioPin, pinInt_Disabled_c);
    GpioClearPinIntFlag(Ranger4InputPins);
}

void phscaR4Driver_EnableINTInterrupt(void)
{
    GpioInputPinInit(Ranger4InputPins,1);
    GpioInstallIsr( Ranger_Int_ISR, gGpioIsrPrioLow_c, gRanger4_IsrPrio_c, &Ranger4InputPins[0]);
    GpioClearPinIntFlag(Ranger4InputPins);
}

void phscaR4Driver_DeInitSixWireInterface(void)
{
    //Set uninstall pin interrupt 
//    (void)GpioUninstallIsr( &Ranger4InputPins[0] );
    phscaR4Driver_DisableINTInterrupt();
    
    phscaR4Driver_ResetOff();
    phscaR4Driver_ChipSelectOff();
}

void phscaR4Driver_SetINTCallBack( pRanger4IntCallBack_t cb, void *pRxParam )
{
    RangerIntcallback = cb;
    pRanger4IntParam = pRxParam;
}

void phscaR4Driver_ResetOn(void)
{
    GPIO_PinWrite(BOARD_UWB_RST_GPIO, BOARD_UWB_RST_PIN, 0u);
}

void phscaR4Driver_ResetOff(void)
{
    GPIO_PinWrite(BOARD_UWB_RST_GPIO, BOARD_UWB_RST_PIN, 1u);
}

void phscaR4Driver_ChipSelectOn(void)
{
    GPIO_PinWrite(BOARD_UWB_CS_GPIO, BOARD_UWB_CS_PIN, 0u);
}

void phscaR4Driver_ChipSelectOff(void)
{
    GPIO_PinWrite(BOARD_UWB_CS_GPIO, BOARD_UWB_CS_PIN, 1u);
}

bool phscaR4Driver_GetReadyLineLevel(void)
{
    bool b_ReadyLineLevel = true;

    b_ReadyLineLevel = (bool)GPIO_PinRead(BOARD_UWB_RDY_GPIO, BOARD_UWB_RDY_PIN);

    return b_ReadyLineLevel;
}

bool phscaR4Driver_GetINTLineLevel(void)
{
    bool b_irqLineLevel = true; 

    b_irqLineLevel = (bool)GPIO_PinRead(BOARD_UWB_INT_GPIO, BOARD_UWB_INT_PIN);

    return b_irqLineLevel;
}

//void phscaR4Driver_ClearINTLineStatus(void)
//{
//
//}

/*! *********************************************************************************
*************************************************************************************
**************************UCI pin controll functions*********************************
*************************************************************************************
*************************************************************************************/

/*! *********************************************************************************
* \brief  before start reveive data, check if there have data from slave need to transfer.
*
* @retval slaveTransferStatus_HasData if ranger4 device has data to send.
* @retval slaveTransferStatus_Undefined if others.
********************************************************************************** */
phscaR4Driver_SlaveTransferStatus_t phscaR4Driver_SixWireStartReceive(void)
{
    /* Initialize values */
    phscaR4Driver_SlaveTransferStatus_t retVal = slaveTransferStatus_Undefined;
    volatile uint16_t delayCount = 0u;
    volatile uint16_t filterCount = 0u;

    /* Pull down CSn line */
    phscaR4Driver_ChipSelectOn();

    /* Wait around 10ms for calculating keys by Crypto function when it is activated */
    while (delayCount < 25000u)
    {
        /* Check if INT line is active or not */
        if (HW_PINLEVEL_LOW == phscaR4Driver_GetINTLineLevel())
        {
            /* Reset filter counter */
            filterCount = 0u;
            /* Delay around 100 microseconds for filtering IRQ toggling */
            while (filterCount < 250u)
            {
                filterCount++;
            }
            /* Check if INT line is still active */
            if (HW_PINLEVEL_LOW == phscaR4Driver_GetINTLineLevel())
            {
                /* Slave has data to send */
                retVal = slaveTransferStatus_HasData;
            }
        }

        if (slaveTransferStatus_HasData == retVal)
        {
            /* Status is have data, run next process to receive data */
            break;
        }

        /* Increase delay counter */
        delayCount++;
    }

    return retVal;
}

/*! *********************************************************************************
 * \brief Stop receive data with six wire mode over SPI peripheral.
 *
 * @retval true if successful.
 * @retval false if failed.
********************************************************************************** */
bool phscaR4Driver_SixWireStopReceive(void)
{
    bool retVal = false;
    volatile uint16_t delayCount = 0u;

    /* Wait around 10ms for calculating keys by crypto function when it is activated */
    while (delayCount < 25000u)
    {
        if (HW_PINLEVEL_HIGH == phscaR4Driver_GetINTLineLevel())
        {
            /* INT line goes to inactive, the transfer is fully completed */
            retVal = true;
            break;
        }
        delayCount++;
    }

    /* Pull up CSn line */
    phscaR4Driver_ChipSelectOff();
    return retVal;
}

/*! *********************************************************************************
* \brief    start transfer data, check if there have data from slave need to transfer,
*           or if slave ready for transfer data
* @retval   slaveTransferStatus_Ready if ranger4 device is ready to receive data.
* @retval   slaveTransferStatus_HasData if ranger4 device has data to send.
* @retval   slaveTransferStatus_Undefined if others.
********************************************************************************** */
phscaR4Driver_SlaveTransferStatus_t phscaR4Driver_SixWireStartTransmit(void)
{
    /* Initialize values */
    phscaR4Driver_SlaveTransferStatus_t retVal = slaveTransferStatus_Undefined;
    bool readyStatus = HW_PINLEVEL_HIGH;
    bool intStatus = HW_PINLEVEL_HIGH;
    volatile uint16_t delayCount = 0u;
    volatile uint16_t filterCount = 0u;

    /* Pull down CSn line */
    phscaR4Driver_ChipSelectOn();

    /* Wait around 10ms for calculating keys by Crypto function when it is activated */
    while (delayCount < 25000u)
    {
        /* Get ready status */
        readyStatus = phscaR4Driver_GetReadyLineLevel();
        /* Get initerrupt status */
        intStatus = phscaR4Driver_GetINTLineLevel();
        /* Check if ready or interrupt is active */
        if ((HW_PINLEVEL_LOW == readyStatus) || (HW_PINLEVEL_LOW == intStatus))
        {
            /* Reset filter counter */
            filterCount = 0u;
            /* Delay around 100 microseconds for filtering RDY toggling */
            while (filterCount < 250u)
            {
                filterCount++;
            }
            /* Check again if ready is active */
            if ((HW_PINLEVEL_LOW == readyStatus) && (HW_PINLEVEL_LOW == phscaR4Driver_GetReadyLineLevel()))
            {
                retVal = slaveTransferStatus_Ready;
            }
            /* Check again if interrupt is active */
            if ((HW_PINLEVEL_LOW == intStatus) && (HW_PINLEVEL_LOW == phscaR4Driver_GetINTLineLevel()))
            {
                retVal = slaveTransferStatus_HasData;
            }
        }

        if (slaveTransferStatus_Undefined != retVal)
        {
            /* Status is ready or has data, break out the loop */
            break;
        }
        delayCount++;
    }
    return retVal;
}

/*! *********************************************************************************
 * @brief Stop transmit data with six wire mode over SPI peripheral.
 *
 * @retval true if successful.
 * @retval false if failed.
********************************************************************************** */
bool phscaR4Driver_SixWireStopTransmit(void)
{
    bool retVal = true;
    bool readyStatus = HW_PINLEVEL_HIGH;

    /* Pull chip select up */
    phscaR4Driver_ChipSelectOff();

    /* Wait for RDY to go high */
    do
    {
        readyStatus = HW_PINLEVEL_HIGH;
        readyStatus = phscaR4Driver_GetReadyLineLevel();
    }
    while(readyStatus == HW_PINLEVEL_LOW );
    
    return retVal;
}


#endif  // for UWB_Support
