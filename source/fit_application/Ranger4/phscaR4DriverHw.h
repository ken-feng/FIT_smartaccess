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
 *   @File : phscaR4DriverHw.h
 *   @Brief: Configuration of driver for Ranger4 UWB module
 */
//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
//******************************************************************************
//
//******************************************************************************
#ifndef __PHSCARNG4DRIVERHW_H__
#define __PHSCARNG4DRIVERHW_H__

/* =============================================================================
 * External Includes
 * ========================================================================== */
#include "fsl_common.h"
/* =============================================================================
 * Symbol Defines
 * ========================================================================== */
#define    PHSCARNG4_u32_CRC16_POLYNOMIAL           (uint32_t)(0x00001021ul)
#define    PHSCARNG4_u32_CRC16_SEED                 (uint32_t)(0x00000000ul)

#define    HW_PINLEVEL_LOW      false
#define    HW_PINLEVEL_HIGH     true
/* =============================================================================
 * Type Definitions
 * ========================================================================== */
/**
 * @brief Slave transfer status
 */
typedef enum
{
    /* Slave is ready to receive data */
    slaveTransferStatus_Ready = 0u,
    /* Slave has data need to send */
    slaveTransferStatus_HasData = 1u,
    /* Undefined status */
    slaveTransferStatus_Undefined = 0xFFu,
} phscaR4Driver_SlaveTransferStatus_t;



/* =============================================================================
 * Public Function-like Macros
 * ========================================================================== */

/* =============================================================================
 * Public Standard Enumerators
 * ========================================================================== */
typedef void (*pRanger4IntCallBack_t)(void * pParam);

/* =============================================================================
 * Public Function Prototypes
 * ========================================================================== */
void phscaR4Driver_InitSixWireInterface(void);
void phscaR4Driver_DeInitSixWireInterface(void);
void phscaR4Driver_ResetOn(void);
void phscaR4Driver_ResetOff(void);
void phscaR4Driver_ChipSelectOn(void);
void phscaR4Driver_ChipSelectOff(void);
bool phscaR4Driver_GetReadyLineLevel(void);
void phscaR4Driver_DisableINTInterrupt(void);
void phscaR4Driver_EnableINTInterrupt(void);
bool phscaR4Driver_GetINTLineLevel(void);
phscaR4Driver_SlaveTransferStatus_t phscaR4Driver_SixWireStartReceive(void);
phscaR4Driver_SlaveTransferStatus_t phscaR4Driver_SixWireStartTransmit(void);
bool phscaR4Driver_SixWireStopTransmit(void);
bool phscaR4Driver_SixWireStopReceive(void);

void phscaR4Driver_SetINTCallBack( pRanger4IntCallBack_t cb, void *pRxParam );

//void phscaR4Driver_ClearINTLineStatus(void);



/*! *********************************************************************************
*************************************************************************************
**************************UCI pin controll functions*********************************
*************************************************************************************
*************************************************************************************/

/*! *********************************************************************************
* \brief    start reveive data, check if there have data from slave need to transfer.
*
* @retval slaveTransferStatus_HasData if ranger4 device has data to send.
* @retval slaveTransferStatus_Undefined if others.
********************************************************************************** */
phscaR4Driver_SlaveTransferStatus_t phscaR4Driver_SixWireStartReceive(void);
/*! *********************************************************************************
 * \brief Stop receive data with six wire mode over SPI peripheral.
 *
 * @retval true if successful.
 * @retval false if failed.
********************************************************************************** */
bool phscaR4Driver_SixWireStopReceive(void);

/*! *********************************************************************************
* \brief    start transfer data, check if there have data from slave need to transfer,
*           or if slave ready for transfer data
* @retval   slaveTransferStatus_Ready if ranger4 device is ready to receive data.
* @retval   slaveTransferStatus_HasData if ranger4 device has data to send.
* @retval   slaveTransferStatus_Undefined if others.
********************************************************************************** */
phscaR4Driver_SlaveTransferStatus_t phscaR4Driver_SixWireStartTransmit(void);
     
/*! *********************************************************************************
 * @brief Stop transmit data with six wire mode over SPI peripheral.
 *
 * @retval true if successful.
 * @retval false if failed.
********************************************************************************** */
bool phscaR4Driver_SixWireStopTransmit(void);


#endif

#endif  // for UWB_Support
