//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
//******************************************************************************
//
//******************************************************************************
#ifndef __PHSCAR4UCISPI__H__
#define __PHSCAR4UCISPI__H__

/* =============================================================================
 * External Includes
 * ========================================================================== */
#include "phscaR4CadsTypesIntf.h"   
/* =============================================================================
 * Symbol Defines
 * ========================================================================== */
#define PHSCAR4_UCI_RES_HEADER_LENGTH                                   (4u)
/* =============================================================================
 * Type Definitions
 * =============================================================================
 */ 

/* =============================================================================
 * Public Function-like Macros
 * ========================================================================== */

/* =============================================================================
 * Public Standard Enumerators
 * ========================================================================== */

/* =============================================================================
 * Public Function Prototypes
 * ========================================================================== */
phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_ReceiveData(phscaR4CadsTypesIntf_UciFrame_t * rxframe);
phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_SendData(phscaR4CadsTypesIntf_UciFrame_t * frame);
phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_TransferData(phscaR4CadsTypesIntf_UciFrame_t * txframe, 
                                                            phscaR4CadsTypesIntf_UciFrame_t * rxframe);
phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_InterfaceInit(void);
phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_InterfaceDeInit(void);
void phscaR4UciSpi_HardwareResetRanger4(void);

#endif

#endif  // for UWB_Support
