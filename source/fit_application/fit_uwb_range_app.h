//***************************************************** 
// The KW38 serial head file define 
// Define the main program Ram section  
//***************************************************** 
//#include "fsl_common.h"
//#include "fsl_port.h"
//#include "fsl_gpio.h"
//#include "pin_mux.h"
//****************************************************
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))

/*************************************************************************************
**************************************************************************************
* Public macros
**************************************************************************************
*************************************************************************************/
#define UWB_MAX_QUERY_TIMESTAMP_LIST_SIZE                   32u
#define START_RANGING_FLAG                                 (1u << 7)

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */

void Init_UWB_Range(void);
void Ranger4App_RspCallback(phscaR4CadsTypesIntf_UciFrame_t * frame);
void Ranger4App_NtfCallback(phscaR4CadsTypesIntf_UciFrame_t * frame);
void ProcessUwbTimeStamp(uint8_t * pPayload, phscaR4CadsTypesIntf_UciStatusCode_t StateCode);
void App_SetRangingSessionCfg(deviceId_t deviceId, SessionManagement_t * pSessionPara);

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

//==============================================================================
// UWB
//==============================================================================
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
/*! *********************************************************************************
 * \brief        Query Time Stamp from UWB
 *
 * \param[in]    deviceId   0xF0 & valid DeviceId: start ranging after get timestamp
                            valid DeviceId: get timestamp for peer device time sync
 ********************************************************************************** */
void QueryUwbTimeStamp(deviceId_t deviceId);

/*! ********************************************************************************************
 * \brief        Sends UWB Ranging Result values over-the-air by uuid_characteristic_ranging_result
 *               notification to the Keyfob.
 * \param[in]    distance   point of distance
 **********************************************************************************************/
void BleApp_SendAllUWBRangingResult(uint32_t sessionID);

uint8_t UWB_Api_Start_Ranging(void* pUsrkDst, uint16_t sessio_id);
uint8_t UWB_Api_Stop_Ranging(uint16_t sessio_id);
void UWB_Api_Reset(void);

#endif

#endif




