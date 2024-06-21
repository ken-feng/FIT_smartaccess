//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
//******************************************************************************
//
//******************************************************************************
#ifndef __R4UCICMD__H__
#define __R4UCICMD__H__



/* =============================================================================
 * External Includes
 * ========================================================================== */
//#include "phscaSTD_StandardTypes.h"
#include "phscaR4CadsTypesIntf.h"   
/* =============================================================================
 * Symbol Defines
 * ========================================================================== */
/**
 * @brief UCI SWUP activate 
 */
#define PHSCA_R4SWUP_UCI_OID_ACTIVATE                                0x3Fu
#define PHSCA_R4SWUP_UCIMAC_OID_ACTIVATE                             0x12u
/**
 * @brief UCI SWUP set deactivate  
 */
#define PHSCA_R4SWUP_UCI_OID_SET_DEACTIVATE                          0x01u

/**
 * @brief UCI SWUP clear RAM Manifest  
 */
#define PHSCA_R4SWUP_UCI_OID_CLEAR_RAM_MANIFEST                      0x06u

/**
 * @brief UCI SWUP transfer Manifest  
 */
#define PHSCA_R4SWUP_UCI_OID_TRANSFER_MANIFEST                       0x07u

/**
 * @brief UCI SWUP start update  
 */
#define PHSCA_R4SWUP_UCI_OID_START_UPDATE                            0x0Au

/**
 * @brief UCI SWUP transfer component  
 */
#define PHSCA_R4SWUP_UCI_OID_TRANSFER_COMPONENT                      0x10u

/**
 * @brief UCI SWUP verify component 
 */
#define PHSCA_R4SWUP_UCI_OID_VERIFY_COMPONENT                        0x11u

/**
 * @brief UCI SWUP verify all  
 */
#define PHSCA_R4SWUP_UCI_OID_VERIFY_ALL                              0x12u

/**
 * @brief UCI SWUP finish update  
 */
#define PHSCA_R4SWUP_UCI_OID_FINISH_UPDATE                           0x1Au

/**
 * @brief UCI SWUP get device info 
 */
#define PHSCA_R4SWUP_UCI_OID_GET_DEVICE_INFO                         0x20u

/**
 * @brief UCI SWUP read device ID  
 */
#define PHSCA_R4SWUP_UCI_OID_READ_DEVICE_ID                          0x21u

#define PHSCAR4_UCISWUP_DATA_SIZE                                   (128u) 

#define PHSCAR4_UCI_HEADER_LENGTH                                   (4u)
     
/* =============================================================================
 * Type Definitions
 * =============================================================================
 */ 
/** @brief Structure to hold data for the Device info */
typedef struct
{
    uint8_t ProductID[8];
    uint8_t HardwareID[4];
    uint8_t TypecheckID[8];
    uint8_t RomID[8];
    uint8_t SWUPVersion[8];
}phscaR4_UciHwDeviceInfo_t;

/** @brief Structure to hold data for the Device info */
typedef struct
{
    uint8_t WaferID[11];
    uint8_t WaferNumber;
    uint8_t WaferXCoordinate[2];
    uint8_t WaferYCoordinate[2];
    uint8_t SerialNumber[4];
}phscaR4_UciWaferDeviceID_t;

/**
 * @brief SPI command Pkt send mode
 */
typedef enum
{
    R4Pkt_SendSync,
    R4Pkt_SendAsync
}Ranger4Pkt_SendMode_t;

/**
 * @brief UCI core control Group opcode identifier
 */
typedef enum
{
    R4_UCI_CORE_OID_RESET_DEVICE            = 0x00u,
    R4_UCI_CORE_OID_CORE_DEVICE_STATUS_NTF  = 0x01u,
    R4_UCI_CORE_OID_CORE_GET_DEVICE_INFO    = 0x02u,
    R4_UCI_CORE_OID_CORE_GET_CAPS_INFO      = 0x03u,
    R4_UCI_CORE_OID_CORE_SET_CONFIG         = 0x04u,
    R4_UCI_CORE_OID_CORE_GET_CONFIG         = 0x05u,
    R4_UCI_CORE_OID_REVERSE                 = 0x06u,
    R4_UCI_CORE_OID_CORE_GENERIC_ERROR_NTF  = 0x07u,
}Ranger4Uci_CoreGroupOid_t;

/**
 * @brief UCI UWB Session Config Group opcode identifier
 */
typedef enum
{
    R4_UCI_SESSION_CFG_OID_INIT             = 0x00u,
    R4_UCI_SESSION_CFG_OID_DEINIT           = 0x01u,
    R4_UCI_SESSION_CFG_OID_STATUS_NTF       = 0x02u,
    R4_UCI_SESSION_CFG_OID_SET_APP_CFG      = 0x03u,
    R4_UCI_SESSION_CFG_OID_GET_APP_CFG      = 0x04u,
    R4_UCI_SESSION_CFG_OID_GET_COUNT        = 0x05u,
    R4_UCI_SESSION_CFG_OID_GET_STATE        = 0x06u,
    R4_UCI_SESSION_CFG_OID_GET_RAN_MULTIPLIER_VALUE  = 0x20u,
}Ranger4Uci_UwbSessionConfigGroupOid_t;

/**
 * @brief UCI UWB Session control Group opcode identifier
 */
typedef enum
{
    R4_UCI_RANGE_CTR_OID_START              = 0x00u,
    R4_UCI_RANGE_CTR_OID_STOP               = 0x01u,
    R4_UCI_RANGE_CTR_OID_GET_RANGING_COUNT  = 0x03u,
    R4_UCI_RANGE_CTR_OID_CCC_DATA_NTF       = 0x20u,
    R4_UCI_RANGE_CTR_OID_RESUME             = 0x21u,
}Ranger4Uci_UwbSessionControlGroupOid_t;

/**
 * @brief UCI Proprietary Group opcode identifier
 */
typedef enum
{
    R4_UCI_PROPRIETARY_OID_LOG_NTF            = 0x00u,
    R4_UCI_PROPRIETARY_OID_RADIO_CFG_DOWNLOAD = 0x11u,
    R4_UCI_PROPRIETARY_OID_ACTIVATE_SWUP      = 0x12u,
    R4_UCI_PROPRIETARY_OID_TEST_START         = 0x20u,
    R4_UCI_PROPRIETARY_OID_TEST_STOP_NTF      = 0x21u,
    R4_UCI_PROPRIETARY_OID_QUERY_UWB_TIMESTAMP= 0x23u,
    R4_UCI_PROPRIETARY_OID_DEVICE_SUSPEND     = 0x24u,
    R4_UCI_PROPRIETARY_OID_TEST_LOOPBACK_NTF  = 0x25u,
    R4_UCI_PROPRIETARY_OID_SET_TRIM_VALUE_NTF = 0x26u,
    R4_UCI_PROPRIETARY_OID_GET_ALL_UWB_SESSION= 0x27u,
    R4_UCI_PROPRIETARY_OID_GET_TRIM_VALUES    = 0x28u,
    
    R4_UCI_PROPRIETARY_OID_STORE_PROTECTION_KEY= 0x29u,
    R4_UCI_PROPRIETARY_OID_SET_EPOCH_ID        = 0x2Au,
    R4_UCI_PROPRIETARY_OID_TBD                 = 0x2Bu,
    R4_UCI_PROPRIETARY_OID_USER_DEFINED_RANGE_DATA = 0x2Cu,    
}Ranger4Uci_ProprietaryGroupOid_t;

typedef enum
{
    R4_UCI_DEVICE_STATUS_UNDEFINE           = 0x00u, 
    R4_UCI_DEVICE_STATUS_READY              = 0x01u,  /*!UWBS is initialized and ready UWB session */ 
    R4_UCI_DEVICE_STATUS_ACTIVE             = 0x02u,  /*!UWBS is busy with UWB session */
    R4_UCI_DEVICE_STATUS_STATUS_ERROR       = 0xFFu,  /*!Error occurred within UWBS*/
    /* Proprietary status*/
    R4_UCI_DEVICE_STATUS_NOT_SUPPORT        = 0xE3u, /*!Device type is not NCJ29D5D */   
    R4_UCI_DEVICE_STATUS_TESTMODE           = 0xE4u, /*!Device is in test mode */
    R4_UCI_DEVICE_STATUS_REBOOT_ON_SW       = 0xFBu, /*!Device reset due to SW reset trigger */
    R4_UCI_DEVICE_STATUS_REBOOT_ON_HDP      = 0xFCu, /*!Device wake up from HPD*/
    R4_UCI_DEVICE_STATUS_REBOOT_ON_PIN      = 0xFDu, /*!Reset caused by RST pin*/
    R4_UCI_DEVICE_STATUS_REBOOT_ON_POWER    = 0xFEu, /*!Reset caused by power issue*/
    R4_UCI_DEVICE_STATUS_REBOOT_ON_WDT      = 0xFFu, /*!Reset caused by watchdog timer */
}Ranger4Uci_DeviceStatusType_t;



typedef enum
{
    R4_DEVICE_CAP_SLOT_BITMASK                 = 0xA0u,
    R4_DEVICE_CAP_SYNC_CODE_INDEX_BITMASK      = 0xA1u,
    R4_DEVICE_CAP_HOPPING_CONFIG_BITMASK       = 0xA2u,
    R4_DEVICE_CAP_CHANNEL_BITMASK              = 0xA3u,
    R4_DEVICE_CAP_SUPPORTED_PROTOCOL_VERSION   = 0xA4u,
    R4_DEVICE_CAP_SUPPORTED_UWB_CONFIG_ID      = 0xA5u,
    R4_DEVICE_CAP_SUPPORTED_PULSESHAPE_COMBO   = 0xA6u,
    
    /* Proprietary device capability information */
    R4_DEVICE_CAP_MAX_PAYLOAD_LEN              = 0xE3u,
    R4_DEVICE_CAP_MIN_SLOT_LEN                 = 0xE4u,
    R4_DEVICE_CAP_MAX_SESSION_NUM              = 0xE5u,
    R4_DEVICE_CAP_MAX_ANCHOR_NUM               = 0xE6u,
    R4_DEVICE_CAP_MIN_UWB_FREQ                 = 0xE7u,
    R4_DEVICE_CAP_MAX_UWB_FREQ                 = 0xE8u,
    R4_DEVICE_CAP_BINARY_COMBINATION           = 0xE9u,   /*!Supported Device role and protocols*/ 
}Ranger4Uci_DeviceCapabilityType_t;

typedef enum
{
    R4_PARAM_DEVICE_STATE            = 0x00u,
    R4_PARAM_LOW_POWER_MODE          = 0x01u,
    /*extended device configurations*/
    R4_PARAM_RESET_TIMEOUT           = 0xE5u,
    R4_PARAM_WAKEUP_PIN              = 0xE6u,
    R4_PARAM_HPD_ENTRY_TIMEOUT       = 0xEAu,
    R4_PARAM_RX_PHY_LOGGING_ENBL     = 0xF4u,
    R4_PARAM_TX_PHY_LOGGING_ENBL     = 0xF5u,
    R4_PARAM_LOG_PARAMS_CONF         = 0xF6u,
    R4_PARAM_CIR_TAP_OFFSET          = 0xF7u,
    R4_PARAM_CIR_NUM_TAPS            = 0xF8u,
}Ranger4Uci_CfgParamType_t;

typedef struct
{
    Ranger4Uci_CfgParamType_t     ID;         /*!< The identifier of the configuration parameter. */
    uint8_t                                 length;     /*!< The length of paramValue */
    uint8_t*                                paramValue; /*!< The value of the configuration parameter */
} Ranger4Uci_ParameterStructure_t;



typedef enum
{
    R4_SESSION_DEVICE_TYPE                 = 0x00u,
    R4_SESSION_STS_CONFIG                  = 0x02u,
    R4_SESSION_CHANNEL_ID                  = 0x04u,
    R4_SESSION_NUMBER_OF_ANCHORS           = 0x05u,
    R4_SESSION_DEVICE_MAC_ADDRESS          = 0x06u,
    R4_SESSION_DST_MAC_ADDRESS             = 0x07u,
    R4_SESSION_RANGING_SLOT_LENGTH         = 0x08u,
    R4_SESSION_RANGING_INTERVAL            = 0x09u,
    R4_SESSION_STS_INDEX0                  = 0x0Au,
    R4_SESSION_MAC_FCS_TYPE                = 0x0Bu,
    R4_SESSION_RNG_DATA_NTF                = 0x0Eu,
    R4_SESSION_RNG_DATA_NTF_PROXIMITY_NEAR = 0x0fu,
    R4_SESSION_RNG_DATA_NTF_PROXIMITY_FAR  = 0x10u,
    R4_SESSION_DEVICE_ROLE                 = 0x11u,
    R4_SESSION_PREAMBLE_ID                 = 0x14u,
    R4_SESSION_SFD_ID                      = 0x15u,
    R4_SESSION_SLOTS_PER_RR                = 0x1Bu,
    R4_SESSION_ADAPTIVE_PAYLOAD_POWER      = 0x1Cu,
    R4_SESSION_RESPONDER_SLOT_INDEX        = 0x1Eu,
    R4_SESSION_KEY_ROTATION                = 0x23u,
    R4_SESSION_SESSION_PRIORITY            = 0x25u,
    R4_SESSION_MAX_RR_RETRY                = 0x2Au,
    R4_SESSION_UWB_INITIATION_TIME         = 0x2Bu,
    R4_SESSION_HOPPING_MODE                = 0x2Cu,
    R4_SESSION_MAX_NUMBER_OF_MEASUREMENTS  = 0x32u,
    R4_SESSION_HOP_MODE_KEY                = 0xA0u,
    R4_SESSION_RANGING_PROTOCOL_VER        = 0xA3u,
    R4_SESSION_UWB_CONFIG_ID               = 0xA4u,
    R4_SESSION_PULSESHAPE_COMBO            = 0xA5u,
    R4_SESSION_URSK_TTL                    = 0xA6u,
    /*extended session configurations*/
    R4_SESSION_RX_START_MARGIN             = 0xE3u,
    R4_SESSION_RX_TIMEOUT                  = 0xE4u,
    R4_SESSION_ADAPTED_RANGING_INDEX       = 0xE5u,
    R4_SESSION_NBIC_CONF                   = 0xE6u,
    R4_SESSION_GROUPDEALY_RECALC_ENA       = 0xE7u,
    R4_SESSION_URSK                        = 0xE8u,
    R4_SESSION_STATIC_KEYS                 = 0xE9u,
    R4_SESSION_RCM_RX_MARGIN_TIME          = 0xEAu,
    R4_SESSION_RCM_RX_TIMEOUT              = 0xEBu,
    R4_SESSION_DYNAMIC_PRIORITY_IN_SYNCH   = 0xECu,
    R4_SESSION_TX_POWER_TEMP_COMPENSATION  = 0xEDu,
    R4_SESSION_LONG_SRC_ADDRESS            = 0xEFu,
    R4_SESSION_N                           = 0xF0u,
    R4_SESSION_RR_RETRY_THR                = 0xF1u,
    R4_SESSION_TX_POWER_ID                 = 0xF2u,
    R4_SESSION_RX_PHY_LOGGING_ENBL         = 0xF4u,
    R4_SESSION_TX_PHY_LOGGING_ENBL         = 0xF5u,
    R4_SESSION_LOG_PARAMS_CONF             = 0xF6u,
    R4_SESSION_CIR_TAP_OFFSET              = 0xF7u,
    R4_SESSION_CIR_NUM_TAPS                = 0xF8u,
    R4_SESSION_STS_INDEX_RESTART           = 0xF9u,
    R4_SESSION_VENDOR_SPECIFIC_OUI         = 0xFAu,
    R4_SESSION_RADIO_CFG_IDXS              = 0xFBu,
    R4_SESSION_CRYPTO_KEY_USAGE_FLAG       = 0xFDu,
    R4_SESSION_SEND_FINAL_ALWAYS           = 0xFEu,
}
Ranger4Uci_CfgSessionType_t;

typedef struct
{
    Ranger4Uci_CfgSessionType_t             ID;         /*!< The identifier of the session configuration. */
    uint8_t                                 length;     /*!< The length of paramValue */
    uint8_t*                                paramValue; /*!< The value of the configuration parameter */
} Ranger4Uci_SessionCfgStructure_t;


typedef enum
{
    R4_SESSION_RANGING_SESSION                = 0x00u,    
    R4_SESSION_CCC_RANGING_SESSION            = 0xA0u, /*!<Only ccc supported */
    R4_SESSION_CCC_COMPATIBLE_SESSION         = 0xA1u, /*!<For ccc specification 0.1.1*/
    R4_SESSION_CUSTOM_SESSION                 = 0xE0u, /*!<Customer session */
    R4_SESSION_CUSTOM_SESSION_ONLY_RANGING    = 0xE1u, /*!<Customer session only ranging frames*/
    R4_SESSION_DEVICE_TEST                    = 0xD0u,
}Ranger4Uci_SessionType_t;

typedef enum
{
    R4_STATE_CHANGE_WITH_SESSION_MANAGEMENT_COMMANDS = 0x00u,
    R4_MAX_RANGING_ROUND_RETRY_COUNT_REACHED         = 0x01u,
    R4_MAX_RANGING_BLOCKS_REACHED                    = 0x02u,
    R4_URSK_EXPIRED                                  = 0x03u,
    R4_TERMINATION_ON_MAX_STS                        = 0x05u,
}Ranger4Uci_SessionStatusChangeReason_t;

typedef enum
{
    R4_SESSION_STATE_INIT                    = 0x00u,
    R4_SESSION_STATE_DEINIT                  = 0x01u,
    R4_SESSION_STATE_ACTIVE                  = 0x02u,    
    R4_SESSION_STATE_IDLE                    = 0x03u,
    R4_SESSION_ERROR                         = 0xFFu,
}Ranger4Uci_SessionStatusType_t;


//typedef struct
//{
//    uint32_t SessionId;
//    uint8_t status;
//    uint8_t reason;
//}Ranger4Uci_SessionStatus_t;


/** @brief UCI CMD send states */
typedef enum 
{
    TransferState_Idle,
    TransferState_Busy,
//    TransferState_WaitSend,
//    TransferState_WaitResponse
}Ranger4UciCmd_TransferState_t;


typedef struct
{
    phscaR4CadsTypesIntf_UciFrame_t PreviousUciFrame;
    Ranger4UciCmd_TransferState_t state;
}Ranger4UciCmd_TransferStateManagement_t;


typedef void (*Ranger4HandleUciRcvCallback_t) (phscaR4CadsTypesIntf_UciFrame_t *frame);
/* =============================================================================
 * Public Function-like Macros
 * ========================================================================== */

/* =============================================================================
 * Public Standard Enumerators
 * ========================================================================== */

/* =============================================================================
 * Public Function Prototypes
 * ========================================================================== */
/*! ***************************************************************************
 * \brief        Reset the UWBS
 * \param[in]    none   
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacDeviceReset(void);
/*! ***************************************************************************
 * \brief        Retrieve the device specific information and version of  
                 followed specifications.
 * \param[in]    none   
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreGetDeviceInfo(void);
/*! ***************************************************************************
 * \brief        Retrieve the capability of UWBS.
 * \param[in]    none   
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreGetCapsInfo(void);
/*! ***************************************************************************
 * \brief        Retrieve current configuration parameters of the UWBS
 * \param[in]    ParamSize    Number of Parameters 
 * \param[in]    pParamID     The point for identifier of the configuration 
                              parameter 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreGetConfiguration(uint8_t ParamSize, uint8_t * pParamID);
/*! ***************************************************************************
 * \brief        Set configuration parameters for the UWBS
 * \param[in]    ParamSize    Number of Parameters 
 * \param[in]    pParam       The point of the configuration parameters 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreSetConfiguration(uint8_t ParamSize, Ranger4Uci_ParameterStructure_t * pParam);
/*! ***************************************************************************
 * \brief        Session Initiation.
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application.
                              Session ID = 0x0000000: Reserved for Test Mode Session
 * \param[in]    tp           Type of session 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionCfgInit(uint32_t SessionId, Ranger4Uci_SessionType_t tp);

/*! ***************************************************************************
 * \brief        Session DeInitiation.
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application.
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionCfgDeInit(uint32_t SessionId);

/*! ***************************************************************************
 * \brief        Set configuration parameters for UWBS Session
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 * \param[in]    CfgCount     Number of session configurations
 * \param[in]    pParam       Point of session configurations(TLV)
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionConfigure(uint32_t SessionId,
                                                                   uint8_t CfgCount, 
                                                                   Ranger4Uci_SessionCfgStructure_t * pParam);
/*! ***************************************************************************
 * \brief        Get configuration parameters for UWBS Session
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 * \param[in]    CfgCount     Number of configuration 
 * \param[in]    pCfgID       The point of the configuration IDs 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetConfigure(uint32_t SessionId,
                                                                      uint8_t  CfgCount, 
                                                                      uint8_t * pCfgID);

/*! ***************************************************************************
 * \brief        Get the number of the active UWB session 
 * \param[in]    None
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetNumSession(void);

/*! ***************************************************************************
 * \brief        Get the current state of the UWB session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetSessionState(uint32_t SessionId);

/*! ***************************************************************************
 * \brief        Get possible ran multiplier value 
 * \param[in]    None
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetRanMult(void);

/*! ***************************************************************************
 * \brief        Start a UWB ranging session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionStartRanging(uint32_t SessionId);

/*! ***************************************************************************
 * \brief        Stop a UWB ranging session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionStopRanging(uint32_t SessionId);
     
/*! ***************************************************************************
 * \brief        Resume a UWB ranging session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionResumeRanging(uint32_t SessionId, uint32_t StsIndex);
     
/*! ***************************************************************************
 * \brief        Get the number of times ranging 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetRangingCnt(uint32_t SessionId);



//SWUP functions
/*! ***************************************************************************
 * \brief        Activate Software Updater 
 *               In case of success, NCJ29D5D resets after 100 milliseconds without 
 *               response and activates SWUP, or it will response error
 * \param[in]    appType  MAC UCI or nxp UCI  
 * \param[in]    comm_interface_type  
 *               0x00: Select the default SWUP communication interface software(RCI). 
 *               0x01: Select the customer-specific communicationinterface software(UCI)
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciActivate(phscaR4_UciAppType_t appType, uint8_t comm_interface_type);

/*! ***************************************************************************
 * \brief        This command deactivates the SWUP by clearing the activation 
 *               conditions and reboots the device 
 * \param[in]    comm_interface_type  
 *               0x00: Select the default SWUP communication interface software(RCI). 
 *               0x01: Select the customer-specific communicationinterface software(UCI)
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciDeactivate(uint8_t comm_interface_type);


/*! ***************************************************************************
 * \brief        Clears all manifest segments from device
 *               
 * \param[in]    null 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciClearRAMManifest(void);

/*! ***************************************************************************
 * \brief        Transfer manifest
 *               
 * \param[in]    offset: Active section of manifest,0 to 0x03 be used to select 
 *               part of manifest.
 * \param[in]    *data : Point of data part of manifest, 128 bytes
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciTransferManifest(uint8_t offset, uint8_t *data);


/*! ***************************************************************************
 * \brief        Start update 
 *               
 * \param[in]    null
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciStartUpdate(void);

/*! ***************************************************************************
 * \brief        Transfer component data to device
 *               
 * \param[in]    ComponentId:Component index as per defined in the manifest's Component Table
 * \param[in]    SegOffset:  Offset from the component's given start address
 * \param[in]    *data :     Point of data part of component, 128 bytes
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciTransferComponent(uint8_t ComponentId, uint16_t SegOffset, uint8_t *data);

/*! ***************************************************************************
 * \brief        Verify the selected component described in the manifest
 *               
 * \param[in]    ComponentId:Component index as per defined in the manifest's Component Table
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciVerifyComponent(uint8_t componentIndex);

/*! ***************************************************************************
 * \brief        Verify the all components described in the manifest
 *               
 * \param[in]    null
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciVerifyAll(void);

/*! ***************************************************************************
 * \brief        This command checks whether all components in manifest have been
 *               verified.The command provides a negative response if any component
 *               is not verified successfully. Upon success, following scenario 
 *               happens: 1. Erase the manifest in the flash area
 *                        2. The SWUP activation condition is cleared.
 *                        3. Reset.
 * \param[in]    null
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciFinistUpdate(void);

/*! ***************************************************************************
 * \brief        Get device info
 *               Positive Response:
 *                  -Length of RCI response (0x2C)
 *                  -CMD Response 0 (4 Bytes)
 *                  -SWUP STATUS xx (4 Bytes)
 *                  -Product ID xx (8 Bytes)
 *                  -Hardware ID xx (4 Bytes)
 *                  -Typecheck ID xx (8 Bytes)
 *                  -Rom ID xx (8 Bytes)
 *                  -SWUP version xx (8 Bytes)
 * \param[in]    null 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciGetDeviceInfo(void);

/*! ***************************************************************************
 * \brief        Read device ID
 *               Positive Response:
 *                 -Length 0x1C (1 Byte)
 *                 -CMD Response 0 (4 Bytes)
 *                 -SWUP STATUS xx (4 Bytes)
 *                 -Wafer ID xx (11 Bytes)
 *                 -Wafer Number xx (1 Bytes)
 *                 -Wafer X coordinate xx (2 Bytes)
 *                 -Wafer Y coordinate xx (2 Bytes)
 *                 -Serial Number xx (4 Bytes)
 * \param[in]    null 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciReadDeviceID(void);

//SWUP functions end



/*! ***************************************************************************
 * \brief        Query UWB Time Stamp, this command will send synchronously
 * \param[in]    
 *
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_ProprietaryQueryTimeStamp(void);

/*! ***************************************************************************
 * \brief        UWB Sleep
 * \param[in]    
 *
 *****************************************************************************/
//Modify (Ken):NXP-V0001 NO.5 -20240319
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_InotLowPower(void);

#endif

#endif  // for UWB_Support
