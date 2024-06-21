/*
   (c) NXP B.V. 2020-2021. All rights reserved.

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
      INCLUDING WITHOUT LIMITATION, DAMAGES RESULTING OR ALLEGDED TO HAVE
      RESULTED FROM ANY DEFECT, ERROR OR OMMISSION IN THE NXP SOFTWARE/SOURCE
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
//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
//******************************************************************************
//
//******************************************************************************
#ifndef PHSCAR4CADSTYPESINTF_H
#define PHSCAR4CADSTYPESINTF_H

/**
 * @file phscaR4CadsTypesIntf.h
 * @brief R4 CADS data types
 */

/**
 * @defgroup intfs_r4cads_types R4 CADS types
 * @brief Provides common data types used in R4 CADS system.
 * @{
 */

/**
 * @brief NULL pointer
 */
#ifndef __cplusplus
#define PHSCA_R4CADSTYPES_NULL                                      ((void *) 0)
#else
#define PHSCA_R4CADSTYPES_NULL                                      0u
#endif

/**
 * @brief UCI frame size
 */
#define PHSCA_R4CADSTYPES_UCI_FRAME_SIZE                            260u //modified by Jia, 266->260

/*******************************message type************************************/
/**
 * @brief UCI message type command message
 */
#define PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG                            1u

/**
 * @brief UCI message type response message
 */
#define PHSCA_R4CADSTYPES_UCI_MT_CMD_RSP                            2u

/**
 * @brief UCI message type notification message
 */
#define PHSCA_R4CADSTYPES_UCI_MT_CMD_NTF                            3u

/**
 * @brief UCI CADS internal message type
 */
#define PHSCA_R4CADSTYPES_UCI_MT_CADS_INTERNAL                      0x07u


/********************************GID define*************************************/
/**
 * @brief UCI core group identifier
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CORE                              0x00u

/**
 * @brief UCI session config group identifier
 */
#define PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG                    0x01u

/**
 * @brief UCI ranging session control group identifier
 */
#define PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL           0x02u

/**
 * @brief UCI data control group identifier
 */
#define PHSCA_R4CADSTYPES_UCI_GID_DATA_CONTROL                      0x03u

/**
 * @brief UCI reserved for future use group identifier min
 */
#define PHSCA_R4CADSTYPES_UCI_GID_RFU_MIN                           0x04u

/**
 * @brief UCI reserved for future use group identifier max
 */
#define PHSCA_R4CADSTYPES_UCI_GID_RFU_MAX                           0x0Cu

/**
 * @brief UCI test group identifier
 */
#define PHSCA_R4CADSTYPES_UCI_GID_TEST                              0x0Du

/**
 * @brief UCI proprietary group identifier min
 */
#define PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN                   0x0Eu

/**
 * @brief UCI proprietary group identifier max
 */
#define PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MAX                   0x0Fu

/**
 * @brief CADS internal command group identify
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL                     0x0Fu

/********************************OID define*************************************/
/**
 * @brief CADS get version opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_GETVERSION                   0x00u

/**
 * @brief CADS length of command get version
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_GETVERSION               0u

/**
 * @brief CADS reset S32K board opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_RESETS32K                    0x01u

/**
 * @brief CADS length of command reset S32K board
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_RESETS32K                0u

/**
 * @brief CADS hard reset ranger4 opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_HARDRESETR4                  0x02u

/**
 * @brief CADS length of command hard reset ranger4
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_HARDRESETR4              0u

/**
 * @brief CADS set system configuration opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_SETSYSCONFIG                 0x03u

/**
 * @brief CADS length of command set system configuration
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_SETSYSCONFIG             64u

/**
 * @brief CADS get system configuration opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_GETSYSCONFIG                 0x04u

/**
 * @brief CADS length of command get system configuration
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_GETSYSCONFIG             0u

/**
 * @brief CADS set playback opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_SET_PLAYBACK                 0x05u

/**
 * @brief CADS minimum length of command set playback
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_SET_PLAYBACK_MIN         6u

/**
 * @brief CADS get playback opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_GET_PLAYBACK                 0x06u

/**
 * @brief CADS minimum length of command get playback
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_GET_PLAYBACK             2u

/**
 * @brief CADS store playback opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_STORE_PLAYBACK               0x07u

/**
 * @brief CADS minimum length of command store playback
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_STORE_PLAYBACK           1u

/**
 * @brief CADS enable/disable CRC opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_CRCENABLE                    0x08u

/**
 * @brief CADS length of command enable/disable CRC
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_CRCENABLE                1u

/**
 * @brief CADS configuration wake-up pin opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_CONFIGWAKEUPPIN              0x09u

/**
 * @brief CADS length of command configuration wake-up pin
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_CONFIGWAKEUPPIN          25u

/**
 * @brief CADS wake-up ranger4 opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_WAKEUPR4                     0x0Au

/**
 * @brief CADS length of command wake-up ranger4
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_WAKEUPR4                 3u

/**
 * @brief CADS enable/disable watchdog opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_WATCHDOGENABLE               0x0Bu

/**
 * @brief CADS length of command enable/disable watchdog
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_WATCHDOGENABLE           1u

/**
 * @brief CADS set timeout watchdog opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_SETWDOGTIMEOUT               0x0Cu

/**
 * @brief CADS length of command set watchdog timeout
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_SETWDOGTIMEOUT           4u

/**
 * @brief CADS get range Watchdog timeout
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_GETRANGE_WDOGTIMEOUT         0x0Du

/**
 * @brief CADS length of command get range Watchdog timeout
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_GETRANGE_WDOGTIMEOUT     0u

/**
 * @brief Watchdog timeout min in millisecond unit
 */
#define PHSCA_R4CADSTYPES_WATCHDOG_TIMEOUT_MIN                      2u

/**
 * @brief Watchdog timeout max in millisecond unit
 */
#define PHSCA_R4CADSTYPES_WATCHDOG_TIMEOUT_MAX                      ((uint32_t)((uint32_t)0xFFFFu<<1u))

/**
 * @brief CADS GPIO toggling opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_GPIO_TOGGLING                0x0Eu

/**
 * @brief CADS length of command GPIO toggling
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_GPIO_TOGGLING            2u

/**
 * @brief CADS get BoardID
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_GET_BOARDID                  0x0Fu

/**
 * @brief CADS length of command get BoardID
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_GET_BOARDID              0u

/**
 * @brief CADS erase all stored playback opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_ERASE_PLAYBACK               0x10u

/**
 * @brief CADS length of command erase all stored playback
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_ERASE_PLAYBACK           1u

/**
 * @brief CADS start playback opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_START_PLAYBACK               0x11u

/**
 * @brief CADS length of command start playback
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_START_PLAYBACK           3u

/**
 * @brief CADS stop playback opcode identifier
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_STOP_PLAYBACK                0x12u

/**
 * @brief CADS length of command stop playback
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_STOP_PLAYBACK            0u

/**
 * @brief CADS generic error notify
 */
#define PHSCA_R4CADSTYPES_UCI_OID_CADS_GENERIC_ERROR                0x3Fu

/**
 * @brief CADS length of CADS generic error notify
 */
#define PHSCA_R4CADSTYPES_UCI_GID_CADS_LEN_GENERIC_ERROR            1u

/**
 * @brief Default playback mode
 */
#define PHSCA_R4CADSTYPES_PLAYBACK_MODE_DEFAULT                     0u

/**
 * @brief Initiator playback mode
 */
#define PHSCA_R4CADSTYPES_PLAYBACK_MODE_INITIATOR                   1u

/**
 * @brief Responder playback mode
 */
#define PHSCA_R4CADSTYPES_PLAYBACK_MODE_RESPONDER                   2u

/**
 * @brief Ranger4 UCI command playback mode
 */
#define PHSCA_R4CADSTYPES_PLAYBACK_MODE_R4UCICMD                    3u

/**
 * @brief Maximum number of modes
 */
#define PHSCA_R4CADSTYPES_PLAYBACK_MAX_NUMBER_MODE                  3u

/**
 * @brief Playback frame size
 */
#define PHSCA_R4CADSTYPES_PLAYBACK_FRAME_SIZE                       260u

/**
 * @brief Maximum number of UCI commands in playback mode: Ranger4 UCI command
 */
#define PHSCA_R4CADSTYPES_PLAYBACK_MAX_NUMBER_UCI_CMD               30u

/**
 * @brief Start address memory for system configuration
 */
#define PHSCA_R4CADSTYPES_START_ADDRESS_MEMORY_SYSTEMCONFIG         (FEATURE_FLS_PF_BLOCK_SIZE - (1u * FEATURE_FLS_PF_BLOCK_SECTOR_SIZE))

/**
 * @brief Start address memory for playback mode: Initiator
 */
#define PHSCA_R4CADSTYPES_START_ADDRESS_MEMORY_PLAYBACK_INITIATOR   (FEATURE_FLS_PF_BLOCK_SIZE - (2u * FEATURE_FLS_PF_BLOCK_SECTOR_SIZE))

/**
 * @brief Start address memory for playback mode: Responser
 */
#define PHSCA_R4CADSTYPES_START_ADDRESS_MEMORY_PLAYBACK_RESPONDER   (FEATURE_FLS_PF_BLOCK_SIZE - (3u * FEATURE_FLS_PF_BLOCK_SECTOR_SIZE))

/**
 * @brief Start address memory for playback mode: Ranger4 UCI command
 */
#define PHSCA_R4CADSTYPES_START_ADDRESS_MEMORY_PLAYBACK_R4UCICMD    (FEATURE_FLS_PF_BLOCK_SIZE - (5u * FEATURE_FLS_PF_BLOCK_SECTOR_SIZE))


/**
 * @brief CADS UCI status code
 */
typedef enum
{
    /** @brief Success */
    uciStatusCode_Ok = 0x00u,
    /** @brief Intended operation is not supported in the current state */
    uciStatusCode_Rejected = 0x01u,
    /** @brief Intended operation failed to complete */
    uciStatusCode_Failed = 0x02u,
    /** @brief UCI packet structure is not per spec */
    uciStatusCode_Syntax_Error = 0x03u,
    /** @brief Config ID is correct, and value is not specified */
    uciStatusCode_Invalid_Param = 0x04u,
    /** @brief Config ID is correct, and value is not in proper range */
    uciStatusCode_Invalid_Range = 0x05u,
    /** @brief UCI packet payload size is not as per spec */
    uciStatusCode_Invalid_Message_Size = 0x06u,
    /** @brief UCI Group ID is not per spec */
    uciStatusCode_Unknown_Gid = 0x07u,
    /** @brief UCI opcode ID is not per spec */
    uciStatusCode_Unknown_Oid = 0x08u,
    /** @brief Config ID is read-only */
    uciStatusCode_Read_Only = 0x09u,
    /** @brief UWBS request retransmission from AP */
    uciStatusCode_Command_Retry = 0x0Au,

    /** @brief Session does not exist (is not created) */
    uciStatusCode_Error_Session_Not_Exist = 0x11u,
    /** @brief Session exists (is already created) */
    uciStatusCode_Error_Session_Duplicate = 0x12u,
    /** @brief Session is active */
    uciStatusCode_Error_Session_Active = 0x13u,
    /** @brief Max number of sessions already created */
    uciStatusCode_Error_Max_Sessions_Exceeded = 0x14u,
    /** @brief Session is not configured with required app configurations */
    uciStatusCode_Error_Session_Not_Configured = 0x15u,
    /** @brief Sessions are actively running in UWBS */
    uciStatusCode_Error_Active_Session_Ongoing = 0x16u,
    /** @brief Slot length is not supported in the requested configuration */
    uciStatusCode_Slot_Len_Not_Supported = 0x1Au,
    /** @brief Number of slots is invalid in the requested configuration */
    uciStatusCode_Invalid_Slot_Per_Rr = 0x1Bu,

    /** @brief Failed to transmit UWB packet */
    uciStatusCode_Ranging_Tx_Failed = 0x20u,
    /** @brief No UWB packet detected by the receiver */
    uciStatusCode_Ranging_Rx_Timeout = 0x21u,
    /** @brief UWB packet channel decoding error */
    uciStatusCode_Ranging_Rx_Phy_Dec_Failed = 0x22u,
    /** @brief Failed to detect time of arrival of the UWB packet from CIR samples */
    uciStatusCode_Ranging_Rx_Phy_Toa_Failed = 0x23u,
    /** @brief UWB packet STS segment mismatch */
    uciStatusCode_Ranging_Rx_Phy_Sts_Failed = 0x24u,
    /** @brief MAC CRC or syntax error */
    uciStatusCode_Ranging_Rx_Mac_Dec_Failed = 0x25u,
    /** @brief IE syntax error */
    uciStatusCode_Ranging_Rx_Mac_Ie_Dec_Failed = 0x26u,
    /** @brief Expected IE missing in the packet */
    uciStatusCode_Ranging_Rx_Mac_Ie_Missing = 0x27u,
    
    uciStatusCode_Unspecified_Error = 0xFFu,  //add by jia
} phscaR4CadsTypesIntf_UciStatusCode_t;

/**
 * @brief R4 CADS Error code
 */
typedef enum
{
    /**
     * @brief Generic error code definition with the first byte value
     * 0x00000000u : 0x000000FFu
     */
    /** @brief Success */
    errorCode_Success = 0x00000000u,
    /** @brief Value is not in proper range */
    errorCode_InvalidRange = 0x00000001u,
    /** @brief Generic error */
    errorCode_GenericError = 0x00000002u,
    /** @brief Spi bus busy error */
    errorCode_SpiBusBusy = 0x00000003u,
    /** @brief Undefined error */
    errorCode_Undefined = 0x000000FFu,

    /**
     * @brief Interpreter error code definition with the second byte value
     * 0x00000100u : 0x0000FF00u
     */
    /** @brief No message received */
    errorCode_InterpreterNoMessage = 0x00000100u,
    /** @brief Interpreter new command */
    errorCode_InterpreterNewCommand = 0x00000200u,
    /** @brief Interpreter already triggered */
    errorCode_InterpreterAlreadyTriggered = 0x00000300u,
    /** @brief Interpreter ready */
    errorCode_InterpreterReadyToNotify = 0x00000400u,
    /** @brief Interpreter busy */
    errorCode_InterpreterBusyToNotify = 0x00000500u,

    /**
     * @brief Driver error code definition with the third byte value
     * 0x00010000u : 0x00FF0000u
     */
    /** @brief Driver initialize error */
    errorCode_DriverInitError = 0x00010000u,
    /** @brief Driver De-initialize error */
    errorCode_DriverDeInitError = 0x00020000u,
    /** @brief Driver busy error */
    errorCode_DriverBusyError = 0x00030000u,
    /** @brief Driver communication error */
    errorCode_DriverComError = 0x00040000u,
    /** @brief Driver timeout error */
    errorCode_DriverTimeoutError = 0x00050000u,
    /** @brief Driver Flash write error */
    errorCode_DriverFlashWriteError = 0x00060000u,
    /** @brief Driver Flash erase error */
    errorCode_DriverFlashEraseError = 0x00070000u,
    /** @brief Driver CRC verify error */
    errorCode_DriverCrcError = 0x00080000u,
    /** @brief Driver unspecified error */
    errorCode_DriverGenericError = 0x00FF0000u,

    /**
     * @brief UCI error code definition with the fourth byte value
     * 0x01000000u : 0xFF000000u
     */
    /** @brief UCI send successfully */
    errorCode_UciSendSuccess = 0x01000000u,
    /** @brief UCI receive successfully */
    errorCode_UciRecvSuccess = 0x02000000u,
    /** @brief UCI no event */
    errorCode_UciNoEvent = 0x03000000u,
    /** @brief UCI busy error */
    errorCode_UciBusyError = 0x04000000u,
    /** @brief UCI communication error */
    errorCode_UciComError = 0x05000000u,
    /** @brief UCI ranger4 not ready error */
    errorCode_UciR4NotReadyError = 0x06000000u,
    /** @brief UCI ranger4 not set RDY high after setting CS high */
    errorCode_UciR4NotSetRdy = 0x07000000u,
    /** @brief UCI ranger4 not set INT high after setting CS high */
    errorCode_UciR4NotSetIrq = 0x08000000u,
    /** @brief UCI communication error */
    errorCode_UciGenericError = 0xFF000000u
} phscaR4CadsTypesIntf_ErrorCode_t;

/**
 * @brief UCI header structure
 */
typedef struct
{
    /** @brief UCI Group identifier */
    uint8_t groupId :4;
    /** @brief UCI boundary flag */
    uint8_t boundaryFlag :1;
    /** @brief UCI message type */
    uint8_t msgType :3;
    /** @brief UIC Opcode identifier */
    uint8_t opcodeId :6;
    /** @brief UCI packet header RFU */
    uint8_t :2;
    /** @brief UCI packet header RFU */
    uint8_t :8;
    /** @brief UIC packet payload length */
    uint8_t payloadLength :8;
} phscaR4CadsTypesIntf_UciHeader_t;

/**
 * @brief UCI packet structure
 */
typedef struct
{
    /** @brief UCI Header */
    phscaR4CadsTypesIntf_UciHeader_t header;
    /** @brief UCI payload */
    uint8_t payload[PHSCA_R4CADSTYPES_UCI_FRAME_SIZE - 4u];
} phscaR4CadsTypesIntf_UciPacket_t;

/**
 * @brief UCI frame
 */
typedef union
{
    /** @brief Access data byte-wise */
    uint8_t bytes[PHSCA_R4CADSTYPES_UCI_FRAME_SIZE];
    /** @brief Access UCI packet format */
    phscaR4CadsTypesIntf_UciPacket_t uciPacket;
} phscaR4CadsTypesIntf_UciFrame_t;

/** @brief Structure for playbackstates */
typedef enum
{
    playbackState_Idle = 0x00u,
    playbackState_Init = 0x01u,
    playbackState_Running = 0x02u,
    playbackState_Finished = 0x03u,
    playbackState_LoopInSwTimer = 0x04u,
    playbackState_Error = 0xFEu,
    playbackState_Undefined = 0xFFu,
} phscaR4CadsTypesIntf_PlaybackState_t;

/** @brief Handle states */
typedef enum
{
    appConnectedSubState_Normal = 0x00u,
    appConnectedSubState_WaitSoftReset = 0x01u,
    appConnectedSubState_WaitPlaybackStart = 0x02u,
    appConnectedSubState_WaitPlaybackStop = 0x03u,
} phscaR4CadsTypesIntf_AppConnectedSubState_t;

/** @brief Application handle response */
typedef enum
{
    appDispatchCode_None = 0x00u,
    appDispatchCode_PlaybackEnabled = 0x01u,
    appDispatchCode_UciCommunicationError = 0x02u,
    appDispatchCode_InterpCommunicationError = 0x03u,
    appDispatchCode_PlaybackNotifyError = 0x04u,
    appDispatchCode_PlaybackDisabled = 0x05u,
    appDispatchCode_CriticalError = 0x06u,
    appDispatchCode_Undefine = 0xFFu,
} phscaR4CadsTypesIntf_AppDispatchCode_t;

/**
 * @brief UCI app or MAC app
 */
typedef enum
{
    /** @brief support UCI application   */
    UciAppType_UciAppSupport,
    /** @brief support MAC application */
    UciAppCode_MacAppSupport
}phscaR4_UciAppType_t;


/** @} */

#endif /* PHSCAR4CADSTYPESINTF_H */
/* EOF */


#endif  // for UWB_Support

