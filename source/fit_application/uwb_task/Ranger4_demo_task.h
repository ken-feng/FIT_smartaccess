//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
//******************************************************************************
//
//******************************************************************************
#ifndef __RANGER4_SWUP_DEMO_TASK_H__
#define __RANGER4_SWUP_DEMO_TASK_H__


//#include "phscaR4_RCICmd.h"
#include "phscaR4UciSpi.h"
#include "Ranger4UciCmd.h"
/*
 * \brief   Configures the UWB task priority.
 */
#ifndef gRanger4TaskPriority_c
#define gRanger4TaskPriority_c                7U  //UWB task priority 
#endif

/* Defines Size for SE process Task*/
#ifndef gRanger4TaskStackSize_c
#define gRanger4TaskStackSize_c               1800
#endif

/*** Ranger4 Application Events ***/
/* Applicaiton Events */
#define gR4EvtUwb2HostMsgAvailable_c        (1U << 0U)
#define gR4EvtTransferCmdToR4Event_c        (1U << 1U)
#define gR4EvtUwb2HostMsgReceivedEvent_c    (1U << 2U)
/* SWUP Events */
#define gR4EvtNewPacketFromUart_c           (1U << 11U)
#define gR4EvtNewPacketMsgFromBLE_c         (1U << 12U)
#define gR4EvtMsgPacketFromSPI_c            (1U << 13U)
//#define gR4EvtMsgFromCAN_c                  (1U << 14U)
#define gR4EvtSelfNotify_c                  (1U << 15U)

#ifndef MaxSessionSize
#define MaxSessionSize  1  //this size can be define by user
#endif 

#define SWUP_UART_HEADER_SIZE                 3u  
#define SWUP_UART_PAYLOAD_MAX_SIZE            134u  //check the swup uart for detail
#define SWUP_UART_CRC_SIZE                    2u
#define SWUP_UART_CMD_SIZE                    1u
#define SWUP_UART_CMDTYPE_SIZE                1u

#define SWUP_UART_CMD_PC2KW_MARKER            0xE5
#define SWUP_UART_CMD_KW2PC_MARKER            0x5E

     
#define SWUP_PKT_SIZE                         128u     


#define CCC_MIN_BLOCK                         96u
#define CCC_TCHAP                             400u

/** @brief Ranger4 application process status */
typedef enum
{
    Ranger4App_Init                 = 0x00u,
    Ranger4App_Running              = 0x01u,
    Ranger4App_HardwareResetDevice  = 0x02u,
    Ranger4App_DeviceSWUP           = 0x03u,
    Ranger4App_UciCommunicationError= 0x04u,
    Ranger4App_CriticalError        = 0x05u,
    Ranger4App_UndefinedError       = 0xFFu
} Ranger4App_Status_t; 


/** @brief Ranger4 Running process status */
typedef enum
{
    Ranger4AppRunning_Init                 = 0x00u,
    Ranger4AppRunning_Running              = 0x01u,
    Ranger4AppRunning_HardwareResetDevice  = 0x02u,
    Ranger4AppRunning_DeviceSWUP           = 0x03u,
    Ranger4AppRunning_UciCommunicationError= 0x04u,
    Ranger4AppRunning_CriticalError        = 0x05u,
    Ranger4AppRunning_UndefinedError       = 0xFFu
} Ranger4AppRunning_Status_t; 




#if 0 //Jia
/** @brief Uart check status */    
typedef enum 
{
    Packet_Rcv_Ok,
    Packet_Rcv_Not_Complete,
    Packet_CrcError,
    Packet_Unspecified_Error
} R4Swup_PacketRcvStatus_t;  

/** @brief Ranger4 SWUP process status */
typedef enum
{
    R4_SWUP_Idle,
    R4_SWUP_StartProcess,
    R4_SWUP_ActivationSwupComplete,
    R4_SWUP_TransferManifest,
    R4_SWUP_StartUpdate,
    R4_SWUP_TransferComponent,
    R4_SWUP_VerifyComponent,
    R4_SWUP_FinishUpdate,
    R4_SWUP_ResetDevice
} R4Swup_Status_t; 

/** @brief RCI/UCI SWUP select */
typedef enum 
{
    R4SWUP_SPI_PROTOCOL_RCI,
    R4SWUP_SPI_PROTOCOL_UCI
}R4Swup_SpiProtocolType_t;

/** @brief NCJ29D5 SWUP status set */
typedef enum
{
    /* SWUP status code */
    PHSCAR4_SWUP_STATUS_INVALID             = 0x00u,
    PHSCAR4_SWUP_STATUS_INIT                = 0x01u,
    PHSCAR4_SWUP_STATUS_ACTIVE              = 0x02u,
    PHSCAR4_SWUP_STATUS_TRANSFER            = 0x03u,
    PHSCAR4_SWUP_STATUS_ERROR               = 0x04u
} phscaR4_SwupStatus_t;

typedef struct 
{
    uint8_t CmdHead;
    uint8_t CmdLen[2];
    uint8_t CmdType;
    uint8_t Cmd;
    uint8_t payload[SWUP_UART_PAYLOAD_MAX_SIZE];
    uint8_t RFU; // just for align 4 bytes.
}R4Swup_SerialCmdPkt_t;

typedef union
{
    uint8_t raw_data[sizeof(R4Swup_SerialCmdPkt_t)];
    R4Swup_SerialCmdPkt_t pkt;  
}R4Swup_SerialCmdPacket_t;

typedef struct
{
    R4Swup_SerialCmdPacket_t  Pkt;
    uint16_t                  bytesReceived;
    uint16_t                  bytesExpect;
}R4Swup_SerialComm_t;


typedef struct
{
    uint8_t                   ComponentCount;
    uint8_t                   ComponentId;
    uint16_t                  PacketdataSize;
    uint32_t                  ImageSize;
    uint32_t                  totalpackets;
    uint32_t                  ComponentSize[8];
    uint32_t                  ComponentAddress[8];
    uint16_t                  ComponentPkts;
    uint16_t                  CurrentComponentPktIndex;
    uint32_t                  SwupStatus;
    uint16_t                  ReqPktNum;
    uint16_t                  RcvPktNum;
}R4Swup_UpdateImgStruct_t;

typedef union
{
    phscaR4_RciHwDeviceInfo_t     Rci_Info;
    phscaR4_UciHwDeviceInfo_t     Uci_Info;
}phscaR4_HwDeviceInfo_t;
#endif  

typedef struct 
{
    uint8_t    Slot_bitMask;
    uint8_t    SYNC_Code_Index_Bitmaks[4];
    uint8_t    Hopping_Config_Bitmask;  
    uint8_t    Channel_BitMask;
    uint8_t    Supported_Protocol_Version[2];    //for CCC3.0 only version 1.0 
    uint8_t    ProtoVer_Len;
    uint8_t    Supported_Cfg_Id[4];              // 0x0000, 0x0001
    uint8_t    CfgId_Len;
    uint8_t    Supported_Pluseshape_Combo[9];
    uint8_t    PluseshapeCombo_Len;
} UwbCapabilityManagement_t;

typedef struct 
{
    uint32_t   UwbSessionID;
    Ranger4Uci_SessionCfgStructure_t * pSessionfixedCfg;
    Ranger4Uci_SessionType_t SessionType;         //should use CCC type
    uint8_t    SessionSetCfgSize;
    uint8_t    SessionGetCfgSize;
    uint8_t    SessionStatus;                     //define in Ranger4Uci_SessionStatusType_t
    uint8_t    SessionStatusChangeReason;         //define in Ranger4Uci_SessionStatusChangeReason_t
    uint8_t    SessionHopingMode;
    uint16_t   SessionChapsperSlot;
    uint32_t   SessionRangingBlockT;
    
    uint8_t    SelectedDkProtolVersion[2];
    uint16_t   SelectedUwbConfigId;               //0 or 1
    uint8_t    SelectedPulseShapeCombo;           //Select Channel by CCC device
    uint8_t    RAN_Multiplier;                    //In CCC Spec, ranging interval(ranging round) =  96ms * RAN_Multiplier
    uint8_t    SelectedChannel;                   //Channel 5 or 9 
    uint8_t    Number_Chaps_per_Slot;             //Slot duration depends on this variable, slot duration = 400 * Number_Chaps_per_Slot (3) = 1ms
    uint8_t    Number_Responders_Nodes;           //Maximum 24 of Number_Responders_Nodes are supported by UWB chip
    uint8_t    Number_Slots_per_Round;            //Number of Slots in one Ranging Round, SLOTS_PER_RR >= Number_Responders_Nodes + 4
    uint8_t    Selected_Hopping_Config_Bitmask;   //SelectedHopping mode
    uint8_t    Selected_SYNC_Code_Index;          //Selected SYNC Preamble code 
    uint32_t   STS_Index0;                        //STS Index
    uint32_t   HOP_Mode_Key;                      //
    uint64_t   UWB_Time0;                         //
} SessionManagement_t;


void Ranger4App_RegisterCallback(Ranger4HandleUciRcvCallback_t  RspCallback,
                                 Ranger4HandleUciRcvCallback_t  NtfCallback);
void Ranger4App_task_Init( void );
void Ranger4App_InitDefaultSessionCfg(SessionManagement_t * pSessionCfg);
//void Ranger4App_RspCallback(phscaR4CadsTypesIntf_UciFrame_t * frame);
//void Ranger4App_NtfCallback(phscaR4CadsTypesIntf_UciFrame_t * frame);


#if 0
void R4SWUP_RxTimoutCallback(void *param);
R4Swup_PacketRcvStatus_t R4SWUP_checkPacket(R4Swup_SerialComm_t *pData, uint8_t * data, uint8_t len);
bool R4SWUP_SetProtocolType(R4Swup_SpiProtocolType_t type);
uint32_t R4SWUP_PostRcvPktToSwapTask( uint8_t * pRxData, uint32_t pktLen );
#endif

#endif

#endif  // for UWB_Support

