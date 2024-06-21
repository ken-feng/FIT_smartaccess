
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
#include "EmbeddedTypes.h"
#include "Ranger4UciCmd.h"
#include "Ranger4_demo_task.h"

//******************************************************************************
//
//******************************************************************************
//Modify (Ken):NXP-V0001 NO.3 -20240319
#ifdef  __UWB_ROLE_RESPONDER_H
    //--------------------------------------------------------------------------
    #if (ANCHOR_INDEX + 1) > NUM_ANCHORS
        #error("Wrong Anchor Index");
    #endif
    //--------------------------------------------------------------------------
    #if NUM_ANCHORS > 4
        #error("The UWB configuration in this demo only support 4 anchors")
    #endif
    //--------------------------------------------------------------------------
    #define IS_ANCHOR                               1
    //--------------------------------------------------------------------------
#elif defined   __UWB_ROLE_INITIATOR_H
    #define IS_ANCHOR                               0
#else
    #error("The UWB configuration no role")
#endif

//******************************************************************************
/*** Ranger4 Session default configurations ***/
//******************************************************************************
/* configuration that will be changed when negotiate UWB parameter via BLE*/
//static uint8_t UwbSessionCfg_UwbCfgId[2] = {0x00,0x00};
//static uint8_t UwbSessionCfg_PulesShapeCombo[1] = {0x11};
//static uint8_t UwbSessionCfg_RangingInterval[4] = {0x60, 0x00, 0x00, 0x00};//96ms
//static uint8_t UwbSessionCfg_ChannelId[1] = {0x09};
//static uint8_t UwbSessionCfg_RangingSlotLength[2] = {0xB0, 0x04};//1200 RSTU = 1ms
//static uint8_t UwbSessionCfg_NumAnchors[1] = {NUM_ANCHORS};
//static uint8_t UwbSessionCfg_SlotsPerRangingRound[1] = {0x0C};
//static uint8_t UwbSessionCfg_HoppingMode[1] ={0x00};       //no hopping
//static uint8_t UwbSessionCfg_PreambleId[1] = {0x0A};
//static uint8_t UwbSessionCfg_STSIndex[4] = {0x00,0x00,0x00,0x00};

/* configuration that used as default fixed*/
static uint8_t UwbSessionCfg_SfdId[1] = {0x00};
static uint8_t UwbSessionCfg_KeyRotation[1] = {0x03};      //STS key
static uint8_t UwbSessionCfg_Max_RR_Retry[2] = {0x14,0x00};//
static uint8_t UwbSessionCfg_HopModeKey[16] ={0x73, 0x05, 0x8F, 0xE5, 0x50, 0x3C, 0x1D, 0x4A, 0x17, 0x54, 0x1D, 0x51, 0x21, 0xBB, 0x7F, 0x9E};
static uint8_t UwbSessionCfg_MaxBlockNumber[2] = {0xFF, 0xFF};
static uint8_t UwbSessionCfg_UrskTtl[2] = {0xD0, 0x02}; //720minutes
static uint8_t UwbSessionCfg_SaltedHash[18] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x43, 0x54, 0x53, 0x53, 0x54, 0x41, 0x54};
static uint8_t UwbSessionCfg_D_Ursk[18] = {0x04, 0x00, 0x49, 0x43, 0x54, 0x53, 0x53, 0x54, 0x41, 0x54, 0x49, 0x43, 0x54, 0x53, 0x53, 0x54, 0x41, 0x54};
static uint8_t UwbSessionCfg_N[1] = {0};
static uint8_t UwbSessionCfg_TxPower[1] = {0x14}; //Tx power = 14dB - 0.25 * 20(0x14)= 9dB
static uint8_t UwbSessionCfg_TxRxRadioCfg[4] = {0x02, 0x03, 0x12, 0x13};
static uint8_t UwbSessionCfg_CryptoKeyUsageFlag[2] = {0xFF, 0x01};
#if IS_ANCHOR
static uint8_t UwbSessionCfg_DeviceRole[1] = {0x00};    //Responder
static uint8_t UwbSessionCfg_ResponderSlotIndex[1] = {ANCHOR_INDEX}; //slot 0-255
static uint8_t UwbSessionCfg_DevMacAddr[2] = {0x00,0x01};
#else
static uint8_t UwbSessionCfg_DeviceRole[1] = {0x01};    //Initiator
#endif

const Ranger4Uci_SessionCfgStructure_t UwbSessionCfg[__InitSessionCfg_Size] =
{

#if IS_ANCHOR       
    {
        .ID = R4_SESSION_DEVICE_MAC_ADDRESS,
        .length = 0x02,
        .paramValue = UwbSessionCfg_DevMacAddr
    },
#endif    
    
    {
        .ID = R4_SESSION_DEVICE_ROLE,
        .length = 0x01,
        .paramValue = UwbSessionCfg_DeviceRole
    },   
    
    {
        .ID = R4_SESSION_SFD_ID,
        .length = 0x01,
        .paramValue = UwbSessionCfg_SfdId
    },    
#if IS_ANCHOR    
    {
        .ID = R4_SESSION_RESPONDER_SLOT_INDEX,
        .length = 0x01,
        .paramValue = UwbSessionCfg_ResponderSlotIndex
    },
#endif  
    {
        .ID = R4_SESSION_KEY_ROTATION,
        .length = 0x01,
        .paramValue = UwbSessionCfg_KeyRotation
    }, 
    
    {
        .ID = R4_SESSION_MAX_RR_RETRY,
        .length = 0x02,
        .paramValue = UwbSessionCfg_Max_RR_Retry
    }, 

    {
        .ID = R4_SESSION_HOP_MODE_KEY,
        .length = 0x10,
        .paramValue = UwbSessionCfg_HopModeKey
    },    
    
    {
        .ID = R4_SESSION_MAX_NUMBER_OF_MEASUREMENTS,
        .length = 0x02,
        .paramValue = UwbSessionCfg_MaxBlockNumber
    },     

    {
        .ID = R4_SESSION_URSK_TTL,
        .length = 0x02,
        .paramValue = UwbSessionCfg_UrskTtl
    },  
    
    {
        .ID = R4_SESSION_STATIC_KEYS,
        .length = 0x12,
        .paramValue = UwbSessionCfg_SaltedHash
    },      
    
    {
        .ID = R4_SESSION_STATIC_KEYS,
        .length = 0x12,
        .paramValue = UwbSessionCfg_D_Ursk
    },      
    
    {
        .ID = R4_SESSION_N,
        .length = 0x01,
        .paramValue = UwbSessionCfg_N
    },     

    {
        .ID = R4_SESSION_TX_POWER_ID,
        .length = 0x01,
        .paramValue = UwbSessionCfg_TxPower
    },
    
    {
        .ID = R4_SESSION_RADIO_CFG_IDXS,
        .length = 0x04,
        .paramValue = UwbSessionCfg_TxRxRadioCfg
    },
    
    {
        .ID = R4_SESSION_CRYPTO_KEY_USAGE_FLAG,
        .length = 0x02,
        .paramValue = UwbSessionCfg_CryptoKeyUsageFlag
    },
};


#endif
