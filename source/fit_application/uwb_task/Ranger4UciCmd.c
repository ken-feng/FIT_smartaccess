//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
/*! *********************************************************************************
* Include
********************************************************************************** */
#include "fsl_common.h"
#include "fsl_os_abstraction.h"

#include "MemManager.h"
#include "Messaging.h"

#include "phscaR4CadsTypesIntf.h"  
#include "phscaR4UciSpi.h"
#include "Ranger4UciCmd.h"
#include "Ranger4_demo_task.h" 


/*! *********************************************************************************
* Public memory declarations
********************************************************************************** */
extern osaEventId_t                            mRanger4ThreadEventId;
extern anchor_t                                mRanger4SendPktMsgQueue;
extern anchor_t                                mRanger4RcvPktMsgQueue;
/*! *********************************************************************************
* Private macros
********************************************************************************** */



/*! *********************************************************************************
* Private function declarations
********************************************************************************** */

static phscaR4CadsTypesIntf_ErrorCode_t Ranger4_UciExecuteCommand(Ranger4Pkt_SendMode_t mode, phscaR4CadsTypesIntf_UciFrame_t * frame)
{
    phscaR4CadsTypesIntf_ErrorCode_t status0 = errorCode_Success;
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    assert(frame != NULL);
    
    if(mode == R4Pkt_SendAsync)
    {
        /* Put packet in the mRanger4SendPktMsgQueue queue to send*/                                
        (void)MSG_Queue(&mRanger4SendPktMsgQueue, frame);
        (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtTransferCmdToR4Event_c);  
    }
    else if(mode == R4Pkt_SendSync)
    {
        status0 = phscaR4UciSpi_SendData(frame);
  
        while(status0 == errorCode_UciBusyError)
        {
            //There is an othre packet avaliable in UWB, data is not sent
            //try get data packet
            pUciCmdPkt = MEM_BufferAlloc(PHSCA_R4CADSTYPES_UCI_FRAME_SIZE); 
            status0 = phscaR4UciSpi_ReceiveData(pUciCmdPkt);
            if(status0 == errorCode_Success)
            {
                /* process receive data */
                /* Put packet in the mRanger4SendPktMsgQueue queue to send*/                                
                (void)MSG_Queue(&mRanger4RcvPktMsgQueue, pUciCmdPkt);
                (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtUwb2HostMsgReceivedEvent_c);  
            }
            else
            {
                (void)MSG_Free(pUciCmdPkt);
            }
            //Try send data again
            status0 = phscaR4UciSpi_SendData(frame); 
        }
        
        (void)MSG_Free(frame);
    }
    return status0;
}
/* =============================================================================
 * Public Function Definitions
 * ========================================================================== */
/*! ***************************************************************************
 * \brief        Reset the UWBS
 * \param[in]    none   
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacDeviceReset(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    // reset cmd is 5 bytes
    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 1); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CORE;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_CORE_OID_RESET_DEVICE;
        pUciCmdPkt->uciPacket.header.payloadLength = 1;
        pUciCmdPkt->uciPacket.payload[0] = 0;                               

        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined;
        
    }
        
    return en_Status;
}

/*! ***************************************************************************
 * \brief        Retrieve the device specific information and version of  
                 followed specifications.
 * \param[in]    none   
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreGetDeviceInfo(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CORE;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_CORE_OID_CORE_GET_DEVICE_INFO;
        pUciCmdPkt->uciPacket.header.payloadLength = 0;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;    
}

/*! ***************************************************************************
 * \brief        Retrieve the capability of UWBS.
 * \param[in]    none   
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreGetCapsInfo(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CORE;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_CORE_OID_CORE_GET_CAPS_INFO;
        pUciCmdPkt->uciPacket.header.payloadLength = 0;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;    
}


/*! ***************************************************************************
 * \brief        Retrieve current configuration parameters of the UWBS
 * \param[in]    ParamSize    Number of Parameters 
 * \param[in]    pParamID     The point for identifier of the configuration 
                              parameter 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreGetConfiguration(uint8_t ParamSize, uint8_t * pParamID)
{
    uint8_t i;
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + ParamSize + 1); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CORE;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_CORE_OID_CORE_GET_CONFIG;
        pUciCmdPkt->uciPacket.header.payloadLength = ParamSize + 1;
        pUciCmdPkt->uciPacket.payload[0] = ParamSize;
        for(i = 0; i < ParamSize; i++)
        {
            pUciCmdPkt->uciPacket.payload[i + 1] = pParamID[i];
        }
        
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;    
}

/*! ***************************************************************************
 * \brief        Set configuration parameters for the UWBS
 * \param[in]    ParamSize    Number of Parameters 
 * \param[in]    pParam       The point of the configuration parameters (TLV)
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacCoreSetConfiguration(uint8_t ParamSize, Ranger4Uci_ParameterStructure_t * pParam)
{
    uint8_t i,j, paramOffset;
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_UciFrame_t UciCmdPkt;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    memset(UciCmdPkt.bytes, 0, PHSCAR4_UCI_HEADER_LENGTH);
    UciCmdPkt.uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
    UciCmdPkt.uciPacket.header.boundaryFlag = 0;
    UciCmdPkt.uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CORE;
    UciCmdPkt.uciPacket.header.opcodeId = R4_UCI_CORE_OID_CORE_SET_CONFIG;
    //UciCmdPkt.uciPacket.header.payloadLength = ParamSize + 1;
    UciCmdPkt.uciPacket.payload[0] = ParamSize;
        
    paramOffset = 1;
        
    for(i = 0; i < ParamSize; i++)
    {
        UciCmdPkt.uciPacket.payload[paramOffset] = pParam->ID;
        UciCmdPkt.uciPacket.payload[paramOffset + 1] = pParam->length;
        paramOffset += 2;
        for(j = 0; j < pParam->length; j++)
        {
            UciCmdPkt.uciPacket.payload[paramOffset + j] = pParam->paramValue[j];
        }
        paramOffset += pParam->length;
        pParam++;
    }

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + paramOffset);
    if(pUciCmdPkt != NULL)
    {     
        memcpy(pUciCmdPkt->bytes, UciCmdPkt.bytes,(PHSCAR4_UCI_HEADER_LENGTH + paramOffset)); 
        pUciCmdPkt->uciPacket.header.payloadLength = paramOffset;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;      
}

/*! ***************************************************************************
 * \brief        Session Initiation.
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application.
                              Session ID = 0x0000000: Reserved for Test Mode Session
 * \param[in]    tp           Type of session 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionCfgInit(uint32_t SessionId, Ranger4Uci_SessionType_t tp)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;
    uint8_t len;

    len = PHSCAR4_UCI_HEADER_LENGTH + sizeof(uint32_t) + sizeof(uint8_t);
    pUciCmdPkt = MEM_BufferAlloc(len); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_SESSION_CFG_OID_INIT;
        pUciCmdPkt->uciPacket.header.payloadLength = len - PHSCAR4_UCI_HEADER_LENGTH;
        
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF) ;
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8) ;
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16) ;        
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24) ;
        pUciCmdPkt->uciPacket.payload[4] = tp;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    
    return en_Status;    
}

/*! ***************************************************************************
 * \brief        Session DeInitiation.
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application.
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionCfgDeInit(uint32_t SessionId)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;
    uint8_t len;

    len = PHSCAR4_UCI_HEADER_LENGTH + sizeof(uint32_t);
    pUciCmdPkt = MEM_BufferAlloc(len); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_SESSION_CFG_OID_DEINIT;
        pUciCmdPkt->uciPacket.header.payloadLength = len - PHSCAR4_UCI_HEADER_LENGTH;
        
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    
    return en_Status;    
}

/*! ***************************************************************************
 * \brief        Set configuration parameters for UWBS Session
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 * \param[in]    CfgCount     Number of session configurations
 * \param[in]    pParam       Point of session configurations(TLV)
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionConfigure(uint32_t SessionId,
                                                                   uint8_t CfgCount, 
                                                                   Ranger4Uci_SessionCfgStructure_t * pParam)
{
    uint8_t i,j, paramOffset;
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_UciFrame_t UciCmdPkt;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;
    
    memset(UciCmdPkt.bytes, 0, PHSCAR4_UCI_HEADER_LENGTH);
    UciCmdPkt.uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
    UciCmdPkt.uciPacket.header.boundaryFlag = 0;
    UciCmdPkt.uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG;
    UciCmdPkt.uciPacket.header.opcodeId = R4_UCI_SESSION_CFG_OID_SET_APP_CFG;
    UciCmdPkt.uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
    UciCmdPkt.uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
    UciCmdPkt.uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
    UciCmdPkt.uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
    UciCmdPkt.uciPacket.payload[4] = CfgCount;
    
    paramOffset = 5;
        
    for(i = 0; i < CfgCount; i++)
    {
        UciCmdPkt.uciPacket.payload[paramOffset] = pParam->ID;
        UciCmdPkt.uciPacket.payload[paramOffset + 1] = pParam->length;
        paramOffset += 2;
        for(j = 0; j < pParam->length; j++)
        {
            UciCmdPkt.uciPacket.payload[paramOffset + j] = pParam->paramValue[j];
        }
        
        if( (paramOffset + pParam->length) > (PHSCA_R4CADSTYPES_UCI_FRAME_SIZE - 4) )
        {
            /* The session configurations is too many to send in one package */
            en_Status = errorCode_InvalidRange;
            return en_Status;
        }
        paramOffset += pParam->length;
        pParam++;
    }

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + paramOffset);
    if(pUciCmdPkt != NULL)
    {     
        memcpy(pUciCmdPkt->bytes, UciCmdPkt.bytes,(PHSCAR4_UCI_HEADER_LENGTH + paramOffset)); 
        pUciCmdPkt->uciPacket.header.payloadLength = paramOffset;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;      
}

/*! ***************************************************************************
 * \brief        Get configuration parameters for UWBS Session
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 * \param[in]    CfgCount     Number of configuration 
 * \param[in]    pCfgID       The point of the configuration IDs 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetConfigure(uint32_t SessionId,
                                                                      uint8_t  CfgCount, 
                                                                      uint8_t * pCfgID)
{
    uint8_t i;
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + CfgCount + 5); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_SESSION_CFG_OID_GET_APP_CFG;
        pUciCmdPkt->uciPacket.header.payloadLength = CfgCount + 1;
        
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
        pUciCmdPkt->uciPacket.payload[4] = CfgCount;

        for(i = 0; i < CfgCount; i++)
        {
            pUciCmdPkt->uciPacket.payload[i + 5] = pCfgID[i];
        }
        
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status; 
}

/*! ***************************************************************************
 * \brief        Get the number of the active UWB session 
 * \param[in]    None
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetNumSession(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_SESSION_CFG_OID_GET_COUNT;
        pUciCmdPkt->uciPacket.header.payloadLength = 0;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}

/*! ***************************************************************************
 * \brief        Get the current state of the UWB session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetSessionState(uint32_t SessionId)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 4 ); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_SESSION_CFG_OID_GET_STATE;
        pUciCmdPkt->uciPacket.header.payloadLength = 4;
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}

/*! ***************************************************************************
 * \brief        Get possible ran multiplier value 
 * \param[in]    None
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetRanMult(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_SESSION_CFG_OID_GET_RAN_MULTIPLIER_VALUE;
        pUciCmdPkt->uciPacket.header.payloadLength = 0;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}


/*! ***************************************************************************
 * \brief        Start a UWB ranging session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionStartRanging(uint32_t SessionId)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 4 ); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_RANGE_CTR_OID_START;
        pUciCmdPkt->uciPacket.header.payloadLength = 4;
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}                                                                  
                                                                   
/*! ***************************************************************************
 * \brief        Stop a UWB ranging session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionStopRanging(uint32_t SessionId)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 4 ); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_RANGE_CTR_OID_STOP;
        pUciCmdPkt->uciPacket.header.payloadLength = 4;
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}

/*! ***************************************************************************
 * \brief        Resume a UWB ranging session 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionResumeRanging(uint32_t SessionId, uint32_t StsIndex)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 8 ); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_RANGE_CTR_OID_RESUME;
        pUciCmdPkt->uciPacket.header.payloadLength = 8;
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
        pUciCmdPkt->uciPacket.payload[4] = (uint8_t) (StsIndex & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[5] = (uint8_t)((StsIndex & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[6] = (uint8_t)((StsIndex & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[7] = (uint8_t)((StsIndex & 0xFF000000) >> 24);
        
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}


/*! ***************************************************************************
 * \brief        Get the number of times ranging 
 * \param[in]    SessionId    Session ID is 4 Octets unique random number generated
                              by application. 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_MacSessionGetRangingCnt(uint32_t SessionId)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 4 ); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_RANGE_CTR_OID_GET_RANGING_COUNT;
        pUciCmdPkt->uciPacket.header.payloadLength = 4;
        pUciCmdPkt->uciPacket.payload[0] = (uint8_t) (SessionId & 0x000000FF);
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)((SessionId & 0x0000FF00) >> 8);
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)((SessionId & 0x00FF0000) >> 16);
        pUciCmdPkt->uciPacket.payload[3] = (uint8_t)((SessionId & 0xFF000000) >> 24);
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}




/*! ***************************************************************************
 * \brief        Query UWB Time Stamp.
 * \param[in]    
 *
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_ProprietaryQueryTimeStamp(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_PROPRIETARY_OID_QUERY_UWB_TIMESTAMP;
        pUciCmdPkt->uciPacket.header.payloadLength = 0u;
        
        //we need to send this command synchronously
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendSync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;   
}





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
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciActivate(phscaR4_UciAppType_t appType, uint8_t comm_interface_type)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 1u ); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN;
        if(appType == UciAppType_UciAppSupport)
        {
            pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_ACTIVATE;
        }
        else
        {
            pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCIMAC_OID_ACTIVATE;
        }
        pUciCmdPkt->uciPacket.header.payloadLength = 0x01u;
        pUciCmdPkt->uciPacket.payload[0] = comm_interface_type;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
        //NCJ29D5 device resets after 100 milli-seconds and activates SWUP, no response 
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status;
}


/*! ***************************************************************************
 * \brief        This command deactivates the SWUP by clearing the activation 
 *               conditions and reboots the device 
 * \param[in]    comm_interface_type  
 *               0x00: Select the default SWUP communication interface software(RCI). 
 *               0x01: Select the customer-specific communicationinterface software(UCI)
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciDeactivate(uint8_t comm_interface_type)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 1u ); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_SET_DEACTIVATE;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x01u;
        pUciCmdPkt->uciPacket.payload[0] = comm_interface_type;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
        //NCJ29D5 device resets after 100 milli-seconds and deactivates SWUP, no response 
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status;
}     

/*! ***************************************************************************
 * \brief        Clears all manifest segments from device
 *               
 * \param[in]    null 
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciClearRAMManifest(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_CLEAR_RAM_MANIFEST;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x00u;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status;    
}     


/*! ***************************************************************************
 * \brief        Transfer manifest
 *               
 * \param[in]    offset: Active section of manifest,0 to 0x03 be used to select 
 *               part of manifest.
 * \param[in]    *data : Point of data part of manifest, 128 bytes
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciTransferManifest(uint8_t offset, uint8_t *data)
{   
    uint32_t i;
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 0x81u); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_TRANSFER_MANIFEST;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x81u;
        
        pUciCmdPkt->uciPacket.payload[0] = offset;
        for(i = 0; i < PHSCAR4_UCISWUP_DATA_SIZE; i++)
        {
            pUciCmdPkt->uciPacket.payload[i+1] = data[i];
        }
        
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status; 
}

/*! ***************************************************************************
 * \brief        Start update 
 *               
 * \param[in]    null
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciStartUpdate(void)
{    
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_START_UPDATE;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x00u;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status;  
} 


/*! ***************************************************************************
 * \brief        Transfer component data to device
 *               
 * \param[in]    ComponentId:Component index as per defined in the manifest's Component Table
 * \param[in]    SegOffset:  Offset from the component's given start address
 * \param[in]    *data :     Point of data part of component, 128 bytes
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciTransferComponent(uint8_t ComponentId, uint16_t SegOffset, uint8_t *data)
{
    uint32_t i;
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 0x83u); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_TRANSFER_COMPONENT;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x83u;
        
        pUciCmdPkt->uciPacket.payload[0] = ComponentId;
        pUciCmdPkt->uciPacket.payload[1] = (uint8_t)SegOffset;
        pUciCmdPkt->uciPacket.payload[2] = (uint8_t)(SegOffset >> 8);
        for(i = 0; i < PHSCAR4_UCISWUP_DATA_SIZE; i++)
        {
            pUciCmdPkt->uciPacket.payload[i+3] = data[i];
        }
        
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status; 
}    
 

/*! ***************************************************************************
 * \brief        Verify the selected component described in the manifest
 *               
 * \param[in]    ComponentId:Component index as per defined in the manifest's Component Table
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciVerifyComponent(uint8_t componentIndex)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + 1u); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_VERIFY_COMPONENT;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x01u;
        pUciCmdPkt->uciPacket.payload[0] = componentIndex;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status; 
}


/*! ***************************************************************************
 * \brief        Verify the all components described in the manifest
 *               
 * \param[in]    null
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciVerifyAll(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_VERIFY_ALL;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x00u;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status; 
}

/*! ***************************************************************************
 * \brief        This command checks whether all components in manifest have been
 *               verified.The command provides a negative response if any component
 *               is not verified successfully. Upon success, following scenario 
 *               happens: 1. Erase the manifest in the flash area
 *                        2. The SWUP activation condition is cleared.
 *                        3. Reset.
 * \param[in]    null
 *****************************************************************************/
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciFinistUpdate(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_FINISH_UPDATE;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x00u;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status;
}  
    
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
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciGetDeviceInfo(void)
{    
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_GET_DEVICE_INFO;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x00u;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status;
} 

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
phscaR4CadsTypesIntf_ErrorCode_t phscaR4_SWUPUciReadDeviceID(void)
{ 
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0u, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_CADS_INTERNAL;
        pUciCmdPkt->uciPacket.header.opcodeId = PHSCA_R4SWUP_UCI_OID_READ_DEVICE_ID;
        pUciCmdPkt->uciPacket.header.payloadLength = 0x00u;
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }    

    return en_Status;
}  

/*! ***************************************************************************
 * \brief        
 * \param[in]    none   
 *****************************************************************************/
//Modify (Ken):NXP-V0001 NO.5 -20240319
phscaR4CadsTypesIntf_ErrorCode_t Ranger4UciCmd_InotLowPower(void)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt = NULL;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;

    pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH); 
    
    if(pUciCmdPkt != NULL)
    {
        memset(pUciCmdPkt, 0, PHSCAR4_UCI_HEADER_LENGTH);
        pUciCmdPkt->uciPacket.header.msgType = PHSCA_R4CADSTYPES_UCI_MT_CMD_MSG;
        pUciCmdPkt->uciPacket.header.boundaryFlag = 0u;
        pUciCmdPkt->uciPacket.header.groupId = PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN;
        pUciCmdPkt->uciPacket.header.opcodeId = R4_UCI_PROPRIETARY_OID_DEVICE_SUSPEND;
        pUciCmdPkt->uciPacket.header.payloadLength = 0u;
        
        
        en_Status = Ranger4_UciExecuteCommand(R4Pkt_SendAsync, pUciCmdPkt);
    }
    else
    {
        en_Status = errorCode_Undefined; 
    }
        
    return en_Status;    
}

#endif  // for UWB_Support

