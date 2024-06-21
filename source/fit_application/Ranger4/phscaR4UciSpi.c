//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))

/* =============================================================================
 * Includes
 * ========================================================================== */
#include "fsl_common.h"
#include "fsl_dspi.h"
#include "fsl_os_abstraction.h"
#include "Panic.h"

/* Ranger 4 include */     
#include "phscaR4CadsTypesIntf.h"    
#include "phscaR4DriverHw.h"
//#include "phscaR4_RCICmd.h"
#include "phscaR4UciSpi.h"

/* =============================================================================
 * Private Symbol Defines
 * ========================================================================== */

/* =============================================================================
 * Private Function-like Macros
 * ========================================================================== */

/* =============================================================================
 * Private Type Definitions
 * ========================================================================== */
//Modify (Ken):NXP-V0001 NO.5 -20240319
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_VENUS_EVT1_H
#define DSPI_MASTER_BASEADDR SPI1
#else
#define DSPI_MASTER_BASEADDR SPI0
#endif
#define DSPI_MASTER_CLK_SRC DSPI0_CLK_SRC
#define DSPI_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)
#define DSPI_MASTER_PCS_FOR_INIT kDSPI_Pcs2
/* =============================================================================
 * Private Function Prototypes
 * ========================================================================== */
/* @brief Actual SPI transmission between host and Ranger4 */
static void phscaR4UciSpi_TransceiveSPI(uint32_t DataLength, uint8_t dataToTransmit[], uint8_t dataReceived[]);
/* =============================================================================
 * Private Module-wide Visible Variables
 * ========================================================================== */
static phscaR4CadsTypesIntf_UciFrame_t  m_UCICommandSPIBuffer = {0};
static phscaR4CadsTypesIntf_UciFrame_t  m_UCIResponseSPIBuffer = {0};
/* =============================================================================
 * Private Function Definitions
 * ========================================================================== */
/* @brief Actual SPI transmission between host and Ranger4 */
static void phscaR4UciSpi_TransceiveSPI(uint32_t DataLength, uint8_t dataToTransmit[], uint8_t dataReceived[])
{
    dspi_transfer_t masterXfer;

    /* Start master transfer, send data to slave */
    masterXfer.txData = dataToTransmit;
    masterXfer.rxData = dataReceived;
    masterXfer.dataSize = DataLength;
    masterXfer.configFlags = kDSPI_MasterCtar0;

    DSPI_MasterTransferBlocking(DSPI_MASTER_BASEADDR, &masterXfer);
}

phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_ReceiveData(phscaR4CadsTypesIntf_UciFrame_t * rxframe)
{
    assert(NULL != rxframe);
    
    memset(m_UCICommandSPIBuffer.bytes,0, sizeof(m_UCICommandSPIBuffer));
    
    if(phscaR4Driver_SixWireStartReceive() == slaveTransferStatus_HasData)
    {
        /* Receive first 1 byte Ignor it*/
        phscaR4UciSpi_TransceiveSPI(1, (uint8_t *)m_UCICommandSPIBuffer.bytes[0u], (uint8_t *)m_UCIResponseSPIBuffer.bytes);
        
        /* Receive 4 header bytes*/
        phscaR4UciSpi_TransceiveSPI(4,(uint8_t *)m_UCICommandSPIBuffer.bytes[0u], (uint8_t *)m_UCIResponseSPIBuffer.bytes);
        
        if(m_UCIResponseSPIBuffer.uciPacket.header.payloadLength != 0)
        {
             /* Receive remain bytes*/
             phscaR4UciSpi_TransceiveSPI(m_UCIResponseSPIBuffer.uciPacket.header.payloadLength,
                                         (uint8_t *)m_UCICommandSPIBuffer.bytes, 
                                         (uint8_t *)&m_UCIResponseSPIBuffer.bytes[PHSCAR4_UCI_RES_HEADER_LENGTH]);
        }

        if(phscaR4Driver_SixWireStopReceive())
        {
            memcpy(rxframe->bytes, m_UCIResponseSPIBuffer.bytes, (m_UCIResponseSPIBuffer.uciPacket.header.payloadLength + 4));
            return errorCode_Success;
        }
        else
        {
            return errorCode_UciGenericError;
        }
    }
    else
    {
        phscaR4Driver_SixWireStopReceive();
    }
    
    return errorCode_UciNoEvent;
}

phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_SendData(phscaR4CadsTypesIntf_UciFrame_t * frame)
{
    assert(NULL != frame);

    /* Initialize values */
    phscaR4CadsTypesIntf_ErrorCode_t statusError = errorCode_Success;

    uint16_t packetLength = 0u;

    packetLength = (uint16_t) frame->uciPacket.header.payloadLength + 4u;
    memcpy(m_UCICommandSPIBuffer.bytes,frame->bytes, packetLength);
    /* Check start transfer */
    if (slaveTransferStatus_Ready == phscaR4Driver_SixWireStartTransmit())
    {
        /* Start transfer */
        phscaR4UciSpi_TransceiveSPI(packetLength,
                                    (uint8_t *)m_UCICommandSPIBuffer.bytes, 
                                    (uint8_t *)m_UCIResponseSPIBuffer.bytes);
    }
    else
    {
        statusError = errorCode_UciBusyError;
    }

    /* Stop transfer */
    (void) phscaR4Driver_SixWireStopTransmit();

    return statusError;
}


//******************************************************************************
// Transmit and Receive
//******************************************************************************
phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_TransferData(phscaR4CadsTypesIntf_UciFrame_t * txframe, 
                                                            phscaR4CadsTypesIntf_UciFrame_t * rxframe)
{
    assert(NULL != txframe);
    assert(NULL != rxframe);
    
    phscaR4CadsTypesIntf_ErrorCode_t status = errorCode_Success;
    phscaR4Driver_SlaveTransferStatus_t slaveTransferStatus = slaveTransferStatus_Undefined;
    uint16_t packetLength = 0u;
    
    packetLength = (uint16_t)txframe->uciPacket.header.payloadLength + 4;

    memcpy(m_UCICommandSPIBuffer.bytes, txframe->bytes, packetLength);
    
    //==========================================================================
    /* Get slave transfer status */   
    //==========================================================================
    slaveTransferStatus =  phscaR4Driver_SixWireStartTransmit();   
    //==========================================================================
    // [Check start transfer] - Slave is ready to receive data
    //==========================================================================
    if (slaveTransferStatus_Ready == slaveTransferStatus)
    {
        phscaR4UciSpi_TransceiveSPI(packetLength,
                                    (uint8_t *)m_UCICommandSPIBuffer.bytes, 
                                    (uint8_t *)m_UCIResponseSPIBuffer.bytes);
        if(phscaR4Driver_SixWireStopTransmit())
        {
            status = errorCode_UciSendSuccess;
        }
        else
        {
            status = errorCode_UciGenericError;
        }
    }
    //==========================================================================
    // [Check start transfer] - Slave has data need to send
    //==========================================================================
    else if(slaveTransferStatus_HasData == slaveTransferStatus)
    {
        memset(m_UCICommandSPIBuffer.bytes,0, sizeof(m_UCICommandSPIBuffer));
        /* Receive first 1 byte Ignor it*/
        phscaR4UciSpi_TransceiveSPI(1, (uint8_t *)m_UCICommandSPIBuffer.bytes[0u], (uint8_t *)m_UCIResponseSPIBuffer.bytes);
        
        /* Receive 4 header bytes*/
        phscaR4UciSpi_TransceiveSPI(4,(uint8_t *)m_UCICommandSPIBuffer.bytes[0u], (uint8_t *)m_UCIResponseSPIBuffer.bytes);
        
        if(m_UCIResponseSPIBuffer.uciPacket.header.payloadLength != 0)
        {
             /* Receive remain bytes*/
             phscaR4UciSpi_TransceiveSPI(m_UCIResponseSPIBuffer.uciPacket.header.payloadLength,
                                         (uint8_t *)m_UCICommandSPIBuffer.bytes, 
                                         (uint8_t *)&m_UCIResponseSPIBuffer.bytes[PHSCAR4_UCI_RES_HEADER_LENGTH]);
        }

        if(phscaR4Driver_SixWireStopReceive())
        {
            memcpy(rxframe->bytes, m_UCIResponseSPIBuffer.bytes, (m_UCIResponseSPIBuffer.uciPacket.header.payloadLength + 4));
            status = errorCode_UciRecvSuccess;
        }
        else
        {
            status = errorCode_UciGenericError;
        }
    }
    //==========================================================================
    // [Check start transfer] - Undefined status
    //==========================================================================
    else
    {
        status = errorCode_UciR4NotReadyError;
        (void) phscaR4Driver_SixWireStopTransmit();
    }
           
    return status;        
}

                                   
/* =============================================================================
 * Public Function Definitions
 * ========================================================================== */
phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_InterfaceInit(void)
{
    /* Initialize values */
    phscaR4CadsTypesIntf_ErrorCode_t statusError = errorCode_Success;
    
    phscaR4Driver_InitSixWireInterface();
       
    /* SPI Configuration:
    *
    * SPI0 Channel
    * KW38 = Master mode only
    * baudRate = 8 MHz
    * bitsPerFrame= 8 Bits per Frame
    * Cpol = 1
    * Cpha = 0
    * direction = Msb First
    * pcsToSckDelayInNanoSec = Chip Select to Clock delay time in nanoseconds
    * lastSckToPcsDelayInNanoSec = The last Clock to Chip Select delay time in nanoseconds
    * betweenTransferDelayInNanoSec = After the SCK delay time in nanoseconds
    * Chip Select = kDSPI_Pcs0 (0)
    * Chip Select active Low
    * enableContinuousSCK = Continuous Clock for SPI disabled
    * enableRxFifoOverWrite = false
    * enableModifiedTimingFormat = modified timing format for SPI disabled
    * SPIO Clock is used
    */
    uint32_t srcClock_Hz;
    dspi_master_config_t masterConfig;

    /* Master config */
    masterConfig.whichCtar = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate = 8000000;
    masterConfig.ctarConfig.bitsPerFrame = 8U;                       // transfer 8 bits per frame (byte)
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveLow;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1U;             // delay in ns between first clock and chip select // -> Delay for the slave to answer
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 0U;         // delay in ns to wait between last clock of the frame and first clock of the next frame
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 0U;      // time between transfers

    masterConfig.whichPcs = DSPI_MASTER_PCS_FOR_INIT;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK = false;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    srcClock_Hz = DSPI_MASTER_CLK_FREQ;
    DSPI_MasterInit(DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);

    return statusError;
}

phscaR4CadsTypesIntf_ErrorCode_t phscaR4UciSpi_InterfaceDeInit(void)
{
    /* De-initialize SPI peripheral */
    DSPI_Deinit(DSPI_MASTER_BASEADDR);
    /* De-initialize UCI port hardware */
    phscaR4Driver_DeInitSixWireInterface();
    
    return errorCode_Success;
}



void phscaR4UciSpi_HardwareResetRanger4(void)
{
    volatile uint32_t counter = 0u;
    
    /* Before hareware reset device, we should close INT interrupt since the INT may drive
       to low during set reset low*/
    phscaR4Driver_DisableINTInterrupt();
    
    phscaR4Driver_ResetOn();
    //OSA_TimeDelay(2);
    // faked delay: use this function for fixed delays -> CLOCK_CONFIG_FllStableDelay();
    for(counter = 0; counter < 10000; counter++)
    {
        ;
    }

    phscaR4Driver_ResetOff();
    
    //Enable INT interrupt
    phscaR4Driver_EnableINTInterrupt();
//     /* delay an amount of time before state of IRQn line is stable */
//    for(counter = 0; counter < 40000; counter++)
//    {
//        ;
//    }
//    OSA_TimeDelay(10);
    //phscaRNG4_v_getRCIResponse()
}


#endif  // for UWB_Support







