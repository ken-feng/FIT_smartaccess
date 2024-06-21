/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "board.h"

#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "SEGGER_RTT.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_CAN CAN0
#define EXAMPLE_CAN_CLKSRC kCLOCK_BusClk
#define EXAMPLE_CAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define USE_CANFD (0)
#define USE_IMPROVED_TIMING_CONFIG (1)

#define APP_DBG_LOG(...) SEGGER_RTT_printf(0,__VA_ARGS__)
/*
 *    DWORD_IN_MB    DLC    BYTES_IN_MB             Maximum MBs
 *    2              8      kFLEXCAN_8BperMB        32
 *    4              10     kFLEXCAN_16BperMB       21
 *    8              13     kFLEXCAN_32BperMB       12
 *    16             15     kFLEXCAN_64BperMB       7
 *
 * Dword in each message buffer, Length of data in bytes, Payload size must align,
 * and the Message Buffers are limited corresponding to each payload configuration:
 */
#define DWORD_IN_MB (4)
#define DLC (8)
#define BYTES_IN_MB kFLEXCAN_16BperMB

/* To get most precise baud rate under some circumstances, users need to set
   quantum which is composed of PSEG1/PSEG2/PROPSEG. Because CAN clock prescaler
   = source clock/(baud rate * quantum), for e.g. 84M clock and 1M baud rate, the
   quantum should be .e.g 14=(6+3+1)+4, so prescaler is 6. By default, quantum
   is set to 10=(3+2+1)+4, because for most platforms e.g. 120M source clock/(1M
   baud rate * 10) is an integer. Remember users must ensure the calculated
   prescaler an integer thus to get precise baud rate. */
#define SET_CAN_QUANTUM 0
#define PSEG1 3
#define PSEG2 2
#define PROPSEG 1
#define FPSEG1 3
#define FPSEG2 3
#define FPROPSEG 1
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829)
/* To consider the First valid MB must be used as Reserved TX MB for ERR005829
   If RX FIFO enable(RFEN bit in MCE set as 1) and RFFN in CTRL2 is set default zero, the first valid TX MB Number is 8
   If RX FIFO enable(RFEN bit in MCE set as 1) and RFFN in CTRL2 is set by other value(0x1~0xF), User should consider
   detail first valid MB number
   If RX FIFO disable(RFEN bit in MCE set as 0) , the first valid MB number is zero */
#ifdef RX_MESSAGE_BUFFER_NUM
#undef RX_MESSAGE_BUFFER_NUM
#define RX_MESSAGE_BUFFER_NUM (10)
#endif
#ifdef TX_MESSAGE_BUFFER_NUM
#undef TX_MESSAGE_BUFFER_NUM
#define TX_MESSAGE_BUFFER_NUM (9)
#endif
#endif
#ifndef DEMO_FORCE_CAN_SRC_OSC
#define DEMO_FORCE_CAN_SRC_OSC 0
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void can_rx(void);
void can_powOn(void);
void CAN_uwb_ranging_rpt(void);
void CAN_cmd_compose_send(uint16_t cmd);
void CAN_set_ranging(uint16_t dist1, uint16_t dist2, uint16_t dist3);
void CAN_set_swVer(uint8_t mainVer, uint8_t minVer);


/*******************************************************************************
 * Variables
 ******************************************************************************/
flexcan_handle_t flexcanHandle;
volatile bool txComplete = false;
volatile bool rxComplete = false;
volatile bool wakenUp    = false;
static flexcan_mb_transfer_t txXfer, rxXfer;
#if (defined(USE_CANFD) && USE_CANFD)
flexcan_fd_frame_t frame;
flexcan_fd_frame_t txframe;
#else
flexcan_frame_t frame;
flexcan_frame_t txframe;
#endif
uint32_t txIdentifier;
uint32_t rxIdentifier;
uint16_t prev_id;
uint8_t VerMajor = 0x01;
uint8_t VerMinor = 0x01;
uint8_t can_paired = 1;
uint8_t can_uwb    =1;
uint8_t can_rfic   =1;
uint8_t can_ack    =0;
uint16_t can_error = 0x1234;
int can_uwb_rep = 1;
int can_in_rge = 1;
uint16_t can_dist1 = 0x1234;
uint16_t can_dist2 = 0x5678;
uint16_t can_dist3 = 0x9abc;
uint8_t  can_rke_cmd = 0x03;
uint8_t can_ranging=1;
enum {
	CAN_ACK_NAK   = 0x500,
	CAN_PowON_rpt = 0x501,
	CAN_Err_Msg   = 0x502,
	CAN_Rang_rpt  = 0x503,
	CAN_Host_Ctrl = 0x504,
	CAN_SA_Diag   = 0x505,
	CAN_RKE       = 0x506,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief FlexCAN Call Back function
 */
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)
    {
        case kStatus_FLEXCAN_RxIdle:
            if (RX_MESSAGE_BUFFER_NUM == result)
            {
            	APP_DBG_LOG("--Rx MB ID: 0x%3x, MB data: 0x%x %x %x %x %x %x %x %x, Time stamp: %d\r\n", frame.id >> CAN_ID_STD_SHIFT,
            	                   frame.dataByte0,
            					   frame.dataByte1,
            					   frame.dataByte2,
            					   frame.dataByte3,
            					   frame.dataByte4,
            					   frame.dataByte5,
            					   frame.dataByte6,
            					   frame.dataByte7,
            					   frame.timestamp);
                rxComplete = true;

                switch(frame.id >> CAN_ID_STD_SHIFT)
                {
                    case CAN_Host_Ctrl:
                	    if(frame.dataByte0==0x01)
                	    {
                	        CAN_cmd_compose_send(CAN_SA_Diag);
                	    }
                	    else if(frame.dataByte0==0x02)
                     	{
                            can_ranging = 0;
                     	}
                	    break;
                    case CAN_RKE:
                	    if(frame.dataByte0==0x01)
                	        APP_DBG_LOG("CAN: arm\r\n");
                	    else if(frame.dataByte0==0x02)
                	        APP_DBG_LOG("CAN: DIS arm\r\n");
                	    else if(frame.dataByte0==0x02)
                	        APP_DBG_LOG("CAN: FINDME\r\n");
                	    break;
                    default:
                    	can_ack =0;
                    	break;
                }

                if(prev_id != CAN_ACK_NAK &&
                   prev_id != CAN_PowON_rpt &&
				   prev_id != CAN_SA_Diag &&
				   prev_id != CAN_Rang_rpt &&
				   prev_id != CAN_Err_Msg ){

                	APP_DBG_LOG("ID:  not match\r\n");
                	can_ack =1;
                    CAN_cmd_compose_send(CAN_ACK_NAK);
                }
                if (frame.id >> CAN_ID_STD_SHIFT > CAN_RKE || frame.id >> CAN_ID_STD_SHIFT < CAN_ACK_NAK)
                {
                	can_ack =0;
                	CAN_cmd_compose_send(CAN_ACK_NAK);
                }

            }
            can_rx();
            break;

        case kStatus_FLEXCAN_TxIdle:
            if (TX_MESSAGE_BUFFER_NUM == result)
            {
                txComplete = true;
            }
            break;

        case kStatus_FLEXCAN_WakeUp:
            wakenUp = true;
            break;
        case kStatus_FLEXCAN_ErrorStatus:
		    APP_DBG_LOG("check CAN connection\r\n");
		    break;
        default:
            break;
    }
}

/*!
 * @brief Main function
 */
int can_init(void)
{
    flexcan_config_t flexcanConfig;
    flexcan_rx_mb_config_t mbConfig;

    /* Initialize board hardware. */
    BOARD_InitPins();
    //BOARD_BootClockRUN();

    FLEXCAN_GetDefaultConfig(&flexcanConfig);
    flexcanConfig.baudRate = 500000U;
    flexcanConfig.baudRateFD = 500000U;

/* Init FlexCAN module. */
#if (!defined(DEMO_FORCE_CAN_SRC_OSC)) || !DEMO_FORCE_CAN_SRC_OSC
#if (!defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)) || !FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE
    flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
#else
#if defined(CAN_CTRL1_CLKSRC_MASK)
    if (!FSL_FEATURE_FLEXCAN_INSTANCE_SUPPORT_ENGINE_CLK_SEL_REMOVEn(EXAMPLE_CAN))
    {
        flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
    }
#endif
#endif /* FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE */
#endif /* DEMO_FORCE_CAN_SRC_OSC */
/* If special quantum setting is needed, set the timing parameters. */
#if (defined(SET_CAN_QUANTUM) && SET_CAN_QUANTUM)
    flexcanConfig.timingConfig.phaseSeg1 = PSEG1;
    flexcanConfig.timingConfig.phaseSeg2 = PSEG2;
    flexcanConfig.timingConfig.propSeg   = PROPSEG;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    flexcanConfig.timingConfig.fphaseSeg1 = FPSEG1;
    flexcanConfig.timingConfig.fphaseSeg2 = FPSEG2;
    flexcanConfig.timingConfig.fpropSeg   = FPROPSEG;
#endif
#endif

#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));
#if (defined(USE_CANFD) && USE_CANFD)
    if (FLEXCAN_FDCalculateImprovedTimingValues(flexcanConfig.baudRate, flexcanConfig.baudRateFD, EXAMPLE_CAN_CLK_FREQ,
                                                &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
    	APP_DBG_LOG("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#else
    if (FLEXCAN_CalculateImprovedTimingValues(flexcanConfig.baudRate, EXAMPLE_CAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
    	APP_DBG_LOG("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#endif
#endif

#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_FDInit(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, BYTES_IN_MB, true);
#else
    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);
#endif

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Set Rx Masking mechanism. */
    FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(0, 0, 0));

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0);
#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_SetFDRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
#else
    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
#endif

/* Setup Tx Message Buffer. */
#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
#else
    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
#endif

    can_rx();
    can_powOn();
   // CAN_uwb_ranging_rpt();


    return 0;
}




void can_powOn(void){

	for(int i=0;i<5;i++){
	    CAN_cmd_compose_send(CAN_PowON_rpt);
	    vTaskDelay(200);
	}
}
static void can_rx(void){

	    frame.format = kFLEXCAN_FrameFormatStandard;
    	frame.type   = kFLEXCAN_FrameTypeData;
        rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
        rxXfer.frame = &frame;
        FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
}

void CAN_uwb_ranging_rpt(void){

//	while(can_ranging){
		CAN_cmd_compose_send(CAN_Rang_rpt);
		vTaskDelay(200);
//	}
}

void CAN_set_ranging_report(int uwb_rep,int in_rge,uint16_t dist1, uint16_t dist2, uint16_t dist3){
	can_uwb_rep = uwb_rep;
	can_in_rge = in_rge;
	can_dist1 = dist1;
	can_dist2 = dist2;
	can_dist3 = dist3;


	CAN_uwb_ranging_rpt();
}

void CAN_set_ranging(uint16_t dist1, uint16_t dist2, uint16_t dist3){
	can_dist1 = dist1;
	can_dist2 = dist2;
	can_dist3 = dist3;
}

void CAN_set_swVer(uint8_t mainVer, uint8_t minVer){
	VerMajor = mainVer;
	VerMinor = minVer;
}

void CAN_cmd_compose_send(uint16_t cmd){

	txframe.id = FLEXCAN_ID_STD(cmd);
	txframe.format = kFLEXCAN_FrameFormatStandard;
	txframe.type   = kFLEXCAN_FrameTypeData;
	txframe.length = DLC;


	switch(cmd){
	case CAN_ACK_NAK:

		if(can_ack)
		    txframe.dataByte0 = 0x00;
		else
			txframe.dataByte0 = 0x01;
		break;
	case CAN_PowON_rpt:
		txframe.dataByte0 = VerMajor;
		txframe.dataByte1 = VerMinor;
		txframe.dataByte2 = can_paired<<2 | can_uwb<<1 | can_rfic;
		break;
	case CAN_Err_Msg:
		txframe.dataByte0 = can_paired<<2 | can_uwb<<1 | can_rfic;
		txframe.dataByte1 = (can_error & 0xff00) >> 8;
		txframe.dataByte2 = (can_error & 0x00ff);
		break;
	case CAN_Rang_rpt:
		txframe.dataByte0 = can_uwb_rep <<1 | can_in_rge ;
		txframe.dataByte1 = 0;
		txframe.dataByte2 = (can_dist1 & 0xff00) >> 8;
		txframe.dataByte3 = (can_dist1 & 0x00ff);
		txframe.dataByte4 = (can_dist2 & 0xff00) >> 8;
		txframe.dataByte5 = (can_dist2 & 0x00ff);
		txframe.dataByte6 = (can_dist3 & 0xff00) >> 8;
		txframe.dataByte7 = (can_dist3 & 0x00ff);
		break;

	case CAN_SA_Diag:
		txframe.dataByte0 = VerMajor;
	    txframe.dataByte1 = VerMinor;
	    txframe.dataByte2 = can_paired<<2 | can_uwb<<1 | can_rfic;
		break;

	default:
		txframe.id = FLEXCAN_ID_STD(CAN_ACK_NAK);
		txframe.dataByte0 = 0x01;
		break;
	}
    txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;

    txXfer.frame = &txframe;
    FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

 //   while (!txComplete || !rxComplete)
    {
    };
    txComplete = false;
    can_rx();
    prev_id = cmd;

}
