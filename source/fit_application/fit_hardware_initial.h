//***************************************************** 
// The KW38 serial head file define 
// Define the main program Ram section  
//***************************************************** 
//#include "fsl_common.h"
//#include "fsl_port.h"
//#include "fsl_gpio.h"
//#include "pin_mux.h"
//****************************************************
//Modify (Ken):NXP-V0001 NO.2 -20240326
#if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __NXP_EVB_TEST_PIN_H || defined __HW_VENUS_EVT1_H



void Init_APP_BOARD(void);
void Init_APP_BOARD_GPIO(void);
//Modify (Ken):NXP-V0001 NO.5 -20240319
#ifdef __FIT_SECURITY_CHIP_H
void Init_APP_SECURITY_CHIP(void);
#endif
#endif




