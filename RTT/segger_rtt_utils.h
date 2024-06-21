#ifndef __INIT_RTT_H__
#define __INIT_RTT_H__

#include "segger_rtt.h"

#ifndef SEGGER_RTT_ENABLE
#define SEGGER_RTT_ENABLE			0
#endif

#if SEGGER_RTT_ENABLE
#define init_segger_rtt()		SEGGER_RTT_Init();log_debug("segger rtt start.\r\n")

#define log_debug(format, ...)	SEGGER_RTT_printf(0, format, ##__VA_ARGS__)

#define log_debug_info(format, ...) SEGGER_RTT_printf(0, "%s--%d:"format, __FILE__, __LINE__, ##__VA_ARGS__)

#define log_debug_array(array, len)             do{																	\
													int __i;														\
													int line = 0;													\
													log_debug("0000: ");											\
													for(__i=0;__i<(len);++__i){										\
														log_debug("%02X ",((uint8_t *)(array))[__i]);				\
														line ++;													\
														if(__i != 0 && (line % 16 == 0) && (line != len)){			\
															log_debug("\r\n%04X: ", __i);							\
														}															\
													}																\
												}while(0)

 #define log_debug_hexBigend(array, len)         do{                                                              \
                                                       int __j;                                                   \
                                                       for(__j = 0; __j < len; __j++)                             \
                                                       {                                                          \
                                                           log_debug("%02X",((uint8_t *)(array))[len - __j - 1]); \
                                                       }                                                          \
                                                   }while(0)                                                      \
                                                       
#define log_debug_array_ex(note, array, len)    do{log_debug("%s [%dbytes]\r\n",note, len); log_debug_array(array,len); log_debug("\r\n");}while(0)

#else

#define init_segger_rtt()
#define log_debug(format, ...)
#define log_debug_info(format, ...)
#define log_debug_array(array, len)
#define log_debug_array_ex(note, array, len)
#define log_debug_hexBigend(array, len)
#endif

#endif
