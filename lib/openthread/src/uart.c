#include <stdlib.h>

#include "sysinit/sysinit.h"

#include "openthread/ot_common.h"
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_hal.h>
#include "console/console.h"
#include <openthread/platform/logging.h>
#include <openthread/platform/uart.h>

void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion,
               const char *aFormat, ...){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    char logString[LOG_PARSE_BUFFER_SIZE + 1];
    uint16_t length = 0;

    va_list paramList;
    va_start(paramList, aFormat);
    length += vsnprintf(&logString[length], (LOG_PARSE_BUFFER_SIZE - length),
                        aFormat, paramList);
    va_end(paramList);
    printf("%s \r\n", logString);
    return;
}


otError otPlatUartEnable(void){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    return OT_ERROR_NONE;
}



otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    console_write((const char *)aBuf,(int)aBufLength);
    otError error = OT_ERROR_NONE;
    otPlatUartSendDone();
    return error;
}
