#include <stdlib.h>

#include "sysinit/sysinit.h"
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_hal.h>
#include "console/console.h"

#include "openthread/ot_common.h"
//#include <openthread/types.h>
#include <openthread/platform/uart.h>
#include <openthread/platform/diag.h>
//#include <openthread/platform/platform.h>
#include <openthread/platform/alarm-milli.h>
//#include <openthread/platform/usec-alarm.h>
#include <openthread/platform/logging.h>
#include <openthread/platform/radio.h>

static otRadioFrame gTransmitFrame;
static otRadioFrame gReceiveFrame;
static otError gTransmitError;
static otError gReceiveError;

static uint8_t gTransmitPsdu[MAX_OT_FRAMELEN];
static uint8_t gReceivePsdu[MAX_OT_FRAMELEN];

static otRadioState gState = OT_RADIO_STATE_DISABLED;
static bool gIsReceiverEnabled = false;
static bool gPromiscuous = false;
static bool gTransmitdone = false;
static bool gReceivedone = false;
static uint8_t gChannel = 0;
static struct os_callout dw1000_callout;
static ot_instance_t *g_ot_inst;

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static void dw1000_sched(struct os_event* ev);

void RadioInit(dw1000_dev_instance_t* inst){
    g_ot_inst = inst->ot;

    inst->ot->cbs = (dw1000_mac_interface_t){
        .id = DW1000_OT,
        .rx_complete_cb = rx_complete_cb,
        .rx_timeout_cb = rx_timeout_cb,
        .tx_complete_cb = tx_complete_cb,
    };

    dw1000_mac_append_interface(inst, &inst->ot->cbs);

    os_callout_init(&dw1000_callout, &(inst->ot->eventq), dw1000_sched ,(void*)inst->ot);
    gTransmitFrame.mLength  = 0;
    gTransmitFrame.mPsdu    = gTransmitPsdu;
    gReceiveFrame.mLength   = 0;
    gReceiveFrame.mPsdu     = gReceivePsdu;

}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    *aIeeeEui64 = g_ot_inst->dev->my_long_address;
    (void)aInstance;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    dw1000_set_eui(g_ot_inst->dev, *((uint64_t *)aAddress->m8));
    (void)aInstance;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)aInstance;
    return gPromiscuous;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)aInstance;

    gPromiscuous = aEnable;
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    g_ot_inst->dev->my_short_address = aAddress;
    (void)aInstance;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)aInstance;

    return (int8_t)dw1000_calc_rssi(g_ot_inst->dev, &g_ot_inst->dev->rxdiag);
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif

	otError error = OT_ERROR_INVALID_STATE;
    (void)aInstance;
    gChannel = aChannel = g_ot_inst->dev->config.channel;
    if (gState != OT_RADIO_STATE_DISABLED)
    {
        gReceiveFrame.mChannel = aChannel;
        error  = OT_ERROR_NONE;
        gState = OT_RADIO_STATE_RECEIVE;

        if (!gIsReceiverEnabled){

            /* TBD: FIFO related changes if required */
            dw1000_set_rx_timeout(g_ot_inst->dev, 0);
            if(!dw1000_start_rx(g_ot_inst->dev).start_rx_error)
            {
                gIsReceiverEnabled = true;
            }
            else
                error  = OT_ERROR_FAILED;
        }
    }

    return error;
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanid){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)aInstance;
    dw1000_dev_instance_t* inst = g_ot_inst->dev;
    dw1000_set_panid(inst, aPanid);
    inst->PANID = aPanid;
}


otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aPacket){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)aInstance;
    
    dw1000_dev_instance_t* inst = g_ot_inst->dev;
    otError error = OT_ERROR_NONE;

    gState = OT_RADIO_STATE_TRANSMIT;
    dw1000_stop_rx(inst);
    memcpy(gTransmitFrame.mPsdu, aPacket->mPsdu, aPacket->mLength);

    dw1000_write_tx_fctrl(inst, aPacket->mLength, 0, true);
    dw1000_write_tx(inst, aPacket->mPsdu, 0, aPacket->mLength);
    dw1000_set_wait4resp(inst, true);
    dw1000_set_rx_timeout(inst, 0);
    if(dw1000_start_tx(inst).start_tx_error)
        error = OT_ERROR_FAILED;

    if (error == OT_ERROR_NONE)
    {
        otPlatRadioTxStarted(aInstance, aPacket);
    }
 
    return error;
}

otError otPlatRadioSleep(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    otError error = OT_ERROR_INVALID_STATE;
    (void)aInstance;

    return error;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;

	printf("# %s #\n",__func__);
#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    return (gState != OT_RADIO_STATE_DISABLED) ? true : false;
}

otError otPlatRadioEnable(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    if (!otPlatRadioIsEnabled(aInstance))
    {
        gState = OT_RADIO_STATE_SLEEP;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance){
 
#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    dw1000_phy_forcetrxoff(g_ot_inst->dev);
    return OT_ERROR_NONE;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance){

    (void)aInstance;

    return &gTransmitFrame;
}

uint8_t otPlatRadioPrintBuf(uint8_t *abuffer){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif

    /*Return the size. (Allowed value is 32 bytes)*/
     return 4;
}

static void dw1000_sched(struct os_event* ev){
    ot_instance_t* ot = (ot_instance_t*)ev->ev_arg;
    otInstance* aInstance = ot->sInstance;
    if(gReceivedone == true){
        gReceivedone = false;
#if MYNEWT_VAL(COMMAND)
        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioReceiveDone(aInstance,
                    &gReceiveFrame,
                    gReceiveError);
        }
        else
#endif
        {
            otPlatRadioReceiveDone(aInstance,
                    &gReceiveFrame,
                    gReceiveError);
        }
    }
    if(gTransmitdone == true){
        gTransmitdone = false;
#if MYNEWT_VAL(COMMAND)
        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance,
                    &gTransmitFrame,
                    gTransmitError);
        }
        else
#endif
        {
            otPlatRadioTxDone(aInstance,
                    &gTransmitFrame,
                    NULL,
                    gTransmitError);
        }
    }
}

static bool 
rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

	gReceiveFrame.mLength = inst->frame_len;
    gReceiveFrame.mChannel = inst->config.channel;
//    gReceiveFrame.mInfo.mRxInfo.mRssi = otPlatRadioGetRssi(g_ot_inst->sInstance); //RSSI should be zero
    gReceiveFrame.mInfo.mRxInfo.mRssi = -50;
    gReceiveFrame.mInfo.mRxInfo.mLqi = 0;
    gReceiveError = OT_ERROR_NONE;
    memcpy(gReceiveFrame.mPsdu, inst->rxbuf, gReceiveFrame.mLength);
    gReceivedone = true;
    os_eventq_put(&inst->ot->eventq, &dw1000_callout.c_ev);
	return true;
}

static bool 
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
	return true;
}

static bool 
tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    gTransmitdone = true;
    gTransmitError = OT_ERROR_NONE;
    os_eventq_put(&inst->ot->eventq, &dw1000_callout.c_ev);
    return true;
}

otError otPlatRadioEnergyScan(otInstance *aInstance,
                              uint8_t aScanChannel,
                              uint16_t aScanDuration){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)aInstance;
    (void)aScanChannel;
    (void)aScanDuration;

    return OT_ERROR_NOT_IMPLEMENTED;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)aInstance;

    return OT_RADIO_CAPS_NONE;
}

void dw1000_auto_pending_bit_set(bool enabled){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
	(void)enabled;
}

bool dw1000_pending_bit_for_addr_set(const uint8_t *p_addr, bool extended){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
	(void)p_addr;
	(void)extended;

     return 1;
}

void dw1000_pending_bit_for_addr_reset(bool extended){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void)extended;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance){

    (void) aInstance;

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    dw1000_pending_bit_for_addr_reset(true);
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void) aInstance;

    dw1000_pending_bit_for_addr_reset(false);
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance,
                                             const uint16_t aShortAddress){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
	(void) aInstance;
    otError error = OT_ERROR_NONE;
    return error;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *ExtAddress){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void) aInstance;
    otError error = OT_ERROR_NONE;
    return error;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable){

    (void) aInstance;

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    dw1000_auto_pending_bit_set(aEnable);
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance,
                                               const uint16_t aShortAddress){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    (void) aInstance;
    otError error = OT_ERROR_NONE;
    return error;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance,
                                           const otExtAddress * aExtAddress){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
	(void) aInstance;
    otError error = OT_ERROR_NONE;
    return error;
}

/*This function used to Receive the Receiver sensitivity */
int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
	printf("# %s #\n",__func__);
#endif
    // TBD for Getting receiver sensitivity
    (void)aInstance;
    return -100;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    otError error = OT_ERROR_NONE;
    (void)aInstance;

    if (aPower == NULL)
    {
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
//        *aPower = sDefaultTxPower;
    }

    return error;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    (void)aInstance;

 //   sDefaultTxPower = aPower;
//    nrf_802154_tx_power_set(aPower);

    return OT_ERROR_NONE;
}
