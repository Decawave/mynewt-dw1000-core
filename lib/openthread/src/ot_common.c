#include <stdlib.h>

#include "sysinit/sysinit.h"
#include <uwb/uwb.h>
#include <uwb/uwb_mac.h>
#include "console/console.h"

#include <openthread/platform/logging.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/uart.h>
#include <openthread/platform/diag.h>
#include <openthread/ot_common.h>
#include <openthread/tasklet.h>
#include "flash_map/flash_map.h"
#include "hal/hal_flash.h"
#include "hal/hal_system.h"

#define OT_FLASH_AREA_ID MYNEWT_VAL(OT_FLASH_AREA)

void __cxa_pure_virtual() { while (1); }

static struct dpl_callout otc_task_callout;
static bool taskletprocess = false;

ot_instance_t * ot_global_inst;

void PlatformInit(struct uwb_dev* inst){
    inst->config.rx.phrMode = DWT_PHRMODE_EXT;
    sysinit();
    return ;
}

otError otPlatRandomGetTrue(uint8_t *aOutput, uint16_t aOutputLength){

#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s #\n",__func__);
#endif
    otError error = OT_ERROR_NONE;

    return error;
}

uint32_t otPlatRandomGet(void){
    srand(os_cputime_get32()); 
    uint32_t rng  = (uint32_t)rand();
    return (uint32_t)rng;
}

uint32_t utilsFlashGetSize(void){

#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s #\n",__func__);
#endif
    const struct flash_area *fa;
    int area_id = OT_FLASH_AREA_ID;
    int rc = 0;
    rc = flash_area_open(area_id, &fa);
    if (rc != 0) {
        return -1;
    }
    uint32_t fa_size = fa->fa_size;
    flash_area_close(fa);
    return fa_size;
}

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize){

#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s : 0x%lX #\n",__func__, aAddress);
#endif
    const struct flash_area *fa;
    int area_id = OT_FLASH_AREA_ID;
    int rc ;
    rc = flash_area_open(area_id, &fa);
    if (rc != 0) {
        return 0;
    }
    rc = flash_area_write(fa, aAddress, aData, aSize);
    if (rc < 0) {
        return 0;
    }
    rc = aSize;
    flash_area_close(fa);
    return rc;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize){

#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s : 0x%lX #\n",__func__, aAddress);
#endif
    const struct flash_area *fa;
    int area_id = OT_FLASH_AREA_ID;
    int rc ;
    rc = flash_area_open(area_id, &fa);
    if (rc != 0) {
        return 0;
    }
    rc = flash_area_read(fa, aAddress, aData, aSize); 
    if (rc < 0) {
        return 0;
    }
    rc = aSize;
    flash_area_close(fa);
    return rc;
}

otError utilsFlashInit(void){
 
#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s #\n",__func__);
#endif
    return OT_ERROR_NONE;
}

otError utilsFlashStatusWait(uint32_t aTimeout){

    otError error = OT_ERROR_NONE;

#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s #\n",__func__);
#endif
    return error;
}

otError utilsFlashErasePage(uint32_t aAddress){

#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s #\n",__func__);
#endif
    const struct flash_area *fa;
    int area_id = OT_FLASH_AREA_ID;
    int rc = -1;
    rc = flash_area_open(area_id, &fa);
    if (rc) {
        return -1;
    }
    rc = flash_area_erase(fa, aAddress, fa->fa_size - aAddress);
    if (rc != 0) {
        return -1;
    }
    flash_area_close(fa);
    return rc;
}

void otPlatReset(otInstance *aInstance){

#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s #\n",__func__);
#endif
    (void)aInstance;
    hal_system_reset();
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower){

    // TBD for Setting Default transmit power
#if MYNEWT_VAL(OT_DEBUG)
    printf("# %s #\n",__func__);
#endif
    (void)aInstance;
    (void)aPower;
}

static void tasklet_sched(struct dpl_event* ev)
{
    ot_instance_t* ot = (ot_instance_t*)dpl_event_get_arg(ev);
    otInstance* aInstance = ot->sInstance;
    if(taskletprocess == true){
        taskletprocess = false;

        if(otTaskletsArePending(aInstance))
            otTaskletsProcess(aInstance);
        else
        {
            taskletprocess = true;
            dpl_callout_reset(&otc_task_callout,DPL_TICKS_PER_SEC/32);
        }
    }
}

void otTaskletsSignalPending(otInstance *aInstance)
{
    taskletprocess = true;
    if(ot_global_inst->status.initialized == 1)
        dpl_callout_reset(&otc_task_callout, 0);
    else{
        if(otTaskletsArePending(aInstance))
            otTaskletsProcess(aInstance);
        else
        {
            ot_global_inst->sInstance = aInstance;
            dpl_callout_reset(&otc_task_callout,DPL_TICKS_PER_SEC/32);
        }
    }
}

static void*
ot_task(void *arg)
{
    ot_instance_t *ot = (ot_instance_t*)arg;
    while (1) {
        dpl_eventq_run(&ot->eventq);
    }
    return 0;
}

void
ot_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"ot_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(UWB_DEVICE_0)
    ot_init(uwb_dev_idx_lookup(0));
#endif
#if MYNEWT_VAL(UWB_DEVICE_1)
    ot_init(uwb_dev_idx_lookup(1));
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
    ot_init(uwb_dev_idx_lookup(2));
#endif
}

ot_instance_t *
ot_init(struct uwb_dev * inst)
{
    assert(inst);
    ot_instance_t *ot = (ot_instance_t*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_OT);

    if (ot == NULL){
        ot  = (ot_instance_t *) malloc(sizeof(ot_instance_t));
        assert(ot);
        memset(ot, 0x00, sizeof(ot_instance_t));
        ot->status.selfmalloc = 1;
    }
    ot->dev_inst = inst;
    ot->task_prio = inst->task_prio + 0x7;

    dpl_error_t err = dpl_sem_init(&ot->sem, 0x01);
    assert(err == DPL_OK);
    ot_global_inst = ot;

    dpl_eventq_init(&ot->eventq);
    dpl_task_init(&ot->task_str, "ot_task",
            ot_task,
            (void *)ot,
            ot->task_prio,
            DPL_WAIT_FOREVER,
            ot->task_stack,
            sizeof(ot->task_stack)/sizeof(ot->task_stack[0]));
    dpl_callout_init(&otc_task_callout, &ot->eventq, tasklet_sched , (void*)ot);
    RadioInit(ot);

    return ot;
}

void ot_post_init(struct uwb_dev* inst, otInstance *aInstance){
    ot_instance_t *ot = (ot_instance_t*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_OT);
    assert(ot);
    ot_global_inst = ot;
    ot->sInstance = aInstance;
    ot->status.initialized = 1;
}
