#include <stdlib.h>

#include "sysinit/sysinit.h"
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_hal.h>
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

struct os_callout task_callout;
static bool taskletprocess = false;

ot_instance_t * ot_global_inst;

void PlatformInit(struct _dw1000_dev_instance_t* inst){
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
    printf("# %s #\n",__func__);
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

static void tasklet_sched(struct os_event* ev){
    ot_instance_t* ot = (ot_instance_t*)ev->ev_arg;
    otInstance* aInstance = ot->sInstance;
    if(taskletprocess == true){
        taskletprocess = false;

        if(otTaskletsArePending(aInstance))
            otTaskletsProcess(aInstance);
        else
        {
            taskletprocess = true;
            os_callout_reset(&task_callout,OS_TICKS_PER_SEC/32);
        }
    }
}

void otTaskletsSignalPending(otInstance *aInstance)
{
    taskletprocess = true;
    if(ot_global_inst->status.initialized == 1)
        os_eventq_put(&ot_global_inst->eventq, &task_callout.c_ev);
    else{
        if(otTaskletsArePending(aInstance))
            otTaskletsProcess(aInstance);
        else
        {
            ot_global_inst->sInstance = aInstance;
            os_callout_reset(&task_callout,OS_TICKS_PER_SEC/32);
        }
    }
}

static void ot_task(void *arg)
{
    ot_instance_t *ot = (ot_instance_t*)arg;
    while (1) {
        os_eventq_run(&ot->eventq);
    }
}

void
ot_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"ot_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    ot_init(hal_dw1000_inst(0));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    ot_init(hal_dw1000_inst(1));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    ot_init(hal_dw1000_inst(2));
#endif
}

ot_instance_t *
ot_init(dw1000_dev_instance_t * inst){

	assert(inst);

	if (inst->ot == NULL ){
		inst->ot  = (ot_instance_t *) malloc(sizeof(ot_instance_t));
		assert(inst->ot);
        memset(inst->ot, 0x00, sizeof(ot_instance_t));
		inst->ot->status.selfmalloc = 1;
	}
	inst->ot->dev = inst;
    inst->ot->task_prio = inst->task_prio + 0x7;

	os_error_t err = os_sem_init(&inst->ot->sem, 0x01);
	assert(err == OS_OK);
	
    ot_global_inst = inst->ot;

    os_eventq_init(&inst->ot->eventq);
    os_task_init(&inst->ot->task_str, "ot_task",
            ot_task,
            (void *)inst->ot,
            inst->ot->task_prio,
            OS_WAIT_FOREVER,
            inst->ot->task_stack,
            DW1000_DEV_TASK_STACK_SZ * 4);
    os_callout_init(&task_callout, &inst->ot->eventq, tasklet_sched , (void*)inst->ot);
    RadioInit(inst);

    return inst->ot;
}

void ot_post_init(dw1000_dev_instance_t* inst, otInstance *aInstance){
    ot_global_inst = inst->ot;
    inst->ot->sInstance = aInstance;
	inst->ot->status.initialized = 1;
}
