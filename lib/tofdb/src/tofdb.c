#include <string.h>
#include <os/mynewt.h>
#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#include <tofdb/tofdb.h>
#include <dw1000/dw1000_hal.h>

int tofdb_cli_register();

static struct tofdb_node nodes[MYNEWT_VAL(TOFDB_MAXNUM_NODES)]; /* ca 18b/node */

struct tofdb_node*
tofdb_get_nodes()
{
    return nodes;
}

int tofdb_get_tof(uint64_t euid, uint16_t addr, uint32_t *tof)
{
    if (!tof) {
        return OS_EINVAL;
    }
    for (int i=0;i<MYNEWT_VAL(TOFDB_MAXNUM_NODES);i++) {
        if (euid && euid == nodes[i].euid) {
            *tof = nodes[i].tof;
            goto ret;
        }
        if (addr && addr == nodes[i].addr) {
            *tof = nodes[i].tof;
            goto ret;
        }
    }
    return OS_ENOENT;
ret:
    return OS_OK;
}

int tofdb_set_tof(uint64_t euid, uint16_t addr, uint32_t tof)
{
    int i;
    /* See if this entry exist in our database already */
    for (i=0;i<MYNEWT_VAL(TOFDB_MAXNUM_NODES);i++) {
        if (euid && euid == nodes[i].euid) {
            nodes[i].tof = tof;
            nodes[i].last_updated = os_cputime_get32();
            goto ret;
        }
        if (addr && addr == nodes[i].addr) {
            nodes[i].tof = tof;
            nodes[i].last_updated = os_cputime_get32();
            goto ret;
        }
    }

    /* No match, look for a free spot */
    for (i=0;i<MYNEWT_VAL(TOFDB_MAXNUM_NODES);i++) {
        if (nodes[i].euid || nodes[i].addr) {
            continue;
        }
        
        nodes[i].euid = euid;
        nodes[i].addr = addr;
        nodes[i].last_updated = os_cputime_get32();
        nodes[i].tof = tof;
        goto ret;
    }

    if (i==MYNEWT_VAL(TOFDB_MAXNUM_NODES)) {
        return OS_ENOMEM;
    }
ret:
    return OS_OK;
}

uint32_t
ccp_cb(uint64_t euid, uint16_t short_addr)
{
    uint32_t tof=0;
    tofdb_get_tof(euid, short_addr, &tof);
    return tof;
}

void
tofdb_pkg_init(void)
{
    int rc;
#if MYNEWT_VAL(TOFDB_CLI)
    rc = tofdb_cli_register();
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    memset(nodes, 0, sizeof(nodes));
    /*  */
#if MYNEWT_VAL(CCP_ENABLED)
    
#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_ccp_set_tof_comp_cb(hal_dw1000_inst(0)->ccp, ccp_cb);
#endif
#endif
}
