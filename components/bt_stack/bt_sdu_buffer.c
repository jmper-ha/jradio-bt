#include <stddef.h>

/* L2CAP gathers every segmented packet of an ERTM channel - the cover-art
 * channel is one - into a buffer of L2CAP_MAX_BUF_SIZE, 10 KB, whatever the
 * packet's size, and the phone sends a cover's packets without waiting for
 * any of them to be taken. A Samsung S24 FE sent them faster than they were
 * passed on: a few 10 KB buffers at once took the 90 KB the module had free,
 * the next one failed, the phone closed the channel, and once the module
 * restarted (2026-11-20). No ERTM channel here takes more than
 * L2CAP_USER_RX_BUF_SIZE - the stack sets max_rx_mtu from it, and only
 * l2c_fcr.c asks for this size - so that much is asked for instead. */
#define BT_SDU_BUFFER_ASKED (10240U + 24U)
#define BT_SDU_BUFFER_GIVEN (4096U + 16U + 16U)

void *__real_osi_malloc_func(size_t size);

void *__wrap_osi_malloc_func(size_t size)
{
    if (size == BT_SDU_BUFFER_ASKED) size = BT_SDU_BUFFER_GIVEN;
    return __real_osi_malloc_func(size);
}
