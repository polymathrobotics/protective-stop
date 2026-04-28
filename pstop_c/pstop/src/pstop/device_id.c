
#include <string.h>

#include "pstop/device_id.h"

void
device_id_init(device_id_t *device_id)
{
    memset(device_id->data, 0, sizeof(device_id_t));
}

void
device_id_copy(device_id_t *device_id, const device_id_t *id)
{
    memcpy(device_id->data, id->data, DEVICE_ID_LENGTH);
}

int
device_id_cmp(const device_id_t *lhs, const device_id_t *rhs)
{
    return memcmp(lhs->data, rhs->data, DEVICE_ID_LENGTH);
}
