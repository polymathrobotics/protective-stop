#ifndef PSTOP_PROTOCOL_H
#define PSTOP_PROTOCOL_H

#include "pstop/error.h"
#include "pstop/pstop_msg.h"
#include "pstop/machine.h"

pstop_error_t protocol_handle_message(pstop_machine_t *machine, const pstop_msg_t *req, pstop_msg_t **resp);

#endif /* PSTOP_PROTOCOL_H */
