#ifndef PSTOP_REQUIREMENTS_TEST_UTILS_H
#define PSTOP_REQUIREMENTS_TEST_UTILS_H

#include <stdint.h>
#include <stdbool.h>

#include "pstop/pstop_application.h"
#include "pstop/pstop_msg.h"
#include "pstop/machine.h"

extern uint64_t get_time(void);
extern void set_time(uint64_t newtime);

extern remote_details_t is_operator_allowed(const device_id_t *id);
extern void set_operator_allowed(bool is_allowed, bool stop_only, uint64_t heartbeat_ms);

extern void robot_status(pstop_status_message_t status);
extern pstop_status_message_t get_last_status();
extern int get_robot_status_counter();
extern void reset_robot_status();

extern void log_error(uint64_t timestamp, const device_id_t *client, uint8_t message, pstop_error_t error);

extern void configure_app_defaults(pstop_application_t *app,
    const device_id_t *machine_id,
    uint16_t max_lost_messages, uint16_t max_missed_heartbeats,uint32_t stop_ok_delay_ms);

extern pstop_error_t send_bond(pstop_machine_t *machine, pstop_msg_t *resp,
    const device_id_t *remote, const device_id_t *local, uint32_t counter, uint64_t timestamp);

extern pstop_error_t send_stop(pstop_machine_t *machine, pstop_msg_t *resp,
    const device_id_t *remote, const device_id_t *local, uint32_t counter, uint64_t timestamp);

extern pstop_error_t send_ok(pstop_machine_t *machine, pstop_msg_t *resp,
    const device_id_t *remote, const device_id_t *local, uint32_t counter, uint64_t timestamp);

extern pstop_error_t send_unbond(pstop_machine_t *machine, pstop_msg_t *resp,
    const device_id_t *remote, const device_id_t *local, uint32_t counter, uint64_t timestamp);

#endif /* PSTOP_REQUIREMENTS_TEST_UTILS_H */
