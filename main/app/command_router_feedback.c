#include "app/command_router_internal.h"

#include "core/motor_manager.h"
#include "esp_timer.h"

bool command_router_try_handle_feedback(const twai_message_t *msg)
{
    float pos = 0, vel = 0, torque = 0, t_mos = 0, t_rotor = 0;
    uint8_t status = 0;
    uint8_t parsed_id = 0;

    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
        motor_state_t m;
        if (motor_manager_get_state((uint8_t)i, &m) && m.vendor != NULL && m.vendor->parse_feedback != NULL) {
            if (m.vendor->parse_feedback(msg, &parsed_id, &pos, &vel, &torque, &t_mos, &t_rotor, &status)) {
                if (parsed_id == i) {
                    motor_manager_update_feedback(parsed_id, pos, vel, torque, t_mos, t_rotor, status);
                    if (parsed_id < (sizeof(g_cmd_seen_us) / sizeof(g_cmd_seen_us[0]))) {
                        g_cmd_seen_us[parsed_id] = esp_timer_get_time();
                    }
                    return true;
                }
            }
        }
    }

    return false;
}
