#include "vendors/damiao/damiao_protocol.h"

#include "vendors/damiao/damiao_codec_utils.h"

bool damiao_parse_feedback(const twai_message_t *msg,
                           uint8_t *id,
                           float *pos,
                           float *vel,
                           float *torque,
                           float *t_mos,
                           float *t_rotor,
                           uint8_t *status)
{
    if (msg == NULL || msg->data_length_code < 8) {
        return false;
    }

    if (msg->identifier == MOTORBRIDGE_HOST_ADMIN_ID) {
        return false;
    }

    uint8_t can_id = msg->data[0] & 0x0F;
    uint8_t st = msg->data[0] >> 4;

    uint32_t pos_i = ((uint32_t)msg->data[1] << 8) | (uint32_t)msg->data[2];
    uint32_t vel_i = ((uint32_t)msg->data[3] << 4) | ((uint32_t)msg->data[4] >> 4);
    uint32_t torq_i = (((uint32_t)msg->data[4] & 0x0F) << 8) | (uint32_t)msg->data[5];

    if (id != NULL) {
        *id = can_id;
    }
    if (status != NULL) {
        *status = st;
    }
    if (pos != NULL) {
        *pos = damiao_uint_to_float(pos_i, -12.5f, 12.5f, 16);
    }
    if (vel != NULL) {
        *vel = damiao_uint_to_float(vel_i, -30.0f, 30.0f, 12);
    }
    if (torque != NULL) {
        *torque = damiao_uint_to_float(torq_i, -10.0f, 10.0f, 12);
    }
    if (t_mos != NULL) {
        *t_mos = (float)msg->data[6];
    }
    if (t_rotor != NULL) {
        *t_rotor = (float)msg->data[7];
    }

    return can_id > 0;
}
