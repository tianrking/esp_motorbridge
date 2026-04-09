#include "vendors/damiao/damiao_protocol.h"

#include <string.h>

void damiao_build_register_write_cmd(uint8_t motor_id, uint8_t rid, uint32_t value, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = motor_id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0x55;
    out_msg->data[3] = rid;
    memcpy(&out_msg->data[4], &value, sizeof(uint32_t));
}

void damiao_build_register_read_cmd(uint8_t motor_id, uint8_t rid, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = motor_id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0x33;
    out_msg->data[3] = rid;
}

void damiao_build_store_params_cmd(uint8_t motor_id, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = motor_id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0xAA;
    out_msg->data[3] = 0x01;
}

bool damiao_parse_register_reply(const twai_message_t *msg, uint8_t *rid, uint32_t *value)
{
    if (msg == NULL || msg->data_length_code < 8) {
        return false;
    }

    if (msg->data[2] != 0x33) {
        return false;
    }

    if (rid != NULL) {
        *rid = msg->data[3];
    }
    if (value != NULL) {
        uint32_t v = 0;
        memcpy(&v, &msg->data[4], sizeof(uint32_t));
        *value = v;
    }
    return true;
}
