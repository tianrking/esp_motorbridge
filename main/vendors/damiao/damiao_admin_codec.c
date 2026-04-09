#include "vendors/damiao/damiao_protocol.h"

#include <string.h>

void damiao_build_scan_request(uint8_t id, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0xCC;
}

void damiao_build_admin_frame(uint8_t id, host_admin_opcode_t op, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = id;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    memset(out_msg->data, 0xFF, sizeof(out_msg->data));

    switch (op) {
    case HOST_OP_ENABLE:
        out_msg->data[7] = 0xFC;
        break;
    case HOST_OP_DISABLE:
        out_msg->data[7] = 0xFD;
        break;
    case HOST_OP_SET_ZERO:
        out_msg->data[7] = 0xFE;
        break;
    case HOST_OP_CLEAR_ERROR:
        out_msg->data[7] = 0xFB;
        break;
    default:
        out_msg->data[7] = 0xFD;
        break;
    }
}
