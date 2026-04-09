#include "vendors/motor_vendor.h"

#include "vendors/damiao/damiao_protocol.h"

static const motor_vendor_ops_t s_damiao_ops = {
    .name = "damiao",
    .build_scan_request = damiao_build_scan_request,
    .build_admin_frame = damiao_build_admin_frame,
    .build_control_frame = damiao_build_control_frame,
    .parse_feedback = damiao_parse_feedback,
};

const motor_vendor_ops_t *motor_vendor_get_damiao(void)
{
    return &s_damiao_ops;
}
