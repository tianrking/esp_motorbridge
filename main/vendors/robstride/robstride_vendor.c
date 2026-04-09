#include "vendors/motor_vendor.h"
#include "vendors/robstride/robstride_protocol.h"

static const motor_vendor_ops_t s_robstride_ops = {
    .name = "robstride",
    .build_scan_request = robstride_build_scan_request,
    .build_admin_frame = robstride_build_admin_frame,
    .build_control_frame = robstride_build_control_frame,
    .parse_feedback = robstride_parse_feedback,
};

const motor_vendor_ops_t *motor_vendor_get_robstride(void)
{
    return &s_robstride_ops;
}
