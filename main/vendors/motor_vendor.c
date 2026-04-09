#include "vendors/motor_vendor.h"
#include <string.h>

extern const motor_vendor_ops_t *motor_vendor_get_damiao(void);
extern const motor_vendor_ops_t *motor_vendor_get_robstride(void);

void motor_vendor_registry_init(void)
{
    // nothing needed for static binding
}

const motor_vendor_ops_t *motor_vendor_get(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    
    if (strcmp(name, "damiao") == 0) {
        return motor_vendor_get_damiao();
    } else if (strcmp(name, "robstride") == 0) {
        return motor_vendor_get_robstride();
    }
    return NULL;
}
