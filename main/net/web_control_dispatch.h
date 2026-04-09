#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

bool web_control_parse_float(const char *query, const char *key, float *out);
bool web_control_parse_vendor_name(const char *query, char *out, size_t out_len);
int web_control_parse_ids(const char *csv, uint8_t *ids, int max_ids);

void web_control_dispatch_admin(uint8_t id, uint8_t op, uint8_t mode);
void web_control_dispatch_set_gains(uint8_t id, float kp, float kd);
void web_control_dispatch_set_ids(uint8_t old_id, uint8_t new_id, bool store_after_set, bool verify_after_set);
void web_control_dispatch_mode(uint8_t id, uint8_t mode, float pos, float vel, float tau, float vlim, float ratio);

void web_control_pace_then_next(int idx, int total);
void web_control_log_action(const char *action,
                            const uint8_t *ids,
                            int n,
                            int mode,
                            float pos,
                            float vel,
                            float tau,
                            float vlim,
                            float ratio);
