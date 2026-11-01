#ifndef SKID_PID_H
#define SKID_PID_H

#include "skid_motion.h"

void skm_pid_init(skm_internal_pid_t *state);
void skm_pid_reset(skm_internal_pid_t *state);
float skm_pid_calculate(skm_internal_pid_t *state,
                        const skm_pid_config_t *config,
                        float target,
                        float measurement,
                        float measurement_rate,
                        float dt_s);

#endif /* SKID_PID_H */
