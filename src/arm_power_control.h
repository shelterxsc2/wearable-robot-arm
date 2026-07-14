#ifndef ARM_POWER_CONTROL_H
#define ARM_POWER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

int arm_power_init(void);
void arm_power_shutdown(void);
int arm_power_on(void);
int arm_power_request_voice_off(double timeout_seconds);
int arm_power_confirm_gesture(void);
int arm_power_off_safe(void);
int arm_power_is_on(void);
int arm_power_shutdown_in_progress(void);
void arm_power_tick(void);

#ifdef __cplusplus
}
#endif
#endif
