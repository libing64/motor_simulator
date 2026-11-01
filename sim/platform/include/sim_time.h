#ifndef SFOC_SIM_TIME_H
#define SFOC_SIM_TIME_H
/**
 * @file sim_time.h
 * @brief Deterministic virtual clock shared by the Arduino shim and the
 *        simulation bridge.
 *
 * The simulated time is expressed in microseconds and never advances on its
 * own: the testbench calls sim_time_set_us() once per control period so that
 * SimpleFOC's PID sample time (Ts), velocity estimation and ballistics all see
 * a consistent, repeatable timebase.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Set the virtual clock to an absolute value [us]. */
void sim_time_set_us(unsigned long long t_us);
/** Read the virtual clock [us]. */
unsigned long long sim_time_get_us(void);
/** Advance the virtual clock by dt_us [us]. */
void sim_time_advance_us(unsigned long dt_us);

#ifdef __cplusplus
}
#endif

#endif  // SFOC_SIM_TIME_H
