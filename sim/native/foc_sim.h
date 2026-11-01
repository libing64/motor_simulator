#ifndef FOC_SIM_H
#define FOC_SIM_H
/**
 * @file foc_sim.h
 * @brief C ABI that exposes a SimpleFOC BLDCMotor controller to the simulator.
 *
 * The library under test is SimpleFOC (third_party/Arduino-FOC).  Instead of
 * touching any hardware, the bridge provides three sub-classes of the
 * SimpleFOC abstract interfaces:
 *
 *   SimDriver        : BLDCDriver  -> captures Ua/Ub/Uc + duty cycles
 *   SimSensor        : Sensor      -> injects shaft angle / velocity
 *   SimCurrentSense  : CurrentSense-> injects measured phase currents
 *
 * so the *unmodified* library control code (FOC core, PIDs, SVPWM, six-step
 * commutation, limits) runs inside the simulation loop.
 *
 * All angles are in rad, currents in A, voltages in V, velocity in rad/s.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- MotionControlType (mirrors SimpleFOC) ---- */
#define FOC_CTRL_TORQUE 0
#define FOC_CTRL_VELOCITY 1
#define FOC_CTRL_ANGLE 2
#define FOC_CTRL_VELOCITY_OPENLOOP 3
#define FOC_CTRL_ANGLE_OPENLOOP 4

/* ---- TorqueControlType (mirrors SimpleFOC) ---- */
#define FOC_TQ_VOLTAGE 0
#define FOC_TQ_DC_CURRENT 1
#define FOC_TQ_FOC_CURRENT 2
#define FOC_TQ_ESTIMATED_CURRENT 3

/* ---- FOCModulationType (mirrors SimpleFOC) ---- */
#define FOC_MOD_SINE 0
#define FOC_MOD_SVPWM 1
#define FOC_MOD_TRAP120 2
#define FOC_MOD_TRAP150 3

/* ---- PID selectors ---- */
#define FOC_PID_VELOCITY 0
#define FOC_PID_CURRENT_Q 1
#define FOC_PID_CURRENT_D 2
#define FOC_P_ANGLE 3

typedef struct {
    int pole_pairs;
    double phase_resistance;   /* [Ohm]      */
    double phase_inductance;   /* [H]        */
    double kv_rating;          /* [rpm/V], <=0 -> not set */
    double vbus;               /* [V]        */
    double voltage_limit;      /* [V]        */
    double current_limit;      /* [A]        */
    double velocity_limit;     /* [rad/s]    */
    double control_period_us;  /* control loop period [us] */
    /* sensor / measurement realism (0 disables) */
    int encoder_bits;          /* quantisation of the shaft angle, 0 = ideal */
    double current_noise_std;  /* [A] gaussian noise added to phase currents */
    double angle_noise_std;    /* [rad] gaussian noise added to the angle  */
    unsigned int noise_seed;
    double zero_electric_angle; /* pre-aligned sensor offset [rad] */
} foc_params_t;

typedef struct {
    double ua, ub, uc;      /* commanded phase voltages [V]  */
    double dc_a, dc_b, dc_c;/* duty cycles [0..1]            */
    double id, iq;          /* measured dq currents [A]      */
    double current_sp;      /* q current setpoint [A]        */
    double ud, uq;          /* dq voltages [V]               */
    double shaft_angle;     /* [rad]                         */
    double shaft_velocity;  /* [rad/s]                       */
    double electrical_angle;/* [rad]                         */
    double voltage_bemf;    /* [V]                           */
    int enabled;
} foc_state_t;

typedef struct foc_s foc_t;

foc_t* foc_create(void);
void foc_destroy(foc_t* f);

/** Configure the motor object and initialise it (driver init + enable). */
int foc_init(foc_t* f, const foc_params_t* p);

void foc_set_controller(foc_t* f, int controller);
void foc_set_torque_mode(foc_t* f, int mode);
void foc_set_modulation(foc_t* f, int modulation, int centered);
void foc_set_target(foc_t* f, double target);
void foc_set_pid(foc_t* f, int which, double P, double I, double D, double ramp, double limit);
void foc_set_limits(foc_t* f, double voltage, double current, double velocity);
void foc_set_lpf_velocity(foc_t* f, double Tf);
void foc_enable(foc_t* f, int en);

/**
 * Run one control period: push measurements in, run loopFOC() + move(),
 * pop the resulting phase voltages and telemetry out.
 *
 * @param iabc          measured phase currents [A] (3)
 * @param shaft_angle   mechanical shaft angle [rad] (continuous)
 * @param shaft_velocity mechanical shaft velocity [rad/s]
 * @param target        controller target for this period
 * @param out           telemetry (may be NULL)
 * @return 1 on success
 */
int foc_step(foc_t* f, const double iabc[3], double shaft_angle, double shaft_velocity,
             double target, foc_state_t* out);

/** Run one control period without moving the target. */
int foc_step_keep_target(foc_t* f, const double iabc[3], double shaft_angle,
                         double shaft_velocity, foc_state_t* out);

double foc_get_target(const foc_t* f);
int foc_get_status(const foc_t* f);

#ifdef __cplusplus
}
#endif

#endif  /* FOC_SIM_H */
