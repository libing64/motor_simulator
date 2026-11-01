/**
 * @file foc_sim.cpp
 * @brief Bridge between the simulated motor and the SimpleFOC algorithm library.
 *
 * The three adapter classes below implement SimpleFOC's abstract hardware
 * interfaces.  Everything else - FOC transforms, dq current PI controllers,
 * velocity/position cascades, modulation (sine / SVPWM / six-step), voltage and
 * current limits - is the stock library code, compiled unmodified.
 */

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <new>

#include "Arduino.h"
#include "foc_sim.h"

#include "BLDCMotor.h"
#include "common/base_classes/BLDCDriver.h"
#include "common/base_classes/CurrentSense.h"
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

namespace {

inline float norm2pi(float a) {
    a = std::fmod(a, _2PI);
    if (a < 0.0f) a += _2PI;
    return a;
}

// ------------------------------------------------------------------ driver
class SimDriver : public BLDCDriver {
public:
    float ua = 0.0f, ub = 0.0f, uc = 0.0f;
    bool driver_enabled = false;

    int init() override {
        // sanity check for the voltage limit configuration (same as stock drivers)
        if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
            voltage_limit = voltage_power_supply;
        dc_a = dc_b = dc_c = 0.0f;
        initialized = true;
        return 1;
    }

    void enable() override { driver_enabled = true; }

    void disable() override {
        driver_enabled = false;
        setPwm(0.0f, 0.0f, 0.0f);
    }

    // Mirrors BLDCDriver6PWM::setPwm(): phase voltage -> duty cycle.
    void setPwm(float Ua, float Ub, float Uc) override {
        ua = _constrain(Ua, 0.0f, (float)voltage_limit);
        ub = _constrain(Ub, 0.0f, (float)voltage_limit);
        uc = _constrain(Uc, 0.0f, (float)voltage_limit);
        dc_a = _constrain(ua / (float)voltage_power_supply, 0.0f, 1.0f);
        dc_b = _constrain(ub / (float)voltage_power_supply, 0.0f, 1.0f);
        dc_c = _constrain(uc / (float)voltage_power_supply, 0.0f, 1.0f);
    }

    void setPhaseState(PhaseState, PhaseState, PhaseState) override {}
};

// ------------------------------------------------------------------ sensor
class SimSensor : public Sensor {
public:
    float angle = 0.0f;     // mechanical angle [rad], continuous
    float velocity = 0.0f;  // mechanical velocity [rad/s]

    void update() override {}
    void init() override {}

    float getSensorAngle() override { return norm2pi(angle); }
    float getMechanicalAngle() override { return norm2pi(angle); }
    float getAngle() override { return angle; }
    double getPreciseAngle() override { return static_cast<double>(angle); }
    float getVelocity() override { return velocity; }
    int needsSearch() override { return 0; }
};

// ------------------------------------------------------------------ current sense
class SimCurrentSense : public CurrentSense {
public:
    float ia = 0.0f, ib = 0.0f, ic = 0.0f;

    int init() override {
        gain_a = gain_b = gain_c = 1.0f;
        offset_ia = offset_ib = offset_ic = 0.0f;
        initialized = true;
        return 1;
    }

    PhaseCurrent_s getPhaseCurrents() override {
        PhaseCurrent_s c;
        c.a = ia;
        c.b = ib;
        c.c = ic;
        return c;
    }

    void enable() override {}
    void disable() override {}
    int driverAlign(float, bool) override { return 1; }
};

// ------------------------------------------------------------------ RNG
inline double gauss(unsigned int* state) {
    // Marsaglia polar method on top of a small LCG - deterministic and portable.
    double u, v, s;
    do {
        *state = (*state) * 1664525u + 1013904223u;
        u = ((*state >> 8) / 16777216.0) * 2.0 - 1.0;
        *state = (*state) * 1664525u + 1013904223u;
        v = ((*state >> 8) / 16777216.0) * 2.0 - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);
    return u * std::sqrt(-2.0 * std::log(s) / s);
}

}  // namespace

// BLDCMotor/FOCMotor keep a few POD members (feed-forward voltages/currents,
// monitor state, ...) that have no default initialiser, so a plain `new` leaves
// them holding whatever was on the heap.  Constructing on zeroed storage keeps
// the controller state deterministic.
static BLDCMotor* make_motor(int pole_pairs, float R, float KV, float Lq, float Ld) {
    void* mem = std::calloc(1, sizeof(BLDCMotor));
    if (!mem) return nullptr;
    BLDCMotor* m = new (mem) BLDCMotor(pole_pairs, R, KV, Lq, Ld);
    m->feed_forward_voltage = {0.0f, 0.0f};
    m->feed_forward_current = {0.0f, 0.0f};
    m->voltage = {0.0f, 0.0f};
    m->current = {0.0f, 0.0f};
    return m;
}

static void free_motor(BLDCMotor* m) {
    if (!m) return;
    m->~BLDCMotor();
    std::free(m);
}

struct foc_s {
    SimDriver drv;
    SimSensor sens;
    SimCurrentSense cs;
    BLDCMotor* motor = nullptr;

    unsigned long long t_us = 0;
    unsigned long period_us = 50;

    int encoder_bits = 0;
    double current_noise = 0.0;
    double angle_noise = 0.0;
    unsigned int rng = 12345u;
};

extern "C" {

foc_t* foc_create(void) { return new foc_s(); }

void foc_destroy(foc_t* f) {
    if (!f) return;
    free_motor(f->motor);
    delete f;
}

int foc_init(foc_t* f, const foc_params_t* p) {
    if (!f || !p) return 0;

    free_motor(f->motor);
    f->motor = nullptr;

    float KV = (p->kv_rating > 0.0) ? (float)p->kv_rating : NOT_SET;
    float Lq = (p->phase_inductance > 0.0) ? (float)p->phase_inductance : NOT_SET;
    f->motor = make_motor(p->pole_pairs, (float)p->phase_resistance, KV, Lq, Lq);
    if (!f->motor) return 0;

    f->period_us = (unsigned long)(p->control_period_us > 1.0 ? p->control_period_us : 50.0);

    f->drv.voltage_power_supply = (float)p->vbus;
    f->drv.voltage_limit = (float)p->voltage_limit;
    f->drv.pwm_frequency = (long)(1e6 / (double)f->period_us);

    f->motor->linkDriver(&f->drv);
    f->motor->linkSensor(&f->sens);
    f->motor->linkCurrentSense(&f->cs);

    f->motor->voltage_limit = (float)p->voltage_limit;
    f->motor->current_limit = (float)p->current_limit;
    f->motor->velocity_limit = (float)p->velocity_limit;

    f->encoder_bits = p->encoder_bits;
    f->current_noise = p->current_noise_std;
    f->angle_noise = p->angle_noise_std;
    f->rng = p->noise_seed ? p->noise_seed : 12345u;

    f->drv.init();
    f->cs.init();

    if (!f->motor->init()) return 0;

    // Sensor alignment is assumed to be known (bench with a calibrated
    // encoder).  Set the offset directly instead of running initFOC().
    f->motor->zero_electric_angle = (float)p->zero_electric_angle;
    f->motor->sensor_direction = Direction::CW;
    f->motor->motor_status = FOCMotorStatus::motor_ready;

    f->t_us = 0;
    sim_time_set_us(0);
    return 1;
}

void foc_set_controller(foc_t* f, int controller) {
    if (!f || !f->motor) return;
    switch (controller) {
        case FOC_CTRL_TORQUE: f->motor->controller = MotionControlType::torque; break;
        case FOC_CTRL_VELOCITY: f->motor->controller = MotionControlType::velocity; break;
        case FOC_CTRL_ANGLE: f->motor->controller = MotionControlType::angle; break;
        case FOC_CTRL_VELOCITY_OPENLOOP:
            f->motor->controller = MotionControlType::velocity_openloop;
            break;
        case FOC_CTRL_ANGLE_OPENLOOP:
            f->motor->controller = MotionControlType::angle_openloop;
            break;
        default: f->motor->controller = MotionControlType::torque; break;
    }
}

void foc_set_torque_mode(foc_t* f, int mode) {
    if (!f || !f->motor) return;
    switch (mode) {
        case FOC_TQ_VOLTAGE: f->motor->torque_controller = TorqueControlType::voltage; break;
        case FOC_TQ_DC_CURRENT: f->motor->torque_controller = TorqueControlType::dc_current; break;
        case FOC_TQ_FOC_CURRENT: f->motor->torque_controller = TorqueControlType::foc_current; break;
        case FOC_TQ_ESTIMATED_CURRENT:
            f->motor->torque_controller = TorqueControlType::estimated_current;
            break;
        default: f->motor->torque_controller = TorqueControlType::foc_current; break;
    }
}

void foc_set_modulation(foc_t* f, int modulation, int centered) {
    if (!f || !f->motor) return;
    switch (modulation) {
        case FOC_MOD_SINE: f->motor->foc_modulation = FOCModulationType::SinePWM; break;
        case FOC_MOD_SVPWM: f->motor->foc_modulation = FOCModulationType::SpaceVectorPWM; break;
        case FOC_MOD_TRAP120: f->motor->foc_modulation = FOCModulationType::Trapezoid_120; break;
        case FOC_MOD_TRAP150: f->motor->foc_modulation = FOCModulationType::Trapezoid_150; break;
        default: f->motor->foc_modulation = FOCModulationType::SpaceVectorPWM; break;
    }
    f->motor->modulation_centered = centered ? 1 : 0;
}

void foc_set_target(foc_t* f, double target) {
    if (f && f->motor) f->motor->target = (float)target;
}

void foc_set_pid(foc_t* f, int which, double P, double I, double D, double ramp, double limit) {
    if (!f || !f->motor) return;
    PIDController* pid = nullptr;
    switch (which) {
        case FOC_PID_VELOCITY: pid = &f->motor->PID_velocity; break;
        case FOC_PID_CURRENT_Q: pid = &f->motor->PID_current_q; break;
        case FOC_PID_CURRENT_D: pid = &f->motor->PID_current_d; break;
        case FOC_P_ANGLE: pid = &f->motor->P_angle; break;
        default: return;
    }
    pid->P = (float)P;
    pid->I = (float)I;
    pid->D = (float)D;
    pid->output_ramp = (float)ramp;
    pid->limit = (float)limit;
}

void foc_set_limits(foc_t* f, double voltage, double current, double velocity) {
    if (!f || !f->motor) return;
    if (voltage > 0.0) f->motor->updateVoltageLimit((float)voltage);
    if (current > 0.0) f->motor->updateCurrentLimit((float)current);
    if (velocity > 0.0) f->motor->updateVelocityLimit((float)velocity);
}

void foc_set_lpf_velocity(foc_t* f, double Tf) {
    if (f && f->motor) f->motor->LPF_velocity.Tf = (float)Tf;
}

void foc_enable(foc_t* f, int en) {
    if (!f || !f->motor) return;
    if (en) f->motor->enable();
    else f->motor->disable();
}

static void fill_state(foc_t* f, foc_state_t* out) {
    if (!out) return;
    out->ua = f->drv.ua;
    out->ub = f->drv.ub;
    out->uc = f->drv.uc;
    out->dc_a = f->drv.dc_a;
    out->dc_b = f->drv.dc_b;
    out->dc_c = f->drv.dc_c;
    out->id = f->motor->current.d;
    out->iq = f->motor->current.q;
    out->current_sp = f->motor->current_sp;
    out->ud = f->motor->voltage.d;
    out->uq = f->motor->voltage.q;
    out->shaft_angle = f->motor->shaft_angle;
    out->shaft_velocity = f->motor->shaft_velocity;
    out->electrical_angle = f->motor->electrical_angle;
    out->voltage_bemf = f->motor->voltage_bemf;
    out->enabled = f->motor->enabled;
}

int foc_step(foc_t* f, const double iabc[3], double shaft_angle, double shaft_velocity,
             double target, foc_state_t* out) {
    if (!f || !f->motor || !iabc) return 0;

    double angle = shaft_angle;
    if (f->encoder_bits > 0) {
        const double n = static_cast<double>(1 << f->encoder_bits);
        const double two_pi = 2.0 * M_PI;
        const double wrapped = std::fmod(shaft_angle, two_pi);
        const double q = std::floor(wrapped / two_pi * n + 0.5) * two_pi / n;
        angle = shaft_angle - wrapped + q;
    }
    if (f->angle_noise > 0.0) angle += f->angle_noise * gauss(&f->rng);

    f->sens.angle = (float)angle;
    f->sens.velocity = (float)shaft_velocity;

    f->cs.ia = (float)iabc[0];
    f->cs.ib = (float)iabc[1];
    f->cs.ic = (float)iabc[2];
    if (f->current_noise > 0.0) {
        const double n = f->current_noise;
        f->cs.ia += (float)(n * gauss(&f->rng));
        f->cs.ib += (float)(n * gauss(&f->rng));
        f->cs.ic += (float)(n * gauss(&f->rng));
    }

    sim_time_set_us(f->t_us);

    // Same order as a typical SimpleFOC control ISR, so that the telemetry we
    // report (current setpoint, dq voltages, duties) belongs to the very same
    // period as the PWM output.
    f->motor->move((float)target);
    f->motor->loopFOC();

    fill_state(f, out);

    f->t_us += f->period_us;
    return 1;
}

int foc_step_keep_target(foc_t* f, const double iabc[3], double shaft_angle,
                         double shaft_velocity, foc_state_t* out) {
    if (!f || !f->motor) return 0;
    return foc_step(f, iabc, shaft_angle, shaft_velocity, f->motor->target, out);
}

double foc_get_target(const foc_t* f) { return (f && f->motor) ? f->motor->target : 0.0; }

int foc_get_status(const foc_t* f) {
    return (f && f->motor) ? static_cast<int>(f->motor->motor_status) : -1;
}

}  // extern "C"
