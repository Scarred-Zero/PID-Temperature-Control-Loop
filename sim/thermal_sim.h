/**
 * @file    thermal_sim.h
 * @brief   Software thermal simulation — first-order lumped capacitance model.
 *
 *  Physics model:
 *
 *    dT/dt = (P_heater - P_loss) / C_thermal
 *
 *  Where:
 *    P_heater = duty_percent/100 × P_max_watts   (heat input from PWM)
 *    P_loss   = (T_chamber - T_ambient) / R_thermal (Newton's law of cooling)
 *    C_thermal = thermal capacitance (J/°C) — how much energy to heat 1°C
 *
 *  Discretised for fixed dt:
 *    T[n+1] = T[n] + dt × ( (duty/100 × P_max) - (T[n]-T_amb)/R_th ) / C_th
 *
 *  This is Euler forward integration — sufficient accuracy for dt=0.1s
 *  when the thermal time constant (tau = R_th × C_th) is tens of seconds.
 *
 *  Parameter selection rationale:
 *    These values approximate a small (1–2 litre) insulated chamber
 *    with a 10W cartridge heater — a common benchtop test rig.
 *    Adjust P_MAX_WATTS and THERMAL_RESISTANCE to match your hardware.
 */

#ifndef THERMAL_SIM_H
#define THERMAL_SIM_H

#include <stdint.h>

/* ── Physical Parameters of the Simulated Chamber ──────────────────────────
 *
 *  Thermal time constant: tau = R_thermal × C_thermal
 *                              = 5.0 × 400.0 = 2000 s ... too slow for demo.
 *
 *  We scale to tau = 5.0 × 20.0 = 100 s — still physically plausible for
 *  a small well-insulated box, and reaches steady state in ~5 min simulation
 *  time (compressible to seconds in accelerated sim).
 * ─────────────────────────────────────────────────────────────────────────── */

/** Maximum heater power output at 100% PWM duty, in Watts. */
#define SIM_P_MAX_WATTS (10.0f)

/**
 * Thermal resistance of chamber walls (°C per Watt).
 * Higher = better insulated = slower cooling.
 * R_th = 5.0 °C/W means: at steady state with 1W input, chamber is 5°C above ambient.
 */
#define SIM_R_THERMAL_CPW (5.0f)

/**
 * Thermal capacitance of chamber (Joules per °C).
 * C_th = 20 J/°C — energy needed to raise chamber temperature by 1°C.
 * Thermal time constant: tau = R × C = 5.0 × 20.0 = 100 seconds.
 */
#define SIM_C_THERMAL_JPC (20.0f)

/** Ambient (room) temperature in °C — the natural equilibrium point. */
#define SIM_T_AMBIENT_C (25.0f)

/** Initial chamber temperature at simulation start (= ambient). */
#define SIM_T_INITIAL_C (SIM_T_AMBIENT_C)

/**
 * @brief  Simulation state structure.
 *         Initialised by Sim_Init(), updated by Sim_Step().
 */
typedef struct
{
    float temperature_c; /**< Current simulated chamber temperature (°C). */
} ThermalSim;

/**
 * @brief  Initialise the thermal simulation to ambient temperature.
 * @param  sim  Pointer to a ThermalSim instance.
 */
void Sim_Init(ThermalSim *sim);

/**
 * @brief  Advance the simulation by one time step.
 *
 *  Call once per control loop iteration, AFTER applying the new duty cycle.
 *  Returns the new simulated temperature — use as the input to PID_Compute().
 *
 * @param  sim           Pointer to an initialised ThermalSim.
 * @param  duty_percent  Current heater PWM duty (0–100).
 * @param  dt_s          Time step in seconds (must match SAMPLE_PERIOD_S).
 * @return Updated simulated temperature in °C.
 */
float Sim_Step(ThermalSim *sim, uint8_t duty_percent, float dt_s);

#endif /* THERMAL_SIM_H */