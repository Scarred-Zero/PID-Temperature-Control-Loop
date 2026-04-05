/**
 * @file    thermal_sim.c
 * @brief   First-order thermal model — Euler integration, hardware-free.
 */

#include "thermal_sim.h"
#include <stddef.h>

void Sim_Init(ThermalSim *sim)
{
    if (sim == NULL)
    {
        return;
    }
    sim->temperature_c = SIM_T_INITIAL_C;
}

/**
 * @brief  Euler forward step of the thermal ODE.
 *
 *  dT/dt = [ P_in - P_loss ] / C_thermal
 *
 *  P_in   = (duty/100) × P_max                     [W]
 *  P_loss = (T_current - T_ambient) / R_thermal     [W]  (Newton cooling)
 *
 *  T_new  = T_old + dt × dT/dt
 *
 *  WHY Euler and not Runge-Kutta?
 *  The thermal time constant (100 s) is 1000× larger than dt (0.1 s).
 *  The dimensionless stability parameter: dt/tau = 0.1/100 = 0.001 — well
 *  within Euler's stability region. Higher-order integrators offer no
 *  meaningful accuracy improvement here.
 */
float Sim_Step(ThermalSim *sim, uint8_t duty_percent, float dt_s)
{
    if (sim == NULL)
    {
        return SIM_T_AMBIENT_C;
    }

    /* Power delivered to chamber by heater */
    float p_in = ((float)duty_percent / 100.0f) * SIM_P_MAX_WATTS;

    /* Heat lost through chamber walls to ambient (Newton's law) */
    float p_loss = (sim->temperature_c - SIM_T_AMBIENT_C) / SIM_R_THERMAL_CPW;

    /* Net rate of temperature change (°C/s) */
    float dT_dt = (p_in - p_loss) / SIM_C_THERMAL_JPC;

    /* Euler integration: advance temperature by one time step */
    sim->temperature_c += dT_dt * dt_s;

    return sim->temperature_c;
}