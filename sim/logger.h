/**
 * @file    logger.h
 * @brief   CSV serial logger for PID performance validation.
 *
 *  Output format (one line per control loop sample):
 *    timestamp_ms,setpoint_c,measured_c,error_c,pid_output_pct
 *
 *  Import into Excel, LibreOffice Calc, or plot with:
 *    Python: pandas.read_csv() + matplotlib
 *    gnuplot: plot 'log.csv' using 1:3 with lines
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

/**
 * @brief  Print the CSV header line. Call once before the control loop.
 */
void Logger_PrintHeader(void);

/**
 * @brief  Log one control loop sample as a CSV row.
 *
 * @param  timestamp_ms   Elapsed time in milliseconds since start.
 * @param  setpoint_c     Target temperature (°C).
 * @param  measured_c     Actual / simulated temperature (°C).
 * @param  pid_output_pct PID output (0.0–100.0 %).
 */
void Logger_LogSample(uint32_t timestamp_ms,
                      float setpoint_c,
                      float measured_c,
                      float pid_output_pct);

#endif /* LOGGER_H */