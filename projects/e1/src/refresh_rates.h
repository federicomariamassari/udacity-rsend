#ifndef REFRESH_RATES_H
#define REFRESH_RATES_H

#include <iostream>
#include <iomanip>
#include <chrono>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <vector>


/**
 * @brief Print the header of table.
 */
void print_header();

/**
 * @brief Print a table line.
 * @
 * @param line The line to print.
 */
template<typename T>
void print_line(std::vector<T> &line);

/**
 * @brief Create a string coordinate pair (x=&x, y=&y) for pretty printing.
 * 
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * 
 * @returns The string coordinate pair.
 */
std::string make_coordinate_pair(float &x, float &y);

/**
 * @brief Fill out missing measurement entry with most recent available data point.
 * 
 * @param vec The structure containing the time series of measurement data.
 * @param result The filled-forward value.
 */
void fill_forward(std::vector<std::pair<float, float>> &vec, std::pair<float, float> &result);

/**
 * @brief Infer future value of an observation from the most recent available measurement and a data model.
 *   Assumes Constant Velocity Model (CVM): meas_(t + delta_t) = meas_(t) + velocity * delta_t.
 * 
 * @param vec The structure containing the time series of measurement data.
 * @param timestamps The structure containing the time series of timestamps.
 * @param velocity The constant velocity parameter.
 * @param result The extrapolated future value.
 * 
 * Resources:
 * 
 * [1] - https://en.wikipedia.org/wiki/Extrapolation
 */
void extrapolate(std::vector<std::pair<float, float>> &vec, std::vector<uint64_t> &timestamps, float velocity,
	std::pair<float, float> &result);

/**
 * @brief Interpolate data based on two endpoints.
 * 
 * @param vec The structure containing the time series of measurement data.
 * @param timestamps The structure containing the time series of timestamps.
 * @param result The interpolated value, delayed one time period.
 * 
 * Resources:
 * 
 * [1] - https://en.wikipedia.org/wiki/Linear_interpolation
 */
void interpolate(std::vector<std::pair<float, float>> &vec, std::vector<uint64_t> &timestamps, 
	std::pair<float, float> &result);

#endif  /* REFRESH_RATES_H */