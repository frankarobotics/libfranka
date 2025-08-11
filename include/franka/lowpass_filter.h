// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cmath>

/**
 * @file lowpass_filter.h
 * Contains functions for filtering signals with a low-pass filter.
 */

namespace franka {
/**
 * Maximum cutoff frequency
 */
constexpr double kMaxCutoffFrequency = 1000.0;
/**
 * Default cutoff frequency
 */
constexpr double kDefaultCutoffFrequency = 100.0;
/**
 * Applies a first-order low-pass filter
 *
 * @param[in] sample_time Sample time constant
 * @param[in] current_signal_value Current value of the signal to be filtered
 * @param[in] last_signal_value Value of the signal to be filtered in the previous time step
 * @param[in] cutoff_frequency Cutoff frequency of the low-pass filter
 *
 * @throw std::invalid_argument if current_signal_value is infinite or NaN.
 * @throw std::invalid_argument if last_signal_value is infinite or NaN.
 * @throw std::invalid_argument if cutoff_frequency is zero, negative, infinite or NaN.
 * @throw std::invalid_argument if sample_time is negative, infinite or NaN.
 *
 * @return Filtered value.
 */
double lowpassFilter(double sample_time,
                     double current_signal_value,
                     double last_signal_value,
                     double cutoff_frequency);

/**
 * Applies a first-order low-pass filter to the translation and spherical linear interpolation
 * to the rotation of a transformation matrix which represents a Cartesian Motion.
 *
 * @param[in] sample_time Sample time constant
 * @param[in] current_signal_value Current Cartesian transformation matrix to be filtered
 * @param[in] last_signal_value Cartesian transformation matrix from the previous time step
 * @param[in] cutoff_frequency Cutoff frequency of the low-pass filter
 *
 * @throw std::invalid_argument if elements of current_signal_value is infinite or NaN.
 * @throw std::invalid_argument if elements of last_signal_value is infinite or NaN.
 * @throw std::invalid_argument if cutoff_frequency is zero, negative, infinite or NaN.
 * @throw std::invalid_argument if sample_time is negative, infinite or NaN.
 *
 * @return Filtered Cartesian transformation matrix.
 */

std::array<double, 16> cartesianLowpassFilter(double sample_time,
                                              std::array<double, 16> current_signal_value,
                                              std::array<double, 16> last_signal_value,
                                              double cutoff_frequency);
}  // namespace franka
