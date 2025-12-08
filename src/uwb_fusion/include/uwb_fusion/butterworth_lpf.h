#ifndef UWB_FUSION_BUTTERWORTH_LPF_H_
#define UWB_FUSION_BUTTERWORTH_LPF_H_

#include <cmath>
#include <array>

namespace uwb_fusion {

/**
 * @brief Second-order Butterworth Low-Pass Filter
 * 
 * Implements a 2nd order IIR Butterworth filter with configurable cutoff frequency.
 * The filter coefficients are computed based on the sampling frequency and cutoff frequency.
 * 
 * Transfer function: H(s) = 1 / (s² + √2·s + 1)  (normalized)
 */
class ButterworthLPF {
 public:
  ButterworthLPF() : initialized_(false), sample_freq_(0), cutoff_freq_(0) {}

  /**
   * @brief Initialize the filter with sampling and cutoff frequencies
   * @param sample_freq Sampling frequency in Hz
   * @param cutoff_freq Cutoff frequency in Hz (-3dB point)
   */
  void initialize(double sample_freq, double cutoff_freq) {
    sample_freq_ = sample_freq;
    cutoff_freq_ = cutoff_freq;
    
    // Compute filter coefficients using bilinear transform
    // Pre-warp the cutoff frequency for bilinear transform
    double wc = 2.0 * M_PI * cutoff_freq;  // Angular cutoff frequency
    double T = 1.0 / sample_freq;           // Sampling period
    
    // Pre-warped frequency
    double wc_warped = (2.0 / T) * std::tan(wc * T / 2.0);
    
    // Normalized frequency
    double K = wc_warped * T / 2.0;
    double K2 = K * K;
    double sqrt2 = std::sqrt(2.0);
    
    // Denominator: 1 + √2·K + K²
    double denom = 1.0 + sqrt2 * K + K2;
    
    // Filter coefficients (Direct Form II)
    // y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
    b_[0] = K2 / denom;
    b_[1] = 2.0 * K2 / denom;
    b_[2] = K2 / denom;
    
    a_[1] = 2.0 * (K2 - 1.0) / denom;
    a_[2] = (1.0 - sqrt2 * K + K2) / denom;
    
    // Reset state
    reset();
    initialized_ = true;
  }

  /**
   * @brief Reset filter state (call when there's a discontinuity)
   */
  void reset() {
    x_hist_.fill(0.0);
    y_hist_.fill(0.0);
  }

  /**
   * @brief Initialize filter state with a value (avoids startup transient)
   * @param value Initial value to set
   */
  void initializeState(double value) {
    // Set history to steady-state values
    x_hist_.fill(value);
    y_hist_.fill(value);
  }

  /**
   * @brief Apply filter to a single sample
   * @param input New input sample
   * @return Filtered output
   */
  double filter(double input) {
    if (!initialized_) {
      return input;  // Pass-through if not initialized
    }

    // Direct Form II Transposed
    double output = b_[0] * input + b_[1] * x_hist_[0] + b_[2] * x_hist_[1]
                  - a_[1] * y_hist_[0] - a_[2] * y_hist_[1];

    // Update history
    x_hist_[1] = x_hist_[0];
    x_hist_[0] = input;
    y_hist_[1] = y_hist_[0];
    y_hist_[0] = output;

    return output;
  }

  /**
   * @brief Check if filter is initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Get the current cutoff frequency
   */
  double getCutoffFreq() const { return cutoff_freq_; }

  /**
   * @brief Get the filter delay in samples (approximate)
   * For 2nd order Butterworth: ~0.35 / fc seconds
   */
  double getDelaySeconds() const {
    return 0.35 / cutoff_freq_;
  }

 private:
  bool initialized_;
  double sample_freq_;
  double cutoff_freq_;
  
  // Filter coefficients
  std::array<double, 3> b_;  // Numerator (feedforward)
  std::array<double, 3> a_;  // Denominator (feedback), a_[0] = 1 (implicit)
  
  // State history
  std::array<double, 2> x_hist_;  // Input history [x[n-1], x[n-2]]
  std::array<double, 2> y_hist_;  // Output history [y[n-1], y[n-2]]
};

/**
 * @brief Wrapper for filtering spherical coordinates (range, azimuth, elevation)
 */
class SphericalLPF {
 public:
  SphericalLPF() : initialized_(false) {}

  /**
   * @brief Initialize all three filters
   * @param sample_freq Expected sample frequency in Hz
   * @param cutoff_freq Cutoff frequency in Hz (typically 1-5 Hz for human motion)
   */
  void initialize(double sample_freq, double cutoff_freq) {
    range_filter_.initialize(sample_freq, cutoff_freq);
    azimuth_filter_.initialize(sample_freq, cutoff_freq);
    elevation_filter_.initialize(sample_freq, cutoff_freq);
    initialized_ = true;
  }

  /**
   * @brief Initialize filter states to avoid startup transient
   */
  void initializeState(double range, double azimuth, double elevation) {
    range_filter_.initializeState(range);
    azimuth_filter_.initializeState(azimuth);
    elevation_filter_.initializeState(elevation);
  }

  /**
   * @brief Filter a spherical measurement
   * @param range_in Input range
   * @param azimuth_in Input azimuth (radians)
   * @param elevation_in Input elevation (radians)
   * @param range_out Filtered range
   * @param azimuth_out Filtered azimuth
   * @param elevation_out Filtered elevation
   */
  void filter(double range_in, double azimuth_in, double elevation_in,
              double& range_out, double& azimuth_out, double& elevation_out) {
    if (!initialized_) {
      range_out = range_in;
      azimuth_out = azimuth_in;
      elevation_out = elevation_in;
      return;
    }

    range_out = range_filter_.filter(range_in);
    azimuth_out = azimuth_filter_.filter(azimuth_in);
    elevation_out = elevation_filter_.filter(elevation_in);
    
    // Clamp range to positive
    if (range_out < 0.1) range_out = 0.1;
  }

  void reset() {
    range_filter_.reset();
    azimuth_filter_.reset();
    elevation_filter_.reset();
  }

  bool isInitialized() const { return initialized_; }

 private:
  bool initialized_;
  ButterworthLPF range_filter_;
  ButterworthLPF azimuth_filter_;
  ButterworthLPF elevation_filter_;
};

}  // namespace uwb_fusion

#endif  // UWB_FUSION_BUTTERWORTH_LPF_H_