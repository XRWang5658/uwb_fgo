#ifndef UWB_FUSION_UWB_PDOA_FACTOR_H_
#define UWB_FUSION_UWB_PDOA_FACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

namespace uwb_fusion {

/**
 * @brief Custom GTSAM Factor for UWB PDoA Tracking.
 * 
 * Measures a target's position (modeled as Pose3) from a known sensor location.
 * Measurement model: Spherical coordinates [Range, Azimuth, Elevation] in sensor frame.
 * 
 * Convention:
 * - Azimuth: atan2(y, x) in sensor frame (0 = forward, positive = left)
 * - Elevation: atan2(z, sqrt(x²+y²)) (positive = up)
 */
class UwbPdoaFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  gtsam::Pose3 sensor_pose_global_;  // Known sensor pose in global frame
  double measured_range_;
  double measured_azimuth_;
  double measured_elevation_;

 public:
  typedef boost::shared_ptr<UwbPdoaFactor> shared_ptr;

  UwbPdoaFactor(gtsam::Key key, const gtsam::Pose3& sensor_pose_global, 
                double measured_range, double measured_azimuth, double measured_elevation, 
                const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key),
        sensor_pose_global_(sensor_pose_global),
        measured_range_(measured_range),
        measured_azimuth_(measured_azimuth),
        measured_elevation_(measured_elevation) {}

  ~UwbPdoaFactor() override = default;

  gtsam::Vector evaluateError(const gtsam::Pose3& person_pose,
                              boost::optional<gtsam::Matrix&> H = boost::none) const override {
    
    // Get person's position in global frame
    gtsam::Point3 p_global = person_pose.translation();
    
    // Transform to sensor's local frame
    // p_local = R_sensor^T * (p_global - t_sensor)
    gtsam::Point3 p_local = sensor_pose_global_.transformTo(p_global);

    double x = p_local.x();
    double y = p_local.y();
    double z = p_local.z();

    // Compute predicted spherical coordinates
    double r = p_local.norm();
    const double eps = 1e-6;
    if (r < eps) r = eps;

    double xy2 = x*x + y*y;
    double rho = std::sqrt(xy2);
    if (rho < eps) rho = eps;

    double predicted_az = std::atan2(y, x);
    double predicted_el = std::atan2(z, rho);

    // Compute residuals
    gtsam::Vector3 error;
    error(0) = r - measured_range_;
    error(1) = predicted_az - measured_azimuth_;
    error(2) = predicted_el - measured_elevation_;
    
    // Normalize azimuth error to [-pi, pi]
    while (error(1) > M_PI) error(1) -= 2.0 * M_PI;
    while (error(1) < -M_PI) error(1) += 2.0 * M_PI;
    
    // Normalize elevation error to [-pi, pi]
    while (error(2) > M_PI) error(2) -= 2.0 * M_PI;
    while (error(2) < -M_PI) error(2) += 2.0 * M_PI;

    if (H) {
      // Jacobian H (3x6) = d(error)/d(person_pose)
      // 
      // Chain rule:
      // d(error)/d(pose) = d(error)/d(p_local) * d(p_local)/d(p_global) * d(p_global)/d(pose)
      //
      // Where:
      // - d(error)/d(p_local) = J_spherical (3x3)
      // - d(p_local)/d(p_global) = R_sensor^T (3x3)
      // - d(p_global)/d(pose) for Pose3 with perturbation δ = [ω, v]:
      //   p_global(X ⊕ δ) = R * Exp(ω) * t_body + t ≈ t + R * v (for point at body origin)
      //   But person_pose.translation() gives t directly, and perturbation is t + R*v
      //   So d(p_global)/d(v) = R_person, d(p_global)/d(ω) = 0

      double r2 = r * r;
      double r3 = r2 * r;
      
      // J_spherical: d(r, az, el) / d(x, y, z)
      gtsam::Matrix33 J_spherical;
      
      // d(range)/d(xyz)
      J_spherical(0, 0) = x / r;
      J_spherical(0, 1) = y / r;
      J_spherical(0, 2) = z / r;

      // d(azimuth)/d(xyz) where az = atan2(y, x)
      J_spherical(1, 0) = -y / xy2;
      J_spherical(1, 1) =  x / xy2;
      J_spherical(1, 2) =  0.0;

      // d(elevation)/d(xyz) where el = atan2(z, rho)
      // del/dx = d/dx[atan2(z, sqrt(x²+y²))] = -xz / (r² * rho)
      // del/dy = -yz / (r² * rho)
      // del/dz = rho / r²
      double r2_rho = r2 * rho;
      J_spherical(2, 0) = -x * z / r2_rho;
      J_spherical(2, 1) = -y * z / r2_rho;
      J_spherical(2, 2) = rho / r2;

      // d(p_local)/d(p_global) = R_sensor^T
      gtsam::Matrix33 R_sensor_T = sensor_pose_global_.rotation().transpose().matrix();

      // d(p_global)/d(pose)
      // For Pose3, the translation perturbation in local frame maps to global via R_person
      // d(translation)/d(δv) = R_person
      // d(translation)/d(δω) = 0 (rotation doesn't directly move the translation)
      gtsam::Matrix33 R_person = person_pose.rotation().matrix();

      // Full Jacobian: J_spherical * R_sensor^T * [d(p_global)/d(pose)]
      gtsam::Matrix33 J_pos = J_spherical * R_sensor_T;
      
      // H = [d(error)/d(ω), d(error)/d(v)]
      // Since we're tracking a POINT (only position matters), rotation perturbation = 0
      *H = gtsam::Matrix::Zero(3, 6);
      
      // Translation part: J_pos * R_person
      H->block<3, 3>(0, 3) = J_pos * R_person;
    }

    return error;
  }
  
  // For debugging
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "UwbPdoaFactor(" << keyFormatter(this->key()) << ")\n";
    std::cout << "  measured: r=" << measured_range_ << ", az=" << measured_azimuth_ * 180/M_PI 
              << "°, el=" << measured_elevation_ * 180/M_PI << "°\n";
  }
};

} // namespace uwb_fusion

#endif // UWB_FUSION_UWB_PDOA_FACTOR_H_