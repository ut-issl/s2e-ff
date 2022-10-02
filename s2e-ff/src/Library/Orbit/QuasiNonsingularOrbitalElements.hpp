#ifndef QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_
#define QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_

#include <Library/Orbit/OrbitalElements.h>

/**
 * @class QuasiNonsingularOrbitalElements
 * @brief Orbital elements avoid singularity when the eccentricity is near zero.
 */
class QuasiNonsingularOrbitalElements {
 public:
  QuasiNonsingularOrbitalElements(const OrbitalElements oe);
  ~QuasiNonsingularOrbitalElements();

  // Getter
  inline double GetSemiMajor() const { return semi_major_axis_m_; }
  inline double GetEccentricityX() const { return eccentricity_x_; }
  inline double GetEccentricityY() const { return eccentricity_y_; }
  inline double GetInclination() const { return inclination_rad_; }
  inline double GetRaan() const { return raan_rad_; }
  inline double GetMeanArgLatEpoch() const { return mean_arg_latitude_epoch_rad_; }

 private:
  double semi_major_axis_m_;
  double eccentricity_x_;               // e * cos(arg_peri)
  double eccentricity_y_;               // e * sin(arg_peri)
  double inclination_rad_;
  double raan_rad_;                     //!< Right Ascension of the Ascending Node [rad]
  double mean_arg_latitude_epoch_rad_;  //!< Mean argument of Latitude at epoch (arg_peri + mean anomaly) [rad]
};

#endif
