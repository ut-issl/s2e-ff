#ifndef QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_
#define QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_

#include <Library/Orbit/OrbitalElements.h>

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
  double eccentricity_x_;
  double eccentricity_y_;
  double inclination_rad_;
  double raan_rad_;                     //!< Right Ascension of the Ascending Node [rad]
  double mean_arg_latitude_epoch_rad_;  //!< Mean argument of Latitude at epoch [rad]
};

#endif
