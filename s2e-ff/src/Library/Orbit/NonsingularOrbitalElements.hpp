#ifndef NONSINGULAR_ORBITAL_ELEMENTS_H_
#define NONSINGULAR_ORBITAL_ELEMENTS_H_

#include <Library/Orbit/OrbitalElements.h>

class NonsingularOrbitalElements {
 public:
  NonsingularOrbitalElements(const OrbitalElements oe);
  ~NonsingularOrbitalElements();

  // Getter
  inline double GetSemiMajor() const { return semi_major_axis_m_; }
  inline double GetEccentricityX() const { return eccentricity_x_; }
  inline double GetEccentricityY() const { return eccentricity_y_; }
  inline double GetInclination() const { return inclination_rad_; }
  inline double GetRaan() const { return raan_rad_; }
  inline double GetArgLonEpoch() const { return arg_longitude_epoch_rad_; }

 private:
  double semi_major_axis_m_;
  double eccentricity_x_;
  double eccentricity_y_;
  double inclination_rad_;
  double raan_rad_;                 //!< Right Ascension of the Ascending Node [rad]
  double arg_longitude_epoch_rad_;  //!< Argument of Latitude at epoch (perigees) [rad]
};

#endif
