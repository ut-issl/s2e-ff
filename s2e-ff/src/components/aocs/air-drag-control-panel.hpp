/**
 * @file air-drag-control-panel.hpp
 * @brief Class to change satellite air drag panel information
 */

#ifndef S2E_COMPONENTS_EXAMPLES_AIR_DRAG_CONTROL_PANEL_HPP_
#define S2E_COMPONENTS_EXAMPLES_AIR_DRAG_CONTROL_PANEL_HPP_

#include <simulation/spacecraft/structure/surface.hpp>
#include <components/base/component.hpp>
#include <library/logger/logger.hpp>

/**
 * @class AirDragControlPanel
 * @brief Class to change satellite air drag panel information
 */
class AirDragControlPanel : public Component, public ILoggable {
 public:
  /**
   * @fn AirDragControlPanel
   * @brief Constructor with power port
   * @param [in] clock_generator: Clock generator
   * @param [in] structure: Structure information
   */
  AirDragControlPanel(ClockGenerator* clock_generator, Surface* surface);
  /**
   * @fn ~AirDragControlPanel
   * @brief Destructor
   */
  ~AirDragControlPanel();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const override;

  /**
   * @fn SetArea_m2
   * @brief Set the area of the surface in square meters
   * @param [in] area_m2: Area in square meters
   */
  void SetArea_m2(double area_m2) { area_m2_ = area_m2; }

  /**
   * @fn SetAngle_deg
   * @brief Set the angle of the surface in degrees
   * @param [in] angle_deg: Angle in degrees
   */
  void SetAngle_deg(double angle_deg) { angle_deg_ = angle_deg; }

  protected:
  Surface* surface_;  //!< Surface information
  double area_m2_;   //!< Area of the surface in square meters
  double angle_deg_; //!< Rotation Angle of the surface in degrees
};

#endif  // S2E_COMPONENTS_EXAMPLES_AIR_DRAG_CONTROL_PANEL_HPP_
