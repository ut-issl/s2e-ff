/**
 * @file information_parser.hpp
 * @brief Information Parser
 */

#ifndef S2E_COMPONENTS_INFORMATION_PARSER_HPP_
#define S2E_COMPONENTS_INFORMATION_PARSER_HPP_

#include <components/base/component.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/vector.hpp>

#include "../../simulation/case/ff_inter_spacecraft_communication.hpp"
#include "./qpd_positioning_sensor.hpp"

/**
 * @class InformationParser
 * @brief Information Parser
 */
class InformationParser : public Component {
 public:
  /**
   * @fn InformationParser
   * @brief Constructor
   */
  InformationParser(const int prescaler, ClockGenerator* clock_gen, std::vector<QpdPositioningSensor*> qpd_positioning_sensors,
                    FfInterSpacecraftCommunication& inter_spacecraft_communication);

  /**
   * @fn ~InformationParser
   * @brief Destructor
   */
  ~InformationParser() {}

  // ComponentBase override function
  /**
   * @fn MainRoutine
   * @brief Main routine
   */
  void MainRoutine(int count);

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 protected:
  std::vector<QpdPositioningSensor*> qpd_positioning_sensors_;
  FfInterSpacecraftCommunication& inter_spacecraft_communication_;
};

#endif  // S2E_COMPONENTS_INFORMATION_PARSER_HPP_
