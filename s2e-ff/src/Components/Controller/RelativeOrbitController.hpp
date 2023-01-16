#ifndef RELATIVE_ORBIT_CONTROLLER_H_
#define RELATIVE_ORBIT_CONTROLLER_H_

#include <Component/Abstract/ComponentBase.h>
#include <Interface/LogOutput/ILoggable.h>

class RelativeOrbitController : public ComponentBase, public ILoggable {
 public:
  RelativeOrbitController(const int prescaler, ClockGenerator* clock_gen);
  ~RelativeOrbitController();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 protected:
};

#endif
