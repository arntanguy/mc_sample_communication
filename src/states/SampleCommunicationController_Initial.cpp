#include "SampleCommunicationController_Initial.h"

#include "../SampleCommunicationController.h"

void SampleCommunicationController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void SampleCommunicationController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SampleCommunicationController &>(ctl_);
}

bool SampleCommunicationController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SampleCommunicationController &>(ctl_);
  output("OK");
  return true;
}

void SampleCommunicationController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SampleCommunicationController &>(ctl_);
}

EXPORT_SINGLE_STATE("SampleCommunicationController_Initial", SampleCommunicationController_Initial)
