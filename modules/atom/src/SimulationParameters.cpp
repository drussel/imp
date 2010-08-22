/**
 *  \File SimulationParameters.cpp   \brief Simple atoms decorator.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/atom/SimulationParameters.h>
#include <IMP/atom/Hierarchy.h>
#include <IMP/atom/Chain.h>
#include <IMP/core/XYZ.h>

#include <IMP/log.h>

#include <sstream>
#include <vector>
#include <limits>

IMPATOM_BEGIN_NAMESPACE


SimulationParameters SimulationParameters::setup_particle(Particle *p,
                                                  double dt, double T) {
  p->add_attribute(get_current_time_key(), 0, false);
  p->add_attribute(get_last_time_step_key(), 0, false);
  p->add_attribute(get_temperature_key(), T, false);
  p->add_attribute(get_maximum_time_step_key(), dt, false);
  IMP_USAGE_CHECK(dt > 0, "Time step must be positive");
  IMP_USAGE_CHECK(T > 0, "Temperature mustbe positive");
  return SimulationParameters(p);
}

void SimulationParameters::show(std::ostream &out) const
{
  out << "Current time is " << get_current_time() << "\n";
  out << "Last time step was " << get_last_time_step() << "\n";
  out << "Temperature is " << get_temperature() << "\n";
  out << std::endl;
}


FloatKey SimulationParameters::get_temperature_key() {
  static FloatKey k("temperature");
  return k;
}

FloatKey SimulationParameters::get_current_time_key() {
  static FloatKey k("current_time");
  return k;
}

FloatKey SimulationParameters::get_last_time_step_key() {
  static FloatKey k("last_time_step");
  return k;
}

FloatKey SimulationParameters::get_maximum_time_step_key() {
  static FloatKey k("maximum_time_step");
  return k;
}

unit::Femtojoule SimulationParameters::get_kT() const {
  return IMP::unit::Femtojoule(IMP::internal::KB*get_temperature_with_units());
}


IMPATOM_END_NAMESPACE
