/**
 *  \file VelocityScalingOptimizerState.cpp
 *  \brief Maintains temperature during molecular dynamics by velocity scaling.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/atom/LangevinThermostatOptimizerState.h>
#include <IMP/atom/MolecularDynamics.h>
#include <IMP/atom/Mass.h>
#include <IMP/random.h>
IMPATOM_BEGIN_NAMESPACE

LangevinThermostatOptimizerState
::LangevinThermostatOptimizerState(
                                   const ParticlesTemp &pis,
                                   Float temperature, double gamma) :
  pis_(pis.begin(), pis.end()), temperature_(temperature), gamma_(gamma)
{
  vs_[0] = FloatKey("vx");
  vs_[1] = FloatKey("vy");
  vs_[2] = FloatKey("vz");
}

void LangevinThermostatOptimizerState::update()
{
    rescale_velocities();
}
IMP_GCC_DISABLE_WARNING("-Wuninitialized")
void LangevinThermostatOptimizerState::rescale_velocities() const
{
  static const double gas_constant = 8.31441e-7;
  MolecularDynamics *md = dynamic_cast<MolecularDynamics *>(get_optimizer());
  double c1 = exp(-gamma_*md->get_last_time_step());
  double c2 = sqrt((1.0-c1)*gas_constant*temperature_);
  IMP_INTERNAL_CHECK(md, "Can only use velocity scaling with "
             "the molecular dynamics optimizer.");
  boost::normal_distribution<Float> mrng(0., 1.);
  boost::variate_generator<RandomNumberGenerator&,
                           boost::normal_distribution<Float> >
    sampler(random_number_generator, mrng);
  for (unsigned int i=0; i< pis_.size(); ++i) {
    Particle *p = pis_[i];
    double mass = Mass(p).get_mass();
    for (int i = 0; i < 3; ++i) {
      double velocity = p->get_value(vs_[i]);
      velocity = c1*velocity+c2*sqrt((c1+1.0)/mass)*sampler();
     p->set_value(vs_[i], velocity);
   }
 }
}

void LangevinThermostatOptimizerState::do_show(std::ostream &out) const
{
  out << "Langevin thermostat with " << temperature_ << std::endl;
}

IMPATOM_END_NAMESPACE
