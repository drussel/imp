/**
 *  \file MolecularDynamics.cpp  \brief Simple molecular dynamics optimizer.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/atom/MolecularDynamics.h>
#include <IMP/core/XYZ.h>

#include <IMP/log.h>
#include <IMP/random.h>
#include <boost/random/normal_distribution.hpp>

#include <cmath>

IMPATOM_BEGIN_NAMESPACE

MolecularDynamics::MolecularDynamics()
{
  initialize();
}

MolecularDynamics::MolecularDynamics(Model *m)
{
  initialize();
  set_model(m);
}

MolecularDynamics::MolecularDynamics(RestraintSet *rs)
{
  initialize();
  set_model(rs->get_model());
  rs_= rs;
}

void MolecularDynamics::initialize() {
  time_step_=4.0;
  degrees_of_freedom_=0;
  velocity_cap_=std::numeric_limits<Float>::max();
  cs_[0] = FloatKey("x");
  cs_[1] = FloatKey("y");
  cs_[2] = FloatKey("z");
  masskey_ = FloatKey("mass");
  vs_[0] = FloatKey("vx");
  vs_[1] = FloatKey("vy");
  vs_[2] = FloatKey("vz");
}

void MolecularDynamics::do_show(std::ostream &) const {
}

IMP_LIST_IMPL(MolecularDynamics, Particle, particle, Particle*,
              Particles,
              {
                if (0) std::cout << index;
                for (unsigned int i=0; i< 3; ++i) {
                  if (!obj->has_attribute(vs_[i])) {
                    obj->add_attribute(vs_[i], 0.0, false);
                  }
                }
              },{},{});


void MolecularDynamics::setup_particles()
{
  degrees_of_freedom_ = 0;
  clear_particles();

  for (Model::ParticleIterator it= get_model()->particles_begin();
       it != get_model()->particles_end(); ++it) {
    Particle *p= *it;
    if (p->has_attribute(cs_[0]) && p->get_is_optimized(cs_[0])
        && p->has_attribute(cs_[1]) && p->get_is_optimized(cs_[1])
        && p->has_attribute(cs_[2]) && p->get_is_optimized(cs_[2])
        && p->has_attribute(masskey_) && !p->get_is_optimized(masskey_)) {
      add_particle(p);
      degrees_of_freedom_ += 3;
    }
  }
}


//! Perform a single dynamics step.
void MolecularDynamics::step()
{
  // Assuming score is in kcal/mol, its derivatives in kcal/mol/angstrom,
  // and mass is in g/mol, conversion factor necessary to get accelerations
  // in angstrom/fs/fs from raw derivatives
  static const Float deriv_to_acceleration = -4.1868e-4;

  for (ParticleIterator iter = particles_begin();
       iter != particles_end(); ++iter) {
    Particle *p = *iter;
    Float invmass = 1.0 / p->get_value(masskey_);
    for (unsigned i = 0; i < 3; ++i) {
      Float coord = p->get_value(cs_[i]);
      Float dcoord = p->get_derivative(cs_[i]);

      // calculate velocity at t+(delta t/2) from that at t-(delta t/2)
      Float velocity = p->get_value(vs_[i]);
      velocity += dcoord * deriv_to_acceleration * invmass * time_step_;

      cap_velocity_component(velocity);
      p->set_value(vs_[i], velocity);

      // get atomic shift
      Float shift = velocity * time_step_;

      // calculate position at t+(delta t) from that at t
      p->set_value(cs_[i], coord + shift);
    }
  }
}


//! Optimize the model.
/** \param[in] max_steps   Maximum number of iterations before aborting.
    \return score of the final state of the model.
 */
Float MolecularDynamics::optimize(unsigned int max_steps)
{
  Model *model = get_model();
  Pointer<RestraintSet> rs;
  if (rs_) {
    rs=rs_;
  } else {
    rs= model->get_root_restraint_set();
  }
  setup_particles();

  // get initial system score
  Float score = rs->evaluate(true);

  for (unsigned int i = 0; i < max_steps; ++i) {
    update_states();
    step();
    score = rs->evaluate(true);
  }
  return score;
}

Float MolecularDynamics::get_kinetic_energy() const
{
  // Conversion factor to get energy in kcal/mol from velocities in A/fs and
  // mass in g/mol
  static const Float conversion = 1.0 / 4.1868e-4;

  Float ekinetic = 0.;
  for (ParticleConstIterator iter = particles_begin();
       iter != particles_end(); ++iter) {
    Particle *p = *iter;
    Float vx = p->get_value(vs_[0]);
    Float vy = p->get_value(vs_[1]);
    Float vz = p->get_value(vs_[2]);
    Float mass = p->get_value(masskey_);

    ekinetic += mass * (vx * vx + vy * vy + vz * vz);
  }
  return 0.5 * ekinetic * conversion;
}

Float MolecularDynamics::get_kinetic_temperature(Float ekinetic) const
{
  if (degrees_of_freedom_ == 0) {
    return 0.;
  } else {
    // E = (n/2)kT  n=degrees of freedom, k = Boltzmann constant
    // Boltzmann constant, in kcal/mol
    const Float boltzmann = 8.31441 / 4186.8;
    return 2.0 * ekinetic / (degrees_of_freedom_ * boltzmann);
  }
}

void MolecularDynamics::assign_velocities(Float temperature)
{
  // gas constant for mass in g/mol
  static const Float gas_constant = 8.31441e-7;

  setup_particles();
  Float mean = 0.0;

  for (ParticleIterator iter = particles_begin();
       iter != particles_end(); ++iter) {
    Particle *p = *iter;
    Float mass = p->get_value(masskey_);
    Float stddev = std::sqrt(gas_constant * temperature / mass);
    boost::normal_distribution<Float> mrng(mean, stddev);
    boost::variate_generator<RandomNumberGenerator&,
                             boost::normal_distribution<Float> >
        sampler(random_number_generator, mrng);

    for (int i = 0; i < 3; ++i) {
      p->set_value(vs_[i], sampler());
    }
  }
}

IMPATOM_END_NAMESPACE
