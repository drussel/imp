/**
 *  \file IMP/atom/VelocityScalingOptimizerState.h
 *  \brief Maintains temperature during molecular dynamics by velocity scaling.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_VELOCITY_SCALING_OPTIMIZER_STATE_H
#define IMPATOM_VELOCITY_SCALING_OPTIMIZER_STATE_H

#include <IMP/atom/atom_config.h>
#include <IMP/Particle.h>
#include <IMP/base_types.h>
#include <IMP/OptimizerState.h>
#include <IMP/optimizer_state_macros.h>

IMPATOM_BEGIN_NAMESPACE

//! Maintains temperature during molecular dynamics by velocity scaling.
/** This OptimizerState, when used with the MolecularDynamics optimizer,
    implements a simple thermostat by periodically rescaling the velocities.
    (Note that this results in discontinuous dynamics.)
    \see MolecularDynamics
 */
class IMPATOMEXPORT VelocityScalingOptimizerState : public OptimizerState
{
 public:
  VelocityScalingOptimizerState(const Particles &pis, Float temperature,
                                unsigned skip_steps);

  //! Set the number of update calls to skip between rescaling.
  void set_skip_steps(unsigned skip_steps) {
    skip_steps_ = skip_steps;
  }

  //! Set the particles to use.
  void set_particles(const Particles &pis) {
    pis_=pis;
  }

  //! Set the temperature to use.
  void set_temperature(Float temperature) {
    temperature_ = temperature;
  }

  //! Rescale the velocities now
  void rescale_velocities() const;

  IMP_OPTIMIZER_STATE(VelocityScalingOptimizerState);

private:
  Particles pis_;
  Float temperature_;
  unsigned skip_steps_;
  unsigned call_number_;

  //! Keys of the xyz velocities
  FloatKey vs_[3];
};

IMP_OBJECTS(VelocityScalingOptimizerState,VelocityScalingOptimizerStates);

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_VELOCITY_SCALING_OPTIMIZER_STATE_H */
