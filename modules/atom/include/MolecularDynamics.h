/**
 *  \file atom/MolecularDynamics.h
 *  \brief Simple molecular dynamics optimizer.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_MOLECULAR_DYNAMICS_H
#define IMPATOM_MOLECULAR_DYNAMICS_H

#include "atom_config.h"
#include "Simulator.h"
#include "atom_macros.h"
#include <IMP/Particle.h>
#include <IMP/Optimizer.h>

IMPATOM_BEGIN_NAMESPACE

//! Simple molecular dynamics optimizer.
/** The particles to be optimized must have optimizable x,y,z attributes
    and a non-optimizable mass attribute; this optimizer assumes the score
    to be energy in kcal/mol, the xyz coordinates to be in angstroms, and
    the mass to be in AMU (g/mol).

    Particles without optimized x,y,z and nonoptimized mass are skipped.
    \see VelocityScalingOptimizerState
    \see LangevinThermostatOptimizerState
    \see BerendsenThermostatOptimizerState
    \see RemoveRigidMotionOptimizerState
 */
class IMPATOMEXPORT MolecularDynamics : public Simulator
{
public:
  /** Score based on the provided model */
  MolecularDynamics(Model *m=NULL);

  //! \return the current kinetic energy of the system, in kcal/mol
  Float get_kinetic_energy() const;

  //! \return the current kinetic temperature of the system
  /** \param[in] ekinetic kinetic energy, e.g. from get_kinetic_energy()
   */
  Float get_kinetic_temperature(Float ekinetic) const;

  //! Set maximum velocity in A/fs
  /** At each dynamics time step, the absolute value of each velocity
      component is capped at this value. This prevents spurious strong forces
      (occasionally encountered with frustrated conformations) from causing
      large oscillations in the system.
      By default, velocities are not capped.

      \note The actual velocities that are capped are the half-step velocities
            in the velocity Verlet algorithm.
   */
  void set_velocity_cap(Float velocity_cap) { velocity_cap_ = velocity_cap; }

  //! Assign velocities representative of the given temperature
  void assign_velocities(Float temperature);
  IMP_SIMULATOR(MolecularDynamics);
private:
  void initialize();

  void setup_degrees_of_freedom(const ParticlesTemp &ps);

  //! First part of velocity Verlet (update coordinates and half-step velocity)
  void propagate_coordinates(const ParticlesTemp &ps, double step_size);

  //! Second part of velocity Verlet (update velocity)
  void propagate_velocities(const ParticlesTemp &ps, double step_size);

  //! Cap a velocity component to the maximum value.
  inline void cap_velocity_component(Float &vel) {
    if (vel >= 0.0) {
      vel = std::min(vel, velocity_cap_);
    } else {
      vel = std::max(vel, -velocity_cap_);
    }
  }

  //! Keys of the xyz velocities
  FloatKey vs_[3];

  //! Number of degrees of freedom in the system
  int degrees_of_freedom_;

  //! Maximum absolute value of a single velocity component
  Float velocity_cap_;
};

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_MOLECULAR_DYNAMICS_H */
