/**
 *  \file atom/BerendsenThermostatOptimizerState.h
 *  \brief Maintains temperature during molecular dynamics using a
 *         Berendsen thermostat.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_BERENDSEN_THERMOSTAT_OPTIMIZER_STATE_H
#define IMPATOM_BERENDSEN_THERMOSTAT_OPTIMIZER_STATE_H

#include "atom_config.h"
#include <IMP/Particle.h>
#include <IMP/base_types.h>
#include <IMP/OptimizerState.h>

IMPATOM_BEGIN_NAMESPACE

//! Maintains temperature during molecular dynamics.
/** The thermostat scales velocities using the algorithm described
    in H. J. C. Berendsen, J. P. M. Postma,
    W. F. van Gunsteren, A. DiNola, and J. R. Haak "Molecular dynamics
    with coupling to an external bath", Journal of Chemical Physics 81
    pp. 3684-3690 (1984).

    At each update, velocities are rescaled by \f[
    \lambda = \left[1 + \frac{\Delta t}{\tau_T}
                        \left( \frac{T}{T_k} -1\right)\right]^{1/2}
    \f]
    where \f$\Delta t\f$ is the molecular dynamics timestep, \f$\tau_T\f$
    is the coupling constant (in fs) of the thermostat, \f$T\f$ is the
    thermostat temperature, and \f$T_k\f$ is the instantaneous (kinetic)
    temperature of the dynamics. (This is equation 11 from the reference above.)
 */
class IMPATOMEXPORT BerendsenThermostatOptimizerState : public OptimizerState
{
 public:
  BerendsenThermostatOptimizerState(const Particles &pis,
                                    double temperature, double coupling,
                                    unsigned skip_steps);

  //! Set the number of update calls to skip between rescaling.
  void set_skip_steps(unsigned skip_steps) {
    skip_steps_ = skip_steps;
  }

  //! Set the particles to use.
  void set_particles(const Particles &pis) {
    pis_=pis;
  }

  //! Rescale the velocities now
  void rescale_velocities() const;

  IMP_OPTIMIZER_STATE(BerendsenThermostatOptimizerState);

private:
  Particles pis_;
  double temperature_, coupling_;
  unsigned skip_steps_;
  unsigned call_number_;

  //! Keys of the xyz velocities
  FloatKey vs_[3];
};

IMP_OBJECTS(BerendsenThermostatOptimizerState,
            BerendsenThermostatOptimizerStates);

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_BERENDSEN_THERMOSTAT_OPTIMIZER_STATE_H */
