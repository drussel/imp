/**
 *  \file atom/BrownianDynamics.h
 *  \brief Simple molecular dynamics optimizer.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPATOM_BROWNIAN_DYNAMICS_H
#define IMPATOM_BROWNIAN_DYNAMICS_H

#include "config.h"
#include "internal/version_info.h"
#include "Diffusion.h"
#include "SimulationParameters.h"

#include <IMP/Particle.h>
#include <IMP/Optimizer.h>
#include <IMP/internal/units.h>
#include <IMP/algebra/Vector3D.h>

IMPATOM_BEGIN_NAMESPACE

// for swig
class SimulationParameters;

//! Simple Brownian dynamics optimizer.
/** The particles to be optimized must have optimizable x,y,z attributes
    and a non-optimizable "Stokes radius"; this optimizer assumes
    the score to be energy in kcal/mol, the xyz coordinates to be in
    angstroms and the diffusion coefficent be in cm^2/s

    Particles without optimized x,y,z and nonoptimized D are skipped.

    Currently, rigid bodies are not supported. The necessary information
    can be found at \xternal{en.wikipedia.org/wiki/Rotational_diffusion,
    the wikipedia}.

    BrownianDynamics uses a SimulationParameters particle to store the
    parameters of the simulation. Such a particle must be passed on
    creation. The BrownianDynamics object will at least see updates
    to the SimulationParamters particle which occur before the
    call to BrownianDynamics::optimize() or BrownianDynamics::simulate(),
    changing the the parameters during optimization has undefined
    results.

    The optimizer can either automatically determine which particles
    to use from the model or be passed a SingletonContainer for the
    particles. If such a container is passed, particles added to it
    during optimization state updates are handled properly.

    \see Diffusion
  */
class IMPATOMEXPORT BrownianDynamics : public Optimizer
{
public:
  //! Create the optimizer
  /** If sc is not null, that container will be used to find particles
      to move, otherwise the model will be searched.*/
  BrownianDynamics(SimulationParameters si,
                   SingletonContainer *sc=NULL);
  virtual ~BrownianDynamics();

  IMP_OPTIMIZER(internal::version_info);

  //! Simulate until the given time in fs
  double simulate(float time_in_fs);

  void set_max_force_change(double df) {
    IMP_check(df > 0, "The max change must be positive",
              ValueException);
    max_squared_force_change_=square(df);
  }
private:
  void copy_forces(SingletonContainer* sc, algebra::Vector3Ds &v) const;
  void copy_coordinates(SingletonContainer *sc, algebra::Vector3Ds &v) const;
  void revert_coordinates(SingletonContainer *sc, algebra::Vector3Ds &v);

  void take_step(SingletonContainer *sc, unit::Femtosecond dt);

  unit::Femtojoule kt() const;

  SingletonContainer* setup_particles();

  /* unit::KilocaloriePerAngstromPerMol
     get_force_scale_from_D(unit::SquareCentimeterPerSecond D) const;*/


  double max_squared_force_change_;
  SimulationParameters si_;
  Pointer<SingletonContainer> sc_;
};

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_BROWNIAN_DYNAMICS_H */
