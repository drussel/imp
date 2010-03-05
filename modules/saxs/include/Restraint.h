/**
 *  \file saxs/Restraint.h
 *  \brief Calculate score based on fit to SAXS profile.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPSAXS_RESTRAINT_H
#define IMPSAXS_RESTRAINT_H

#include "saxs_config.h"

#include <IMP/saxs/Score.h>
#include <IMP/saxs/Profile.h>

#include <IMP/core/rigid_bodies.h>

#include <IMP/Model.h>
#include <IMP/Restraint.h>
#include <IMP/VersionInfo.h>

IMPSAXS_BEGIN_NAMESPACE

//! Calculate score based on fit to SAXS profile
/** \ingroup exp_restraint

    The restraint takes rigid bodies into account, in order
    to speed up the calculations. Rigid body should be gived as single
    RigidBody Particle. Other, non-rigid body Particles can also be given.

    The shape of a rigid body is assumed to be defined by the set of
    atom::Hierarchy leaves of the atom::Hierarchy rooted at the rigid body
    particle.

    \par Algorithmic details:
    The distances between the atoms of rigid body do not change, therefore
    their contribution to the profile is pre-computed and stored.
 */
class IMPSAXSEXPORT Restraint : public IMP::Restraint
{
 public:
  //! Constructor
  /**
     \param[in] particles The particles participating in the fitting score
     \param[in] exp_profile  The experimental profile used in the fitting score
  */
  Restraint(const Particles& particles, const Profile& exp_profile);

  IMP_RESTRAINT(Restraint);

 private:
  void compute_profile(Profile& model_profile);
 private:
  Particles particles_; // non-rigid bodies particles
  std::vector<core::RigidBody> rigid_bodies_decorators_; //rigid bodies
  std::vector<Particles> rigid_bodies_; // rigid bodies particles
  Profile rigid_bodies_profile_; // non-changing part of the profile
  Pointer<Score> saxs_score_; // computes profiles and derivatives
};

IMPSAXS_END_NAMESPACE

#endif  /* IMPSAXS_RESTRAINT_H */
