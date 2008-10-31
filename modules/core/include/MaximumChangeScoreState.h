/**
 *  \file MaximumChangeScoreState.h
 *  \brief Keep track of the Maximumimum change of a set of attributes.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_MAXIMUM_CHANGE_SCORE_STATE_H
#define IMPCORE_MAXIMUM_CHANGE_SCORE_STATE_H

#include "core_exports.h"
#include "internal/core_version_info.h"
#include "ParticleContainer.h"

#include <IMP/ScoreState.h>
#include <IMP/Index.h>
#include <IMP/Particle.h>

#include <vector>

IMPCORE_BEGIN_NAMESPACE

//! Keeps track of the Maximumimum change of a set of attributes.
/** The score state maintains a list of particle and a list of
    float attribute keys and keeps track of the Maximumimum amount
    any of these have changed since the last time reset was called.

 */
class IMPCOREEXPORT MaximumChangeScoreState: public ScoreState
{
  typedef IMP::internal::
    AttributeTable<IMP::internal::FloatAttributeTableTraits> AT;
  FloatKeys keys_;
  std::map<ParticleIndex, AT> orig_values_;
  float maximum_change_;
  Pointer<ParticleContainer> pc_;
public:
  //! Track the changes with the specified keys.
  MaximumChangeScoreState(ParticleContainer *pc,
                          const FloatKeys &keys);

  virtual ~MaximumChangeScoreState();

  IMP_SCORE_STATE(internal::core_version_info);

  //! Measure differences from the current value.
  void reset();

  //! Return the maximum amount any attribute has changed.
  float get_maximum_change() const {
    return maximum_change_;
  }

  void set_particle_container(ParticleContainer *pc) {
    pc_=pc;
  }
  ParticleContainer *get_particle_container() const {
    return pc_;
  }

};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MAXIMUM_CHANGE_SCORE_STATE_H */
