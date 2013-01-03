/**
 * \file KinematicForestScoreState.h
 * \brief
 *
 * \authors Dina Schneidman, Barak Raveh
 * Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKINEMATICS_KINEMATIC_FOREST_SCORE_STATE_H
#define IMPKINEMATICS_KINEMATIC_FOREST_SCORE_STATE_H

#include "kinematics_config.h"
#include <IMP/ScoreState.h>
#include <IMP/base/warning_macros.h>

IMPKINEMATICS_BEGIN_NAMESPACE

class IMPKINEMATICSEXPORT KinematicForestScoreState : public IMP::ScoreState {
 public:

  IMP_OBJECT(KinematicForestScoreState);

  KinematicForestScoreState(KinematicForest *kf) : kf_(kf) {}

  void do_before_evaluate() {
    std::cerr << " before kf update " << std::endl;
    kf_->update_all_external_coordinates();
    std::cerr << " after kf update " << std::endl;
  }
  void do_after_evaluate(DerivativeAccumulator *da) { IMP_UNUSED(da); }
  ContainersTemp get_input_containers() const { return ContainersTemp(); }
  ContainersTemp get_output_containers() const { return ContainersTemp(); }
  ParticlesTemp get_input_particles() const {
    std::cerr << "get input particles " << std::endl;
    return ParticlesTemp(); }
  ParticlesTemp get_output_particles() const { return ParticlesTemp(); }

 private:
  KinematicForest *kf_;
};

IMPKINEMATICS_END_NAMESPACE

#endif /* IMPKINEMATICS_KINEMATIC_FOREST_SCORE_STATE_H */
