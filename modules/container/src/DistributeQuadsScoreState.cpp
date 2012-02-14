/**
 *  \file PredicateQuadsRestraint.cpp   \brief Container for quad.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/DistributeQuadsScoreState.h"

IMPCONTAINER_BEGIN_NAMESPACE
DistributeQuadsScoreState::
DistributeQuadsScoreState(QuadContainer *input,
                               std::string name): ScoreState(name) {
  input_=input;
  updated_=false;
}

ParticlesTemp DistributeQuadsScoreState
::get_output_particles() const {
  return ParticlesTemp();
}
ContainersTemp DistributeQuadsScoreState
::get_output_containers() const {
  ContainersTemp ret(data_.size());
  for (unsigned int i=0; i< data_.size(); ++i) {
    ret[i]=data_[i].get<0>();
  }
  return ret;
}

ParticlesTemp DistributeQuadsScoreState
::get_input_particles() const {
  // not correct, but correct is complicated
  return input_->get_contained_particles();
}
ContainersTemp DistributeQuadsScoreState
::get_input_containers() const {
  // List containers don't do anything interesting
  return ContainersTemp(1, input_);
}


void DistributeQuadsScoreState::do_before_evaluate() {
  update_lists_if_necessary();
}
void DistributeQuadsScoreState
::do_after_evaluate(DerivativeAccumulator *) {
}

void DistributeQuadsScoreState
::update_lists_if_necessary() const {
  if (updated_ && !input_->get_contents_changed()) return;
  updated_=true;

  vector<ParticleIndexQuads> output(data_.size());
  IMP_FOREACH_QUAD_INDEX(input_, {
      for (unsigned int i=0; i< data_.size(); ++i) {
        if (data_[i].get<1>()->get_value_index(get_model(), _1)
            == data_[i].get<2>()) {
          output[i].push_back(_1);
        }
      }
    });
  for (unsigned int i=0; i< output.size(); ++i) {
    data_[i].get<0>()->set_particle_quads(output[i]);
  }
}

void DistributeQuadsScoreState::do_show(std::ostream &) const {
}
IMPCONTAINER_END_NAMESPACE
