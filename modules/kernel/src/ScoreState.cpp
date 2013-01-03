/**
 *  \file ScoreState.cpp \brief Shared score state.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/base/log.h"
#include "IMP/ScoreState.h"
#include "IMP/Model.h"
#include "IMP/container_base.h"
#include "IMP/input_output.h"
#include "IMP/internal/utility.h"
#include <algorithm>
#include <cmath>
#include <limits>

IMP_BEGIN_NAMESPACE

ScoreState::ScoreState(std::string name) :
  ModelObject(name)
{
  order_=-1;
}

ScoreState::ScoreState(Model *m, std::string name) :
  ModelObject(m, name)
{
  order_=-1;
}


void ScoreState::before_evaluate() {
  do_before_evaluate();
}

void ScoreState::after_evaluate(DerivativeAccumulator *da) {
  do_after_evaluate(da);
}


namespace {
struct CompOrder {
  bool operator()(const ScoreState*a,
                  const ScoreState*b) const {
    IMP_INTERNAL_CHECK(a->order_ != -1 && b->order_!= -1,
                    "No order assigned yet.");
    return a->order_ < b->order_;
  }
};
}


ScoreStatesTemp get_update_order( ScoreStatesTemp in) {
  IMP_FUNCTION_LOG;
  if (in.empty()) return in;
  // make sure the order_ entries are up to date
  if (!in[0]->get_model()->get_has_dependencies()) {
    in[0]->get_model()->compute_dependencies();
  }
  std::sort(in.begin(), in.end(), CompOrder());
  IMP_IF_LOG(TERSE) {
    IMP_LOG(TERSE, "Order: [");
    for (unsigned int i=0; i<in.size(); ++i) {
      IMP_LOG(TERSE, in[i]->order_ << ": " << in[i]->get_name() << ",");
    }
    IMP_LOG(TERSE, "]" << std::endl);
  }
  return in;
}
#ifdef IMP_USE_DEPRECATED

 ParticlesTemp ScoreState::get_input_particles() const {
   IMP_DEPRECATED_FUNCTION(get_inputs());
   return IMP::get_input_particles(get_inputs());
 }
ContainersTemp ScoreState::get_input_containers() const {
  IMP_DEPRECATED_FUNCTION(get_inputs());
  return IMP::get_input_containers(get_inputs());
}
ParticlesTemp ScoreState::get_output_particles() const {
  IMP_DEPRECATED_FUNCTION(get_outputs());
  return IMP::get_output_particles(get_outputs());
}
ContainersTemp ScoreState::get_output_containers() const {
  IMP_DEPRECATED_FUNCTION(get_outputs());
  return IMP::get_output_containers(get_outputs());
}
#endif
IMP_END_NAMESPACE
