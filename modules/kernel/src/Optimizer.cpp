/**
 *  \file Optimizer.cpp   \brief Base class for all optimizers.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/base/log.h>
#include "IMP/Optimizer.h"
#include "IMP/OptimizerState.h"
#include "IMP/RestraintSet.h"
#include "IMP/generic.h"
#include "IMP/dependency_graph.h"
#include "IMP/internal/graph_utility.h"
#include "IMP/internal/RestraintsScoringFunction.h"
#include "IMP/internal/container_helpers.h"
#include "IMP/internal/utility.h"
#include <boost/tuple/tuple.hpp>
#include <limits>
#include <algorithm>

IMP_BEGIN_NAMESPACE

Optimizer::Optimizer(): Object("Optimizer%1%")
{
  set_was_used(true);
  min_score_= -std::numeric_limits<double>::max();
  stop_on_good_score_=false;
}

Optimizer::Optimizer(Model *m, std::string name): Object(name)
{
  if (m) set_model(m);
  set_was_used(true);
  min_score_= -std::numeric_limits<double>::max();
  stop_on_good_score_=false;
}

Optimizer::~Optimizer()
{
}

void Optimizer::set_model(Model *m) {
  cache_= m->create_model_scoring_function();
  model_=m;
}

void Optimizer::update_states() const
{
  IMP_LOG(VERBOSE,
          "Updating OptimizerStates " << std::flush);
  for (OptimizerStateConstIterator it = optimizer_states_begin();
       it != optimizer_states_end(); ++it) {
    IMP_CHECK_OBJECT(*it);
    (*it)->update();
    IMP_LOG(VERBOSE, "." << std::flush);
  }
  IMP_LOG(VERBOSE, "done." << std::endl);
}

double Optimizer::optimize(unsigned int max_steps) {
  IMP_FUNCTION_LOG;
  if (!model_) {
    IMP_THROW("Must give the optimizer a model to optimize",
              base::ValueException);
  }
  set_was_used(true);

  double ret= do_optimize(max_steps);
  return ret;
}

IMP_LIST_IMPL(Optimizer, OptimizerState, optimizer_state,
              OptimizerState*, OptimizerStates);

void Optimizer::set_optimizer_state_optimizer(OptimizerState *os, Optimizer *o)
{
  os->set_optimizer(o);
}


void Optimizer::set_restraints(const RestraintsTemp &rs) {
  set_scoring_function(new internal::RestraintsScoringFunction(rs));
}

void Optimizer::set_scoring_function(ScoringFunction *sf) {
  cache_= sf;
}


Restraints Optimizer::get_restraints() const {
  return cache_->create_restraints();
}

IMP_END_NAMESPACE
