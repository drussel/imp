/**
 *  \file ScoreState.cpp \brief Shared score state.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/base/log.h"
#include "IMP/ScoreState.h"
#include "IMP/internal/utility.h"
#include <cmath>
#include <limits>

IMP_BEGIN_NAMESPACE

ScoreState::ScoreState(std::string name) :
  Object(name)
{
  order_=-1;
}


void ScoreState::before_evaluate() {
  do_before_evaluate();
}


void ScoreState::after_evaluate(DerivativeAccumulator *da) {
  do_after_evaluate(da);
}

//! Give accesss to model particle data.
/** \param[in] model_data All particle data in the model.
 */
void ScoreState::set_model(Model* model)
{
  IMP_USAGE_CHECK(!model || !model_
            || (model_ && model_ == model),
            "Model* different from already stored model "
                  << model->get_name() << " " << model_->get_name());
  model_ = model;
  set_was_used(true);
}

IMP_END_NAMESPACE
