/**
 *  \file SingletonsOptimizerState.cpp
 *  \brief Use a SingletonModifier applied to a SingletonContainer to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/container/SingletonsOptimizerState.h"
#include <utility>

IMPCONTAINER_BEGIN_NAMESPACE

SingletonsOptimizerState
::SingletonsOptimizerState(SingletonContainer *c,
                           SingletonModifier *gm,
                           std::string name):
  OptimizerState(name),
  c_(c)
{
  f_=gm;
}


void SingletonsOptimizerState::update()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_LOG(TERSE, "Begin SingletonsOptimizerState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP_CHECK_OBJECT(c_);
  ParticlesTemp set =c_->get_particles();
  f_->apply(set);

  IMP_LOG(TERSE, "End SingletonsOptimizerState::update" << std::endl);
}



void SingletonsOptimizerState::show(std::ostream &out) const {
  out << "SingletonsOptimizerState base" << std::endl;
}

IMPCONTAINER_END_NAMESPACE
