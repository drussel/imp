/**
 *  \file PairsOptimizerState.cpp
 *  \brief Use a PairModifier applied to a PairContainer to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/PairsOptimizerState.h"
#include <utility>

IMPCORE_BEGIN_NAMESPACE

PairsOptimizerState::PairsOptimizerState(PairContainer *c,
                                           PairModifier *gm):
  c_(c){
  f_=gm;
}


void PairsOptimizerState::update()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_LOG(TERSE, "Begin PairsOptimizerState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP_CHECK_OBJECT(c_);
  ParticlePairs set =c_->get_particle_pairs();
  std::for_each(set.begin(), set.end(),
                PairFunctor(f_));

  IMP_LOG(TERSE, "End PairsOptimizerState::update" << std::endl);
}



void PairsOptimizerState::show(std::ostream &out) const {
  out << "PairsOptimizerState base" << std::endl;
}

IMPCORE_END_NAMESPACE
