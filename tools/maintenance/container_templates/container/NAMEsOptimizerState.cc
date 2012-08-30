/**
 *  \file CLASSNAMEsOptimizerState.cpp
 *  \brief Use a CLASSNAMEModifier applied to a CLASSNAMEContainer to
 *  maintain an invariant
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/CLASSNAMEsOptimizerState.h"
#include <utility>

IMPCONTAINER_BEGIN_NAMESPACE

CLASSNAMEsOptimizerState
::CLASSNAMEsOptimizerState(CLASSNAMEContainerAdaptor c,
                           CLASSNAMEModifier *gm,
                           std::string name):
  OptimizerState(name),
  c_(c)
{
  f_=gm;
}


void CLASSNAMEsOptimizerState::update()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_LOG(TERSE, "Begin CLASSNAMEsOptimizerState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP_CHECK_OBJECT(c_);
  c_->apply(f_);

  IMP_LOG(TERSE, "End CLASSNAMEsOptimizerState::update" << std::endl);
}



void CLASSNAMEsOptimizerState::do_show(std::ostream &) const {
}

IMPCONTAINER_END_NAMESPACE
