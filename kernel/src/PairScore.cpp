/**
 *  \file PairScore.cpp  \brief Define PairScore
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#include <IMP/PairScore.h>
#include <IMP/internal/utility.h>

IMP_BEGIN_NAMESPACE

PairScore::PairScore(std::string name):
  Object(name)
{
  /* Implemented here rather than in the header so that PairScore
     symbols are present in the kernel DSO */
}

PairScoreRestraint::PairScoreRestraint(std::string name):
  Restraint(name){}

PairsScoreRestraint::PairsScoreRestraint(std::string name):
  DecomposableRestraint(name){}

IMP_END_NAMESPACE
