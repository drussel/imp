/**
 *  \file TripletScore.cpp  \brief Define TripletScore
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#include <IMP/TripletScore.h>
#include <IMP/internal/utility.h>

IMP_BEGIN_NAMESPACE

TripletScore::TripletScore(std::string name):
  Object(name)
{
  /* Implemented here rather than in the header so that PairScore
     symbols are present in the kernel DSO */
}

TripletScoreRestraint::TripletScoreRestraint(std::string name):
  Restraint(name){}

TripletsScoreRestraint::TripletsScoreRestraint(std::string name):
  DecomposableRestraint(name){}

IMP_END_NAMESPACE
