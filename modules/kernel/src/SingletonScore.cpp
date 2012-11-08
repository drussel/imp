/**
 *  \file SingletonScore.cpp  \brief Define SingletonScore
 *
 *  WARNING This file was generated from NAMEScore.cc
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/SingletonScore.h>
#include <IMP/internal/utility.h>
#include <IMP/Restraint.h>
#include <IMP/singleton_macros.h>
#include <IMP/internal/TupleRestraint.h>
IMP_BEGIN_NAMESPACE

SingletonScore::SingletonScore(std::string name):
  Object(name)
{
  /* Implemented here rather than in the header so that PairScore
     symbols are present in the kernel DSO */
}


Restraints
SingletonScore
::create_current_decomposition(Particle* vt) const {
  return internal::create_score_current_decomposition(this, vt);
}

IMP_INPUTS_DEF(SingletonScore);

IMP_END_NAMESPACE
