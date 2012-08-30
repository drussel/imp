/**
 *  \file CLASSNAMEScore.cpp  \brief Define CLASSNAMEScore
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/CLASSNAMEScore.h>
#include <IMP/internal/utility.h>
#include <IMP/Restraint.h>
#include <IMP/LCCLASSNAME_macros.h>
#include <IMP/internal/TupleRestraint.h>
IMP_BEGIN_NAMESPACE

CLASSNAMEScore::CLASSNAMEScore(std::string name):
  Object(name)
{
  /* Implemented here rather than in the header so that PairScore
     symbols are present in the kernel DSO */
}


Restraints
CLASSNAMEScore
::create_current_decomposition(ARGUMENTTYPE vt) const {
  return Restraints(1,
     internal::create_tuple_restraint(const_cast<CLASSNAMEScore*>(this),
                                     vt,
                                     get_name()));
}

IMP_END_NAMESPACE
