/**
 *  \file MinimumCLASSNAMERestraint.h
 *  \brief Score based on the minimum score over a set of CLASSNAMEs
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_MINORMAXHEADERNAME_HEADERNAME_RESTRAINT_H
#define IMPCONTAINER_MINORMAXHEADERNAME_HEADERNAME_RESTRAINT_H

#include "container_config.h"
#include <IMP/Restraint.h>
#include <IMP/CLASSNAMEScore.h>
#include <IMP/CLASSNAMEContainer.h>
#include <IMP/restraint_macros.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Score based on the min or max CLASSNAMEScore over a set
/** The score is evaluated for each of the VALUETYPE in the container
    and the value of the min or max n scores is used. That is,
    if n is 1, the value of the restraint is the value of the min or max
    score over the container.
 */
class IMPCONTAINEREXPORT MINORMAXCLASSNAMERestraint
: public Restraint
{
  IMP::OwnerPointer<CLASSNAMEScore> f_;
  IMP::OwnerPointer<CLASSNAMEContainer> c_;
  unsigned int n_;
public:
  /** n is the number of LCMINORMAX scores to use.
   */
  MINORMAXCLASSNAMERestraint(CLASSNAMEScore *f,
                                 CLASSNAMEContainerAdaptor c,
                                 unsigned int n=1,
                                 std::string name
                                 ="MINORMAXCLASSNAMERestraint %1%");

  IMP_RESTRAINT(MINORMAXCLASSNAMERestraint);

  //! Set the number of lowest scores to use.
  void set_n(unsigned int n) { n_=n;}
#ifndef IMP_DOXYGEN
  Restraints do_create_current_decomposition() const;
  double unprotected_evaluate_if_good(DerivativeAccumulator *da,
                                      double max) const;
#endif
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MINORMAXHEADERNAME_HEADERNAME_RESTRAINT_H */
