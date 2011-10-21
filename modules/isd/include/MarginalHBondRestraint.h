/**
 *  \file isd/MarginalHBondRestraint.h
 *  \brief A lognormal restraint that uses the ISPA model to model HBond-derived
 *  distance fit.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPISD_MARGINAL_HBOND_RESTRAINT_H
#define IMPISD_MARGINAL_HBOND_RESTRAINT_H

#include "isd_config.h"
#include <IMP/isd/ISDRestraint.h>
#include <IMP/PairContainer.h>

IMPISD_BEGIN_NAMESPACE

//! Apply a lognormal distance restraint between two particles.
/** Marginal of the lognormal model for NOEs where only \f$\sigma\f$ was
    marginalized, and \f$\gamma\f$ was set to 1.
    Since the restraint is complicated, pass individual particles to
    add_contribution() command.

   \f[p(D|X,I) =
    \left(\sum_{i=1}^N \log^2\left(\frac{V_i^{exp}}
                     {d_i^{-6}(X)}\right)\right)^{-\frac{N}{2}}
    \f]

    The source code is as follows:
    \include MarginalHBondRestraint.h
    \include MarginalHBondRestraint.cpp
 */
class IMPISDEXPORT MarginalHBondRestraint : public ISDRestraint
{
  PairContainers contribs_;
  std::vector<double> volumes_;
  double logsquares_;
  void set_logsquares(double logsquares) {logsquares_=logsquares;}

public:
  //! Create the restraint.
  /** Restraints should store the particles they are to act on,
      preferably in a Singleton or PairContainer as appropriate.
   */
  MarginalHBondRestraint(){};


  // add a contribution: simple case
  void add_contribution(Particle *p1, Particle *p2, double Iexp);

  //add a contribution: general case
  void add_contribution(PairContainer *pc, double Iexp);

  //return the sum inside the parentheses
  double get_logsquares() const {return logsquares_;}

  unsigned get_number_of_contributions() const {return volumes_.size();}

  /* call for probability */
  double get_probability() const
  {
    return exp(-unprotected_evaluate(NULL));
  }


  /** This macro declares the basic needed methods: evaluate and show
   */
  IMP_RESTRAINT(MarginalHBondRestraint);


};

IMPISD_END_NAMESPACE

#endif  /* IMPISD_MARGINAL_HBOND_RESTRAINT_H */
