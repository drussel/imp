/**
 *  \file CoverRefined.h
 *  \brief Cover a bond with a sphere.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_COVER_REFINED_H
#define IMPCORE_COVER_REFINED_H

#include "core_config.h"

#include "XYZR.h"
#include <IMP/Refiner.h>
#include <IMP/Pointer.h>
#include <IMP/Decorator.h>
#include <IMP/SingletonModifier.h>


IMPCORE_BEGIN_NAMESPACE

// for swig
class XYZR;

/** \brief This class sets the position and radius of each particle to
 enclose the refined.

 \see DerivativesFromRefined
 \see DerivativesToRefined
 \see CentroidOfRefined
 \see atom::CoverBond

 Set the coordinates and radius of the passed particle to cover the particles
 listed by the particle refiner.
 An example showing a how to use such a score state to maintain a cover
 of the atoms of a protein by a sphere per residue.
 \htmlinclude cover_particles.py

 \note The particle passed must be an XYZR with the given radius key.
 \uses{class CoverRefined, CGAL}
 */
class IMPCOREEXPORT CoverRefined: public SingletonModifier
{
  IMP::internal::OwnerPointer<Refiner> refiner_;
  FloatKey rk_;
  Float slack_;
public:
  //! Create with the given refiner and radius key
  /** Slack is the amount added to the radius.*/
  CoverRefined(Refiner *ref,
               FloatKey rk
               =XYZR::get_default_radius_key(),
               Float slack=0);

  //! Set how nmuch extra to add to the radius.
  void set_slack(Float slack) {
    slack_=slack;
  }

  IMP_SINGLETON_MODIFIER(CoverRefined);
};

//! A particle which covers a set of other particles.
/** A decorator which sets up a particle to be the centroid of a
    set of other particles.

    \usesconstraint
*/
IMP_SUMMARY_DECORATOR_DECL(Cover, XYZR, XYZs);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_COVER_REFINED_H */
