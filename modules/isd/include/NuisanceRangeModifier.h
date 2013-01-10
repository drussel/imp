/**
 *  \file IMP/isd/NuisanceRangeModifier.h
 *  \brief A singleton modifier which wraps an attribute into a
 *  given range.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPISD_NUISANCE_RANGE_MODIFIER_H
#define IMPISD_NUISANCE_RANGE_MODIFIER_H

#include <IMP/isd/isd_config.h>
#include <IMP/SingletonModifier.h>
#include <IMP/isd/Nuisance.h>
#include <IMP/singleton_macros.h>

IMPISD_BEGIN_NAMESPACE

class IMPISDEXPORT NuisanceRangeModifier: public SingletonModifier
{
public:
  NuisanceRangeModifier() {};

  // note, Doxygen wants a semicolon at the end of macro lines
  IMP_SINGLETON_MODIFIER(NuisanceRangeModifier);
};


IMPISD_END_NAMESPACE

#endif  /* IMPISD_NUISANCE_RANGE_MODIFIER_H */
