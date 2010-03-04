/**
 *  \file atom/StereochemistryPairFilter.h
 *  \brief A filter that excludes bonds, angles and dihedrals.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPATOM_STEREOCHEMISTRY_PAIR_FILTER_H
#define IMPATOM_STEREOCHEMISTRY_PAIR_FILTER_H

#include "atom_config.h"
#include <IMP/PairFilter.h>
#include <IMP/atom/internal/ExcludedPair.h>

IMPATOM_BEGIN_NAMESPACE

//! A filter that excludes bonds, angles and dihedrals.
/** This is to be used with a core::ClosePairsScoreState to exclude all
    stereochemical interactions between the particles of an atomic system.
    Call set_bonds() to exclude bonds (1-2 particle interactions), set_angles()
    to exclude angles (1-3 interactions) and set_dihedrals() to exclude
    dihedrals (1-4 interactions).
 */
class IMPATOMEXPORT StereochemistryPairFilter : public PairFilter
{
  typedef std::map<internal::ExcludedPair, Particle *> ExcludedMap;
  ExcludedMap excluded_map_;

  Particles bonds_, angles_, dihedrals_;

  void rebuild_map();

public:
  StereochemistryPairFilter();

  void set_bonds(const Particles &bonds) { bonds_ = bonds;  rebuild_map(); }
  void set_angles(const Particles &angles) { angles_ = angles; rebuild_map(); }
  void set_dihedrals(const Particles &dihedrals) {
    dihedrals_ = dihedrals; rebuild_map();
  }

  IMP_PAIR_FILTER(StereochemistryPairFilter);
};


IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_STEREOCHEMISTRY_PAIR_FILTER_H */
