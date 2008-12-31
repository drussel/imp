/**
 *  \file ListPairContainer.h    \brief Store a list of ParticlePairs
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_LIST_PAIR_CONTAINER_H
#define IMPCORE_LIST_PAIR_CONTAINER_H

#include "config.h"
#include "internal/core_version_info.h"
#include <IMP/core/PairContainer.h>

IMPCORE_BEGIN_NAMESPACE

//! Store a list of ParticlePairs
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCOREEXPORT ListPairContainer : public PairContainer
{
public:
  //! construct and pass an initial set of particle_pairs
  ListPairContainer(const ParticlePairs &ps= ParticlePairs());

  virtual ~ListPairContainer();

  //! log n time
  virtual bool get_contains_particle_pair(ParticlePair vt) const;

  IMP_LIST(public, ParticlePair, particle_pair, ParticlePair);

  IMP::VersionInfo get_version_info() const {
    return internal::core_version_info;
  }

  virtual void show(std::ostream &out = std::cout) const;
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_LIST_PAIR_CONTAINER_H */
