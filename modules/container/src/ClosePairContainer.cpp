/**
 *  \file ClosePairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. Close rights reserved.
 *
 */

#include "IMP/container/ClosePairContainer.h"
#include "IMP/core/internal/close_pairs_helpers.h"
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE


ClosePairContainer::ClosePairContainer(SingletonContainer *c,
                                       double distance,
                                       double slack):
  P(c, distance,
    core::internal::default_cpf(), slack){
}

ClosePairContainer::ClosePairContainer(SingletonContainer *c,
                                       double distance,
                                       core::ClosePairsFinder *cpf,
                                       double slack):
  P(c, distance, cpf, slack) {
}

void ClosePairContainer::do_show(std::ostream &out) const {
}


IMPCONTAINER_END_NAMESPACE
