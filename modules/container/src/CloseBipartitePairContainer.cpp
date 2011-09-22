/**
 *  \file CloseBipartitePairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. Close rights reserved.
 *
 */

#include "IMP/container/CloseBipartitePairContainer.h"


IMPCONTAINER_BEGIN_NAMESPACE


CloseBipartitePairContainer::CloseBipartitePairContainer(SingletonContainer *a,
                                                         SingletonContainer *b,
                                                         double distance,
                                                         double slack):
  P(a,b,distance,slack){}

CloseBipartitePairContainer::CloseBipartitePairContainer(SingletonContainer *a,
                                                         SingletonContainer *b,
                                                         double distance,
                                                  core::ClosePairsFinder *,
                                                         double slack):
  P(a,b,distance, slack) {
}
IMPCONTAINER_END_NAMESPACE
