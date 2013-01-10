/**
 *  \file QuadFilter.cpp   \brief Filter for quad.
 *
 *  This file is generated by a script (core/tools/make-filter).
 *  Do not edit directly.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/InContainerQuadFilter.h"

IMPCONTAINER_BEGIN_NAMESPACE


InContainerQuadFilter
::InContainerQuadFilter(QuadContainer *c,
                             std::string name): QuadPredicate(name)
{
  c_=new internal::QuadContainerIndex(c, true);
}


InContainerQuadFilter
::InContainerQuadFilter(QuadContainer *c,
                             bool handle_permutations,
                             std::string name): QuadPredicate(name)
{
  c_=new internal::QuadContainerIndex(c, handle_permutations);
}


IMPCONTAINER_END_NAMESPACE
