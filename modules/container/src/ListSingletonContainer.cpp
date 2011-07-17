/**
 *  \file ListSingletonContainer.cpp   \brief A list of ParticlesTemp.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/ListSingletonContainer.h"
#include "IMP/SingletonModifier.h"
#include "IMP/SingletonScore.h"
#include <IMP/core/internal/singleton_helpers.h>
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE

ListSingletonContainer
::ListSingletonContainer():
  P(){}

ListSingletonContainer
::ListSingletonContainer(const ParticlesTemp &ps):
  P(P::get_model(ps.begin(), ps.end()), "ListSingletonContainer%1%")
{
  IMP_USAGE_CHECK(is_ok(ps.begin(), ps.end()),
                  "All particles must belong to the same model.");
  set_particles(ps);
}

ListSingletonContainer
::ListSingletonContainer(const ParticlesTemp &ps,
                         std::string name):
  P(P::get_model(ps.begin(), ps.end()), name)
{
  IMP_USAGE_CHECK(is_ok(ps.begin(), ps.end()),
                  "All particles must belong to the same model.");
  set_particles(ps);
}

ListSingletonContainer
::ListSingletonContainer(Model *m, std::string name):
  P(m, name){
}

ListSingletonContainer
::ListSingletonContainer(Model *m, const char *name):
  P(m, name){
}

void ListSingletonContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out <<  get_number_of_particles()
      << " Singletons" << std::endl;
}

IMPCONTAINER_END_NAMESPACE
