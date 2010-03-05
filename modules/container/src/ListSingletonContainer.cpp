/**
 *  \file ListSingletonContainer.cpp   \brief A list of Particles.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/ListSingletonContainer.h"
#include "IMP/SingletonModifier.h"
#include "IMP/SingletonScore.h"
#include <IMP/core/internal/singleton_helpers.h>
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE

ListSingletonContainer
::ListSingletonContainer(bool):
  P(true){}

ListSingletonContainer
::ListSingletonContainer(const Particles &ps,
                         std::string name):
  P(name)
{
  if (ps.empty()) return;
  for (unsigned int i=0; i< ps.size(); ++i) {
    IMP_USAGE_CHECK(IMP::internal::is_valid(ps[i]),
                    "Passed Particle cannot be NULL (or None)");
    IMP_USAGE_CHECK(IMP::internal::get_model(ps[i])
                    == IMP::internal::get_model(ps[0]),
                    "All particles in container must have the same model. "
                    << "Particle " << IMP::internal::get_name(ps[i])
                    << " does not.");
  }
  set_particles(ps);
}

ListSingletonContainer
::ListSingletonContainer(std::string name):
  P(name){
}

ListSingletonContainer
::ListSingletonContainer(const char *name):
  P(name){
}

void ListSingletonContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out <<  get_number_of_particles()
      << " particles" << std::endl;
}

IMPCONTAINER_END_NAMESPACE
