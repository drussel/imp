/**
 *  \file ListQuadContainer.cpp   \brief A list of ParticleQuadsTemp.
 *
 *  WARNING This file was generated from ListNAMEContainer.cc
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/ListQuadContainer.h"
#include "IMP/QuadModifier.h"
#include "IMP/QuadScore.h"
#include <IMP/internal/InternalListQuadContainer.h>
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE


ListQuadContainer
::ListQuadContainer(const ParticleQuadsTemp &ps):
  P(IMP::internal::get_model(ps[0]),
    "ListSingletonContainer%1%")
{
  set_particle_quads(ps);
}

ListQuadContainer
::ListQuadContainer(const ParticleQuadsTemp &ps,
                         std::string name):
  P(IMP::internal::get_model(ps[0]), name)
{
  set_particle_quads(ps);
}

ListQuadContainer
::ListQuadContainer(Model *m, std::string name):
  P(m, name){
}

ListQuadContainer
::ListQuadContainer(Model *m, const char *name):
  P(m, name){
}

void ListQuadContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out <<  get_number_of_particle_quads()
      << " Quads" << std::endl;
}

IMPCONTAINER_END_NAMESPACE
