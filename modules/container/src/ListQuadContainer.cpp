/**
 *  \file ListQuadContainer.cpp   \brief A list of ParticleQuads.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/ListQuadContainer.h"
#include "IMP/QuadModifier.h"
#include "IMP/QuadScore.h"
#include <IMP/core/internal/quad_helpers.h>
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE

ListQuadContainer
::ListQuadContainer():
  P(){}

ListQuadContainer
::ListQuadContainer(const ParticleQuads &ps,
                         std::string name):
  P(P::get_model(ps.begin(), ps.end()), name)
{
  IMP_USAGE_CHECK(is_ok(ps.begin(), ps.end()),
                  "All particles must belong to the same model.");
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
      << " particle_quads" << std::endl;
}

IMPCONTAINER_END_NAMESPACE
