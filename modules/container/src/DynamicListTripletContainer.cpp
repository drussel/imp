/**
 *  \file ListTripletContainer.cpp   \brief A list of ParticleTripletsTemp.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/DynamicListTripletContainer.h"
#include "IMP/TripletModifier.h"
#include "IMP/TripletScore.h"
#include <IMP/internal/InternalListTripletContainer.h>
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE


DynamicListTripletContainer
::DynamicListTripletContainer(Container *m,  std::string name):
  P(m, name){
}

void DynamicListTripletContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out <<  get_number_of_particle_triplets()
      << " Triplets" << std::endl;
}

IMPCONTAINER_END_NAMESPACE
