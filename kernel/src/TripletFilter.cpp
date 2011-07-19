/**
 *  \file TripletFilter.cpp   \brief Filter for triplet.
 *
 *  This file is generated by a script (core/tools/make-filter).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/TripletFilter.h"
#include "IMP/internal/utility.h"
#include "IMP/TripletModifier.h"
#include "IMP/internal/container_helpers.h"
#include <algorithm>

IMP_BEGIN_NAMESPACE

TripletFilter::TripletFilter(std::string name): Object(name) {
}

void TripletFilter
::filter_in_place(ParticleTripletsTemp &ps) const {
  ps.erase(std::remove_if(ps.begin(), ps.end(),
         internal::GetContains<TripletFilter>(this)),
           ps.end());

}

IMP_END_NAMESPACE
