/**
 *  \file smoothing_functions.cpp    Classes to smooth nonbonded interactions
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#include <IMP/atom/smoothing_functions.h>

IMPATOM_BEGIN_NAMESPACE

SmoothingFunction::SmoothingFunction() {}

void ForceSwitch::do_show(std::ostream &out) const {
  out << "switching between " << min_distance_
      << " and " << max_distance_;
}

IMPATOM_END_NAMESPACE
