/**
 *  \file core/utility.h    \brief Various important functionality
 *                                       for implementing decorators.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_UTILITY_H
#define IMPCORE_UTILITY_H

#include "core_macros.h"
#include "core_config.h"
#include "XYZR.h"
#include <IMP/base_types.h>
#include <IMP/algebra/Segment3D.h>
#include <IMP/algebra/Transformation3D.h>
#include <IMP/Model.h>
#include <IMP/Particle.h>

IMPCORE_BEGIN_NAMESPACE
//! Get the centroid
/** Compute the centroid (mean) of a set of particles.
 */
IMPCOREEXPORT algebra::VectorD<3> get_centroid(const XYZs &ps);

//! Get the bounding box
/** Compute the boudning box of a set of particles
 */
IMPCOREEXPORT algebra::BoundingBoxD<3> get_bounding_box(const XYZRs &ps);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_UTILITY_H */
