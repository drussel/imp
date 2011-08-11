/**
 *  \file cgal/internal/polygons.h
 *  \brief manipulation of text, and Interconversion between text and numbers
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
*/

#ifndef IMPCGAL_INTERNAL_POLYGONS_H
#define IMPCGAL_INTERNAL_POLYGONS_H

#include "../cgal_config.h"
#include <IMP/base_types.h>
#include <IMP/algebra/VectorD.h>
#include <IMP/algebra/BoundingBoxD.h>
#include <vector>


IMPCGAL_BEGIN_INTERNAL_NAMESPACE

IMPCGALEXPORT std::vector<algebra::Vector3D >
get_intersection(const algebra::Vector3D &normal,
                 double d,
                 const algebra::BoundingBoxD<3> &bb);

IMPCGALEXPORT
Ints
get_convex_polygons(const Ints &indexes,
                    const std::vector<algebra::Vector3D > &vertices);


IMPCGAL_END_INTERNAL_NAMESPACE

#endif  /* IMPCGAL_INTERNAL_POLYGONS_H */
