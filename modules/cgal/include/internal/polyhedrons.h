/**
 *  \file cgal/internal/polyhedrons.h
 *  \brief manipulation of text, and Interconversion between text and numbers
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
*/

#ifndef IMPCGAL_INTERNAL_POLYHEDRONS_H
#define IMPCGAL_INTERNAL_POLYHEDRONS_H

#include "../cgal_config.h"
#include <IMP/base_types.h>
#include <IMP/algebra/VectorD.h>
#include <IMP/algebra/SphereD.h>
#include <IMP/algebra/Plane3D.h>
#include <IMP/algebra/Grid3D.h>
#include <vector>


IMPCGAL_BEGIN_INTERNAL_NAMESPACE
/** compute the polygon as the subtraction of the intersection of
    the various planes
 */
IMPCGALEXPORT
std::vector<std::vector<algebra::VectorD<3> > >
get_polyhedron_facets(const algebra::BoundingBoxD<3> &bb,
                      const std::vector< algebra::Plane3D > &outer,
                      const std::vector< algebra::Plane3D > &hole);


IMPCGALEXPORT
std::vector<std::vector<algebra::VectorD<3> > >
get_polyhedron_facets(const algebra::BoundingBoxD<3> &bb,
                      const std::vector< algebra::Plane3D > &outer);

IMPCGALEXPORT
std::pair<std::vector<algebra::VectorD<3> >,Ints >
get_polyhedron_indexed_facets(const algebra::BoundingBoxD<3> &bb,
                              const std::vector< algebra::Plane3D > &outer,
                              const std::vector< algebra::Plane3D > &hole);


IMPCGALEXPORT
std::pair<std::vector<algebra::VectorD<3> >,Ints >
get_skin_surface(const std::vector<algebra::SphereD<3> > &ss);

IMPCGALEXPORT
std::pair<std::vector<algebra::VectorD<3> >,Ints >
get_iso_surface(const algebra::grids::GridD<3,
                    algebra::grids::DenseGridStorageD<3,
        double> > &grid, double iso_level);

IMPCGALEXPORT
std::pair<std::vector<algebra::VectorD<3> >,Ints >
get_iso_surface(const algebra::grids::GridD<3,
                   algebra::grids::DenseGridStorageD<3,
        float> > &grid, double iso_level);


IMPCGAL_END_INTERNAL_NAMESPACE

#endif  /* IMPCGAL_INTERNAL_POLYHEDRONS_H */
