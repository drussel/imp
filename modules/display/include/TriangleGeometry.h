/**
 *  \file TriangleGeometry.h
 *  \brief Geometry of a triangle
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPDISPLAY_TRIANGLE_GEOMETRY_H
#define IMPDISPLAY_TRIANGLE_GEOMETRY_H

#include "config.h"

#include "geometry.h"
#include "macros.h"

IMPDISPLAY_BEGIN_NAMESPACE

//! Store a triangle
/** Represented by three vertices. The triangles are oriented counter
    clockwise.
 */
class IMPDISPLAYEXPORT TriangleGeometry: public Geometry
{
  algebra::Vector3D v_[3];
public:
  TriangleGeometry(const algebra::Vector3D &a,
                   const algebra::Vector3D &b,
                   const algebra::Vector3D &c);

  IMP_GEOMETRY(TriangleGeometry, get_module_version_info())
};


IMPDISPLAY_END_NAMESPACE

#endif  /* IMPDISPLAY_TRIANGLE_GEOMETRY_H */
