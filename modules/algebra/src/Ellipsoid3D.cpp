/**
 *  \file  Ellipsoid3D.cpp
 *  \brief simple implementation of ellipsoids in 3D
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */
#include <IMP/base_types.h>
#include <IMP/algebra/Ellipsoid3D.h>
#include <cmath>


IMPALGEBRA_BEGIN_NAMESPACE
Ellipsoid3D::Ellipsoid3D(const Vector3D& center,double r0,
                         double r1, double r2,
                         const Rotation3D &rot):center_(center),
                                               rot_(rot){
  radii_[0]=r0;
  radii_[1]=r1;
  radii_[2]=r2;
}


IMPALGEBRA_END_NAMESPACE
