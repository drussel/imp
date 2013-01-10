/**
 *  \file XYZ.cpp   \brief Simple xyz decorator.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/XYZ.h>
#include <IMP/core/rigid_bodies.h>
#include <cmath>

IMPCORE_BEGIN_NAMESPACE

void XYZ::show(std::ostream &out) const
{
  out << "(" <<algebra::commas_io(get_coordinates())<<")";

}

const FloatKeys& XYZ::get_xyz_keys() {
  static FloatKeys fks(IMP::internal::xyzr_keys,
                       IMP::internal::xyzr_keys+3);
  return fks;
}

void transform(XYZ a, const algebra::Transformation3D &tr) {
  IMP_USAGE_CHECK(!RigidBody::particle_is_instance(a),
                  "Python is calling the wrong function");
  a.set_coordinates(tr.get_transformed(a.get_coordinates()));
}

IMPCORE_END_NAMESPACE
