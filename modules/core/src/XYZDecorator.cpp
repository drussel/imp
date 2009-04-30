/**
 *  \file XYZDecorator.cpp   \brief Simple xyz decorator.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/core/XYZDecorator.h>

#include <cmath>

IMPCORE_BEGIN_NAMESPACE

void XYZDecorator::show(std::ostream &out, std::string prefix) const
{
  out << prefix << "(" <<algebra::commas_io(get_coordinates())<<")";

}
const FloatKeys&  XYZDecorator::get_xyz_keys() {
  static FloatKey fka[]={get_coordinate_key(0),
                         get_coordinate_key(1),
                         get_coordinate_key(2)};
  static FloatKeys fks(fka, fka+3);
  return fks;
}


Float distance(XYZDecorator a, XYZDecorator b)
{
  return algebra::distance(a.get_coordinates(),b.get_coordinates());
}

IMPCORE_END_NAMESPACE
