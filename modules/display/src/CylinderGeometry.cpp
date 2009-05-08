/**
 *  \file CylinderGeometry.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/display/CylinderGeometry.h"


IMPDISPLAY_BEGIN_NAMESPACE

CylinderGeometry::CylinderGeometry(const algebra::Cylinder3D &c): c_(c){
}

void CylinderGeometry::show(std::ostream &out) const {
  out << "CylinderGeometry" << std::endl;
}

unsigned int CylinderGeometry::get_dimension() const{
  return 1;
}
unsigned int CylinderGeometry::get_number_of_vertices() const{
  return 2;
}
algebra::Vector3D CylinderGeometry::get_vertex(unsigned int i) const {
  return c_.get_segment().get_point(i);
}


Float CylinderGeometry::get_size() const {
  return c_.get_radius();
}
IMPDISPLAY_END_NAMESPACE
