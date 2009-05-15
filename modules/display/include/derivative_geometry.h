/**
 *  \file derivative_geometry.h
 *  \brief Display the derivatives of various things
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPDISPLAY_DERIVATIVE_GEOMETRY_H
#define IMPDISPLAY_DERIVATIVE_GEOMETRY_H

#include "config.h"
#include "macros.h"

#include "internal/version_info.h"
#include <IMP/PairContainer.h>
#include <IMP/core/XYZR.h>
#include <IMP/core/rigid_bodies.h>
#include <IMP/display/geometry.h>

IMPDISPLAY_BEGIN_NAMESPACE

//! Display the derivatives of an XYZ particle
/** This class draws the derivative of an XYZ particle
    as cylinders starting at the particle center. The radius
    of the cylinder can be specified. The name is the
    Particle::get_name() name.
 */
class IMPDISPLAYEXPORT XYZDerivativeGeometry: public Geometry
{
  core::XYZ d_;
  Float radius_;
public:
  XYZDerivativeGeometry(core::XYZ d, Float radius=0);

  std::string get_name() const {
    return d_.get_particle()->get_name();
  }
  IMP_GEOMETRY(XYZDerivativeGeometry, internal::version_info)
};


//! Display the derivatives of an RigidBody particle
/** This class displays the derivatives of a rigid body. Currently,
    it draws a cylinder like the XYZDerivativeGeometry for each of
    the members, splitting the motion into rotational and translational
    parts. The name is the Particle::get_name() name.
 */
class IMPDISPLAYEXPORT RigidBodyDerivativeGeometry:
  public CompoundGeometry
{
  Color xyzcolor_, qcolor_, ccolor_;
  core::RigidBody d_;
public:
  RigidBodyDerivativeGeometry(core::RigidBody d);

  //! Set the color used to display the translational part
  void set_translational_color(Color c) {
    xyzcolor_=c;
  }

  //! Set the color used to display the rotational part
  void set_rotational_color(Color c) {
    qcolor_=c;
  }

  IMP_COMPOUND_GEOMETRY(RigidBodyDerivativeGeometry, internal::version_info);
};


IMPDISPLAY_END_NAMESPACE

#endif  /* IMPDISPLAY_DERIVATIVE_GEOMETRY_H */
