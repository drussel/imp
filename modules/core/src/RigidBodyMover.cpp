/**
 *  \file RigidBodyMover.cpp
 *  \brief A mover that transforms a rigid body
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/core/RigidBodyMover.h>
#include <IMP/core/XYZ.h>
#include <IMP/algebra/vector_generators.h>
IMPCORE_BEGIN_NAMESPACE

RigidBodyMover::RigidBodyMover(RigidBody d,
                               Float max_translation, Float max_angle) {
  IMP_LOG(VERBOSE,"start RigidBodyMover constructor");
  max_translation_=max_translation;
  max_angle_ =max_angle;
  d_= d;
  IMP_LOG(VERBOSE,"finish mover construction" << std::endl);
}

void RigidBodyMover::propose_move(Float f) {
  IMP_LOG(VERBOSE,"RigidBodyMover:: propose move f is  : " << f <<std::endl);
  {
    ::boost::uniform_real<> rand(0,1);
    double fc =rand(random_number_generator);
    if (fc > f) return;
  }
  last_transformation_= d_.get_reference_frame().get_transformation_to();
  algebra::VectorD<3> translation
    = algebra::get_random_vector_in(algebra::Sphere3D(d_.get_coordinates(),
                                                      max_translation_));
  algebra::VectorD<3> axis =
    algebra::get_random_vector_on(algebra::Sphere3D(algebra::VectorD<3>(0.0,
                                                                        0.0,
                                                                        0.0),
                                                    1.));
  ::boost::uniform_real<> rand(-max_angle_,max_angle_);
  Float angle =rand(random_number_generator);
  algebra::Rotation3D r
    = algebra::get_rotation_about_axis(axis, angle);
  algebra::Rotation3D rc
    = r*d_.get_reference_frame().get_transformation_to().get_rotation();
  algebra::Transformation3D t(rc, translation);
  IMP_LOG(VERBOSE,"RigidBodyMover:: propose move : " << t << std::endl);
  d_.set_reference_frame(algebra::ReferenceFrame3D(t));
}



void RigidBodyMover::reset_move() {
  d_.set_reference_frame(algebra::ReferenceFrame3D(last_transformation_));
  last_transformation_= algebra::Transformation3D();
}


void RigidBodyMover::do_show(std::ostream &out) const {
  out << "max translation: " << max_translation_ << "\n";
  out << "max angle: " << max_angle_ << "\n";
}
IMPCORE_END_NAMESPACE
