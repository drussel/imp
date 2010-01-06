/**
 *  \file RigidBodyMover.cpp
 *  \brief A mover that transforms a rigid body
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
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
  last_transformation_= d_.get_transformation();
  algebra::Vector3D translation
    = algebra::random_vector_in_sphere(d_.get_coordinates(),
                                       max_translation_);
  algebra::Vector3D axis =
      algebra::random_vector_on_sphere(algebra::Vector3D(0.0,0.0,0.0),1.);
  ::boost::uniform_real<> rand(-max_angle_,max_angle_);
  Float angle =rand(random_number_generator);
  algebra::Rotation3D r= algebra::rotation_in_radians_about_axis(axis, angle);
  algebra::Rotation3D rc= compose(r, d_.get_transformation().get_rotation());
  algebra::Transformation3D t(rc, translation);
  IMP_LOG(VERBOSE,"RigidBodyMover:: propose move : " << t << std::endl);
  d_.set_transformation(t);
}


void RigidBodyMover::accept_move()
{
}

//! Roll back any changes made to the Particles
void RigidBodyMover::reject_move() {
  d_.set_transformation(last_transformation_);
  last_transformation_= algebra::Transformation3D();
}

IMPCORE_END_NAMESPACE
