/**
 *  \file joints.cpp
 *  \brief functionality for defining kinematic joints between rigid bodies
 *         as part of a kinematic tree
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */


#include "KinematicForest.h"
#include "KinematicNode.h"
#include <IMP/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/algebra/Transformation3D.h>

IMPCORE_BEGIN_NAMESPACE

/********************** Joint ***************/

Joint::Joint
(RigidBody parent, RigidBody child) :
  Object("IMP_CORE_JOINT"),
  parent_(parent), child_(child), owner_kf_(nullptr)
{
}


const IMP::algebra::Transformation3D&
Joint::get_transformation_child_to_parent() const
{
  if( get_owner_kf() ) {
    get_owner_kf()->update_all_internal_coordinates();
  }
  return tr_child_to_parent_;
}


void
Joint::update_child_node_reference_frame() const
{
  // TODO: make this efficient - indexing? lazy? update flag?
  using namespace IMP::algebra;

  ReferenceFrame3D parent_rf = parent_.get_reference_frame();
  const Transformation3D& tr_parent_to_global =
    parent_rf.get_transformation_to();
  const Transformation3D& tr_child_to_parent =
    get_transformation_child_to_parent();
  Transformation3D tr_child_to_global
    (tr_parent_to_global * tr_child_to_parent);

  RigidBody child_rb = RigidBody(child_.get_particle());
  child_rb.set_reference_frame
    ( ReferenceFrame3D( tr_child_to_global ) );
}

void
Joint::do_show(std::ostream & os) const
{
  os << "(Joint " << child_ << " to " << parent_ << ")";
}

/********************** Transformation Joint ***************/

TransformationJoint::TransformationJoint
(RigidBody parent, RigidBody child)
  :  Joint(parent, child)
{
  update_joint_from_cartesian_witnesses();
}


// Sets the transfromation from parent to child
void
TransformationJoint::set_transformation_child_to_parent
(IMP::algebra::Transformation3D transformation)
{
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates( );
  }
  Joint::set_transformation_child_to_parent_no_checks( transformation );
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}

void
TransformationJoint::update_joint_from_cartesian_witnesses()
{
  // TODO: make this efficient - indexing? lazy? update flag?
  using namespace IMP::algebra;

  ReferenceFrame3D parent_rf = get_parent_node().get_reference_frame();
  ReferenceFrame3D child_rf = get_child_node().get_reference_frame();
  const Transformation3D& tr_global_to_parent =
    parent_rf.get_transformation_from();
  const Transformation3D& tr_child_to_global =
    child_rf.get_transformation_to();
  set_transformation_child_to_parent_no_checks
    (tr_global_to_parent * tr_child_to_global);
}


/********************** Revolute Joint ***************/

RevoluteJoint::RevoluteJoint
( RigidBody parent, RigidBody child,
  XYZ a, XYZ b,
  double initial_angle )
  : Joint(parent, child)
{
  // TODO: who are the witnesses here exactly?
  set_revolute_joint_params
    ( a.get_coordinates(),
      b.get_coordinates() - a.get_coordinates(),
      initial_angle);

  //      ss=new RevoluteJointScoreState(p, ...); // TODO: implement that?
  //p->get_model()->add_score_state(ss); // TODO: implement that?
}

// definition of dummy pure virtual, just for SWIG
RevoluteJoint::~RevoluteJoint()
{}



double
RevoluteJoint::get_angle() const
{
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates( );
  }
  return angle_;
}


//   sets the angle of the revolute joint and update the joint
//   transformation accordingly
void
RevoluteJoint::set_angle(double angle) {
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates();
  }
  using namespace IMP::algebra;;
  angle_ = angle;
  // update child to parent transformation:
  Transformation3D R = get_rotation_about_joint();
  const Transformation3D tr_child_to_global =
      R * tr_child_to_global_without_rotation_ ;
  ReferenceFrame3D parent_rf =
    get_parent_node().get_reference_frame();
  const Transformation3D& tr_global_to_parent =
    parent_rf.get_transformation_from();
  Joint::set_transformation_child_to_parent_no_checks
    ( tr_global_to_parent * tr_child_to_global );
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}

/********************** DihedralAngleRevoluteJoint ***************/

DihedralAngleRevoluteJoint
::DihedralAngleRevoluteJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b, XYZ c, XYZ d) :
  RevoluteJoint(parent, child,
                b, c,
                internal::dihedral(a,b,c,d,nullptr,nullptr,nullptr,nullptr)
                ),
  a_(a), b_(b), c_(c), d_(d) // TODO: are b_ and c_ redundant?
{
  // TODO: scorestate for udpating the model? see revolute joint
  update_joint_from_cartesian_witnesses();
}


void
DihedralAngleRevoluteJoint::update_joint_from_cartesian_witnesses()
{
  double angle = internal::dihedral
    ( a_, b_, c_, d_,
      nullptr, // derivatives - TODO: support?
      nullptr,
      nullptr,
      nullptr );
  RevoluteJoint::set_revolute_joint_params
    ( b_.get_coordinates(),
      c_.get_coordinates() - b_.get_coordinates(),
      angle );
  // TODO: perhaps the knowledge of normalized joint axis can accelerate
  // the dihedral calculation in get_angle_from_witnesses()?
  // TODO: support derivatives?
}



/********************** Prismatic Joint ***************/

PrismaticJoint::PrismaticJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b) :
  Joint(parent, child), a_(a), b_(b)
{
  update_joint_from_cartesian_witnesses();
}

double
PrismaticJoint::get_length() const
{
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates( );
  }
  return l_;
}

void
PrismaticJoint::set_length
(double l)
{
  IMP_USAGE_CHECK( l > 0 ,
                   "Only a strictly positive length is expected for"
                   << " prismatic joints" );
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates();
  }
  l_ = l;
  IMP::algebra::Vector3D v =
    b_.get_coordinates() - a_.get_coordinates();
  IMP::algebra::Vector3D translation =
    l_ * v.get_unit_vector();
  set_transformation_child_to_parent_no_checks
    ( IMP::algebra::Transformation3D( translation ) );
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
  // note: lazy so we don't update coords of b
}

void
PrismaticJoint::update_joint_from_cartesian_witnesses()
{
  using namespace IMP::algebra;
  const double tiny_double = 1e-12;
  IMP_USAGE_CHECK
    ( get_distance(a_.get_coordinates(), b_.get_coordinates())
      > tiny_double,
      "witnesses of prismatic joint should have different"
      << " coordinates" );

  Vector3D v =
    b_.get_coordinates() - a_.get_coordinates();
  double mag = v.get_magnitude();
  l_ = mag;
  // TODO: should implement set_transformation instead?
  set_transformation_child_to_parent_no_checks
    ( IMP::algebra::Transformation3D( v ) );

  IMP_UNUSED(tiny_double);
}




IMPCORE_END_NAMESPACE
