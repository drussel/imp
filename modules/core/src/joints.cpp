/**
 *  \file IMP/core/joints.cpp
 *  \brief functionality for defining kinematic joints between rigid bodies
 *         as part of a kinematic tree
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */


#include <IMP/core/KinematicForest.h>
#include <IMP/core/KinematicNode.h>
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
  if(!owner_kf_){
    IMP_THROW("IMP supports only joints that are managed by a"
              << " kinematic forest",
              IMP::ValueException);
  }
}


const IMP::algebra::Transformation3D&
Joint::get_transformation_child_to_parent() const
{
  if( get_owner_kf() ) {
    get_owner_kf()->update_all_internal_coordinates();
  }
  return transformation_child_to_parent_;
}

void
Joint::update_child_node_reference_frame() const
{
  // TODO: make this efficient - indexing? lazy? update flag?
  using namespace IMP::algebra;

  const Transformation3D& tr_parent_to_global =
    parent_.get_reference_frame().get_transformation_to();
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
  transformation_child_to_parent_ = transformation;
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}

void
TransformationJoint::update_joint_from_cartesian_witnesses()
{
  // TODO: IMPLEMENT
  IMP_NOT_IMPLEMENTED;
}


/********************** Revolute Joint ***************/

RevoluteJoint::RevoluteJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b, double angle)
  : Joint(parent, child),
    angle_(angle)
{
  // TODO: who are the witnesses here exactly?
  set_joint( b.get_coordinates() - a.get_coordinates(),
             a.get_coordinates());
  // TODO: is angle useful for anything
  //      ss=new RevoluteJointScoreState(p, ...); // TODO: implement that?
  //p->get_model()->add_score_state(ss); // TODO: implement that?
}

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
  angle_ = angle;
  update_transformation_from_angle();
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}

/********************** DihedralAngleRevoluteJoint ***************/

DihedralAngleRevoluteJoint
::DihedralAngleRevoluteJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b, XYZ c, XYZ d) :
  RevoluteJoint(parent, child, b, c),
  a_(a), b_(b), c_(c), d_(d) // TODO: are b_ and c_ redundant?
{
  // TODO: scorestate for udpating the model? see revolute joint
  update_joint_from_cartesian_witnesses();
}


void
DihedralAngleRevoluteJoint::update_joint_from_cartesian_witnesses()
{
  set_joint( c_.get_coordinates() - b_.get_coordinates(),
             b_.get_coordinates());
  // TODO: perhaps the knowledge of normalized joint axis can accelerate
  // the dihedral calculation in next line?
  set_angle( internal::dihedral
             (a_, b_, c_, d_,
              nullptr, // derivatives - TODO: support?
              nullptr,
              nullptr,
              nullptr)
             );
}

/********************** Prismatic Joint ***************/

PrismaticJoint::PrismaticJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b) :
  Joint(parent, child), a_(a), b_(b)
{
  double tiny_double = 1e-12;
  if( (a.get_coordinates() - b_.get_coordinates()).get_magnitude()
      < tiny_double ) {
    IMP_THROW( "cannot create a prismatic joint with witnesses of"
               << " identical coordinates " << a << " and " << b,
               IMP::ValueException );
  }
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
  transformation_child_to_parent_ =
    IMP::algebra::Transformation3D( translation );
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
      < tiny_double,
      "witnesses of prismatic joint should have different"
      << " coordinates" );

  Vector3D v =
    b_.get_coordinates() - a_.get_coordinates();
  double mag = v.get_magnitude();
  l_ = mag;
  // TODO: should implement set_transformation instead?
  transformation_child_to_parent_ =
    IMP::algebra::Transformation3D( v ) ;

  IMP_UNUSED(tiny_double);
}




IMPCORE_END_NAMESPACE
