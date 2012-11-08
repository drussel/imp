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
#include "Joint.h"
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
  update_joint_from_cartesian_witnesses();
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

  std::cout << "Joint::update_child_node_reference_frame()" << std::endl;
  ReferenceFrame3D parent_rf = parent_.get_reference_frame();
  const Transformation3D& tr_parent_to_global =
    parent_rf.get_transformation_to();
  const Transformation3D& tr_child_to_parent =
    get_transformation_child_to_parent_no_checks();
  Transformation3D tr_child_to_global
    (tr_parent_to_global * tr_child_to_parent);

  // TODO: should we add a set_reference_frame_lazy() variant? this
  // has effects that need to be thought through
  RigidBody child_rb = RigidBody(child_.get_particle());
  child_rb.set_reference_frame
    ( ReferenceFrame3D( tr_child_to_global ) );
}

void
Joint::update_joint_from_cartesian_witnesses()
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


void
Joint::do_show(std::ostream & os) const
{
  os << "(Joint " << child_ << " to " << parent_ << ")";
}

IMPCORE_END_NAMESPACE
