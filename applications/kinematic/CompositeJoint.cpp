/**
 *  \file CompositeJoint.cpp
 *  \brief a joint composed of several joints, applied on the same
 *         pair of rigid bodies
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */


#include "KinematicForest.h"
#include "KinematicNode.h"
#include "CompositeJoint.h"
#include <IMP/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/algebra/Transformation3D.h>

IMPCORE_BEGIN_NAMESPACE



/********************** CompositeJoint ***************/

CompositeJoint::CompositeJoint
(RigidBody parent, RigidBody child, Joints joints )
  : TransformationJoint(parent, child)
{
  set_joints( joints );
  update_joint_from_cartesian_witnesses();
}


void
CompositeJoint::update_child_node_reference_frame() const
{
  // TODO: make this efficient - augment all joint transformation
  //       in global coordinates instead of doing it one by one?
  using namespace IMP::algebra;

  for(int i = (int)joints_.size() - 1; i >= 0; i++) {
    joints_[i]->update_child_node_reference_frame();
  }
}


void CompositeJoint::set_joints(Joints joints) {
  for(unsigned int i = 0; i < joints_.size(); i++){
    joints_[i]->set_owner_kf( nullptr );
  }
  // add new joints
  for(unsigned int i = 0; i < joints.size(); i++){
    add_downstream_joint( joints[i] );
  }
}


void CompositeJoint::update_joint_from_cartesian_witnesses() {
  for(unsigned int i = 0; i < joints_.size(); i++){
    joints_[i]->update_joint_from_cartesian_witnesses();
  }
  TransformationJoint::update_joint_from_cartesian_witnesses();
}


IMPCORE_END_NAMESPACE
