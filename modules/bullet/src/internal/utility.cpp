/**
 *  \file ResolveCollisionsOptimizer.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/bullet/internal/utility.h"

#include <IMP/core/rigid_bodies.h>
#include <IMP/core/XYZR.h>
#include <IMP/atom/Mass.h>
#include <IMP/core/PairRestraint.h>
#include <IMP/core/DistancePairScore.h>
#include <IMP/core/SphereDistancePairScore.h>
#include <IMP/container/PairsRestraint.h>
#include <IMP/domino/utility.h>
#include <IMP/scoped.h>
#include <IMP/internal/map.h>
#include <IMP/atom/internal/SpecialCaseRestraints.h>

#include <btBulletDynamicsCommon.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/bind.hpp>


IMPBULLET_BEGIN_INTERNAL_NAMESPACE



std::pair<std::vector<btScalar>, Ints>
get_as_bt(const algebra::Vector3Ds &vertices,
               const Ints &tris) {
  std::pair<std::vector<btScalar>, Ints> ret=
    std::make_pair(std::vector<btScalar>(vertices.size()*3),
                                      Ints(tris.size()*3));
  for (unsigned int i=0; i< vertices.size(); ++i) {
    ret.first[3*i]=vertices[i][0];
    ret.first[3*i+1]=vertices[i][1];
    ret.first[3*i+2]=vertices[i][2];
  }
  int oi=0;
  for (unsigned int i=0; i< tris.size(); ++i) {
    if (tris[i] != -1) {
      ret.second[oi++]=tris[i];
    }
  }
  return ret;
}

ParticlesTemp get_particles(const ParticlesTemp &in) {
  ParticlesTemp ret;
  for (unsigned int i=0; i< in.size(); ++i) {
    if (core::XYZR::particle_is_instance(in[i])) {
      ret.push_back(in[i]);
    }
  }
  IMP_LOG(TERSE, "Found " << ret.size() << " particles." << std::endl);
  return ret;
}


btRigidBody *create_rigid_body(btCollisionShape *shape,
                               const algebra::Transformation3D &center,
                               double mass,
                               btDiscreteDynamicsWorld* world,
                               Memory &memory) {
  // compute coordinates
  // create rigid body
  btDefaultMotionState* fallMotionState
    = new btDefaultMotionState(tr(center));
  memory.motion_states.push_back(fallMotionState);
  btVector3 fallInertia(0,0,0);
  //if (mass > 0) {
  //shape->calculateLocalInertia(mass,fallInertia);
  //} else if (mass < 0) {
  fallInertia= btVector3(.01, .01, .01);
  //}
  btRigidBody::btRigidBodyConstructionInfo
    fallRigidBodyCI((mass<0? .0001:mass),fallMotionState,shape,fallInertia);
  btRigidBody* fallRigidBody= new btRigidBody(fallRigidBodyCI);
  fallRigidBody->setDamping(.9, .9);
  {
    btTransform trans;
    fallRigidBody->getMotionState()->getWorldTransform(trans);
    //std::cout << center << " vs " << internal::tr(trans) << std::endl;
  }
  world->addRigidBody(fallRigidBody);
  memory.rigid_bodies.push_back(fallRigidBody);
  return fallRigidBody;
}


void copy_back_coordinates(const RigidBodyMap &map) {
  for (internal::RigidBodyMap::const_iterator
         it = map.begin(); it != map.end(); ++it) {
    btTransform trans;
    Particle *p= it->first;
    it->second->getMotionState()->getWorldTransform(trans);
    if (core::RigidBody::particle_is_instance(p)) {
      //std::cout << "was " << core::RigidBody(it->first)
      //  .get_transformation() << " is " << internal::tr(trans) << std::endl;
      core::RigidBody(it->first)
        .set_reference_frame(algebra::ReferenceFrame3D(internal::tr(trans)));
    } else {
      //std::cout << internal::tr(trans.getOrigin()) << std::endl;
      core::XYZ(it->first).set_coordinates(internal::tr(trans.getOrigin()));
    }
  }
}

IMPBULLET_END_INTERNAL_NAMESPACE
