/**
 *  \file ResolveCollisionsOptimizer.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-8 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/bullet/ResolveCollisionsOptimizer.h"

#include <IMP/core/rigid_bodies.h>
#include <IMP/core/XYZR.h>
#include <IMP/atom/Mass.h>
#include <IMP/core/PairRestraint.h>
#include <IMP/core/DistancePairScore.h>
#include <IMP/container/PairsRestraint.h>
#include <IMP/scoped.h>
#include <IMP/internal/map.h>

#include <btBulletDynamicsCommon.h>
#include <boost/ptr_container/ptr_vector.hpp>
IMPBULLET_BEGIN_NAMESPACE

#define IMP_BNEW(Name, name, args) std::auto_ptr<Name> name(new Name args);
btVector3 tr(const algebra::VectorD<3> &v) {
  return btVector3(v[0], v[1], v[2]);
}
const algebra::VectorD<3> tr(const btVector3 &v) {
  return algebra::VectorD<3>(v[0], v[1], v[2]);
}

/**
   Target implementation:
   decompose restraints
   find all restraints with a harmonic distance pair score
   and implement those
 */

ResolveCollisionsOptimizer
::ResolveCollisionsOptimizer(const RestraintSetsTemp &rs,
                             const ParticlesTemp &ps):
  Optimizer(rs[0]->get_model(), "ResolveCollisionsOptimizer %1%"),
  rs_(rs), ps_(ps){
}

ResolveCollisionsOptimizer::ResolveCollisionsOptimizer(Model *m):
  Optimizer(m, "ResolveCollisionsOptimizer %1%"),
  rs_(1, m->get_root_restraint_set()), ps_(m->particles_begin(),
                                           m->particles_end()){
}
/*
void ResolveCollisionsOptimizer::add_obstacle
(const algebra::Vector3Ds &vertices,
const std::vector<boost::tuple<int,int,int> > &tris) {
    obstacles_.push_back(std::make_pair(tris, vertices));
    }*/
/**
   bonds use btGeneric6DofSpringConstraint set the transform of one
   to the center of the other and constrain x,y,z to be 0
   or just use a point-to-point constraint on the center of one
 */

namespace {
  void handle_restraints(RestraintSet *rs,double weight,
                         btDiscreteDynamicsWorld *world,
                   const IMP::internal::Map<Particle*, btRigidBody *> &map,
                    boost::ptr_vector< btGeneric6DofSpringConstraint> &springs,
                       boost::ptr_vector< ScopedRemoveRestraint> &restraints) {
    Restraints rss(rs->restraints_begin(), rs->restraints_end());
    for (unsigned int i=0; i < rss.size(); ++i) {
      Restraint *r= rss[i];
      if (dynamic_cast<core::PairRestraint*>(r)) {
        core::PairRestraint*pr= dynamic_cast<core::PairRestraint*>(r);
        PairScore *ps= pr->get_pair_score();
        core::HarmonicDistancePairScore *hdps
          = dynamic_cast<core::HarmonicDistancePairScore*>(ps);
        if (hdps) {
          IMP_LOG(TERSE, "Handling restraint " << pr->get_name() << std::endl);
          double x0= hdps->get_rest_length();
          Particle *p0= pr->get_particle_pair()[0];
          Particle *p1= pr->get_particle_pair()[1];
          btRigidBody *r0= map.find(p0)->second;
          btRigidBody *r1= map.find(p1)->second;
          // assume center is coordinates of particle
          btTransform it; it.setIdentity();
          btTransform it1; it1.setIdentity();
          it1.setOrigin(btVector3(x0, 0,0));
          springs.push_back(new btGeneric6DofSpringConstraint(*r0, *r1,
                                                                 it, it1,
                                                                 true));
          for (unsigned int i=0; i< 3; ++i) {
            springs.back().enableSpring(i, true);
            springs.back().setStiffness(i, hdps->get_k());
          }
          world->addConstraint(&springs.back());
          restraints.push_back(new ScopedRemoveRestraint(pr,rs));
        }
      } else if (dynamic_cast<RestraintSet*>(r)){
        RestraintSet *rsc=
          dynamic_cast<RestraintSet*>(r);
        handle_restraints(rsc, weight*rsc->get_weight(),
                          world, map, springs, restraints);
      } else if (dynamic_cast<container::PairsRestraint*>(r)) {
        container::PairsRestraint*pr
          = dynamic_cast<container::PairsRestraint*>(r);
        PairScore *ps= pr->get_pair_score();
        core::HarmonicDistancePairScore *hdps
          = dynamic_cast<core::HarmonicDistancePairScore*>(ps);
        if (hdps) {
          IMP_LOG(TERSE, "Handling restraint " << pr->get_name() << std::endl);
          double x0= hdps->get_rest_length();
          IMP_FOREACH_PAIR(pr->get_pair_container(), {
              Particle *p0= _1[0];
              Particle *p1= _1[1];
              btRigidBody *r0= map.find(p0)->second;
              btRigidBody *r1= map.find(p1)->second;
              // assume center is coordinates of particle
              btTransform it; it.setIdentity();
              btTransform it1; it1.setIdentity();
              it1.setOrigin(btVector3(x0, 0,0));
              springs.push_back(new btGeneric6DofSpringConstraint(*r0, *r1,
                                                                  it, it1,
                                                                  true));
              for (unsigned int i=0; i< 3; ++i) {
                springs.back().enableSpring(i, true);
                springs.back().setStiffness(i, hdps->get_k());
              }
              world->addConstraint(&springs.back());
            });
          restraints.push_back(new ScopedRemoveRestraint(pr,rs));
        }
      }
    }
  }
}

/**
 dynamicsWorld->setGravity(btVector3(0,-10,0));

 btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  shapes.push_back(sp(groundShape));
  btCollisionShape* fallShape = new btSphereShape(1);
  shapes.push_back(sp(fallShape));
  IMP_BNEW(btDefaultMotionState, groundMotionState,
           (btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0))));
  btRigidBody::btRigidBodyConstructionInfo
    groundRigidBodyCI(0,groundMotionState.get(),groundShape,btVector3(0,0,0));
  IMP_BNEW(btRigidBody, groundRigidBody, (groundRigidBodyCI));
  dynamicsWorld->addRigidBody(groundRigidBody.get());

  IMP_BNEW(btDefaultMotionState, fallMotionState,
           (btTransform(btQuaternion(0,0,0,1),btVector3(0,50,0))));
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass,fallInertia);
  btRigidBody::btRigidBodyConstructionInfo
    fallRigidBodyCI(mass,fallMotionState.get(),fallShape,fallInertia);
  IMP_BNEW(btRigidBody, fallRigidBody, (fallRigidBodyCI));
  dynamicsWorld->addRigidBody(fallRigidBody.get());
 */

double ResolveCollisionsOptimizer::optimize(unsigned int iter) {
  // http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
  IMP_BNEW(btDbvtBroadphase, broadphase, ());
  IMP_BNEW(btDefaultCollisionConfiguration, collisionConfiguration, ());
  IMP_BNEW(btCollisionDispatcher, dispatcher,
           (collisionConfiguration.get()));
  IMP_BNEW(btSequentialImpulseConstraintSolver, solver, ());
  IMP_BNEW(btDiscreteDynamicsWorld,
           dynamicsWorld, (dispatcher.get(),broadphase.get(),
                           solver.get(),collisionConfiguration.get()));

  boost::ptr_vector<btCollisionShape > shapes;
  boost::ptr_vector<btMotionState > motion_states;
  boost::ptr_vector<btRigidBody > rigid_bodies;
  IMP::internal::Map<Particle*, btRigidBody *> map;
  boost::ptr_vector<btGeneric6DofSpringConstraint > springs;
  boost::ptr_vector< ScopedRemoveRestraint> restraints;
  typedef IMP::internal::Map<double, btCollisionShape*> Spheres;
  Spheres spheres;
  for (unsigned int i=0; i< ps_.size(); ++i) {
    if (core::RigidBody::particle_is_instance(ps_[i])) {
      core::RigidBody d(ps_[i]);
      IMP_USAGE_CHECK(false, "Rigid bodies not yet supported");
      //http://www.bulletphysics.com/Bullet/BulletFull/classbtMultiSphereShape.html
    } else if (core::XYZR::particle_is_instance(ps_[i])){
      core::XYZR d(ps_[i]);
      btScalar mass=1;
      if (atom::Mass::particle_is_instance(ps_[i])) {
        mass= atom::Mass(ps_[i]).get_mass();
      }
      Spheres::const_iterator it= spheres.find(d.get_radius());
      btCollisionShape* shape;
      if (it != spheres.end()) {
        shape= it->second;
      } else {
        shape= new btSphereShape(d.get_radius());
        spheres[d.get_radius()]= shape;
        shapes.push_back(shape);
      }

      btDefaultMotionState* fallMotionState
               = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),
                                                      tr(d.get_coordinates())));
      motion_states.push_back(fallMotionState);
      btVector3 fallInertia(0,0,0);
      shape->calculateLocalInertia(mass,fallInertia);
      btRigidBody::btRigidBodyConstructionInfo
        fallRigidBodyCI(mass,fallMotionState,shape,fallInertia);
      btRigidBody* fallRigidBody= new btRigidBody(fallRigidBodyCI);
      map[ps_[i]]= fallRigidBody;
      dynamicsWorld->addRigidBody(fallRigidBody);
      rigid_bodies.push_back(fallRigidBody);
    }
  }
  for (unsigned int i=0; i< rs_.size(); ++i) {
    handle_restraints(rs_[i], rs_[i]->get_weight(), dynamicsWorld.get(),
                      map, springs, restraints);
  }
  IMP_LOG(TERSE, "Special cased " << restraints.size()
          << " restraint." << std::endl);
  unsigned int rrs=0;
  for (unsigned int i=0; i< rs_.size(); ++i) {
    rrs+=get_restraints(rs_[i]).size();
  }
  IMP_LOG(TERSE, "Remaining " << rrs << " restraints." << std::endl);
  for (unsigned int i=0; i< iter; ++i) {
    if (rrs >0) {
      get_model()->evaluate(RestraintsTemp(rs_.begin(), rs_.end()), true);
      for (unsigned int j=0; j< ps_.size(); ++j) {
        core::XYZ d(ps_[j]);
        rigid_bodies[j].applyCentralForce(tr(d.get_derivatives()));
      }
    }
    dynamicsWorld->stepSimulation(1/60.f,10);
    if (iter== i+1 || rrs >0) {
      for (unsigned int j=0; j< ps_.size(); ++j) {
        btTransform trans;
        rigid_bodies[j].getMotionState()->getWorldTransform(trans);
        core::XYZ(ps_[j]).set_coordinates(tr(trans.getOrigin()));
      }
    }
    update_states();
  }
  return get_model()->evaluate(RestraintsTemp(rs_.begin(), rs_.end()), true);
}


void ResolveCollisionsOptimizer::do_show(std::ostream &out) const {
}


IMPBULLET_END_NAMESPACE
