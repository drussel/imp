/*
 * Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#include <IMP.h>
#include <IMP/core.h>
#include <IMP/examples/ExampleRestraint.h>
#include <IMP/container/ListPairContainer.h>

using namespace IMP;
int main(){
  set_log_level(VERBOSE);
  WeakPointer<Model> m(new Model());
  Particle* p0= new Particle(m);
  core::XYZ d0= core::XYZ::setup_particle(p0);
  Particle* p1= new Particle(m);
  core::XYZ d1= core::XYZ::setup_particle(p1);
  core::DistancePairScore *dps
    = new core::DistancePairScore(new core::Linear(0,1));
  container::ListPairContainer *pc= new container::ListPairContainer();
  pc->add_particle_pair(ParticlePair(p0, p1));
  examples::ExampleRestraint *r= new examples::ExampleRestraint(dps, pc);
  d0.set_coordinates(algebra::Vector3D(0,0,0));
  d1.set_coordinates(algebra::Vector3D(0,0,1));

  m->add_restraint(r);
  IMP_INTERNAL_CHECK(std::abs(m->evaluate(false) -1) < .01, "Out of range");
  return 0;
}
