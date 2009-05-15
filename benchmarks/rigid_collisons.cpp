/**
 * Copyright 2007-9 Sali Lab. All rights reserved.
 */

#include <IMP.h>
#include <IMP/core.h>
#include <IMP/algebra.h>
#include <IMP/atom.h>
#include <boost/timer.hpp>

using namespace IMP;
using namespace IMP::core;
using namespace IMP::algebra;
using namespace IMP::atom;

void test_one(Model *m,
              std::vector<RigidBody> rbs,
              float side) {
  Vector3D minc(0,0,0), maxc(side, side, side);
  set_log_level(SILENT);
  m->evaluate(false);
  set_log_level(SILENT);
  double runtime, inittime;
  IMP_TIME(
           {
             for (unsigned int i=0; i< rbs.size(); ++i) {
               Vector3D t= random_vector_in_box(minc, maxc);
               Rotation3D r= random_rotation();
               Transformation3D tr(r, t);
               rbs[i].set_transformation(tr, true);
             }
           }, inittime);
  double value=0;
  IMP_TIME(
           {
             for (unsigned int i=0; i< rbs.size(); ++i) {
               Vector3D t= random_vector_in_box(minc, maxc);
               Rotation3D r= random_rotation();
               Transformation3D tr(r, t);
               rbs[i].set_transformation(tr, true);
             }
             value+=m->evaluate(false);
           }, runtime);

  std::cout << " took " << runtime-inittime << " with side " << side
            << " and value " << value << std::endl;
}

int main() {
  Pointer<Model> m(new Model());
  Particles atoms;
  std::vector<RigidBody> rbs;
  for (int i=0; i< 5; ++i) {
    MolecularHierarchy mhd
      = read_pdb("benchmarks/input/single_protein.pdb", m);
    Particles catoms= get_by_type(mhd, MolecularHierarchy::ATOM);
    IMP_assert(catoms.size() != 0, "What happened to the atoms?");
    atoms.insert(atoms.end(), catoms.begin(), catoms.end());
    ScoreState *ss= create_rigid_body(mhd.get_particle(),
                                       catoms);
    m->add_score_state(ss);
    rbs.push_back(RigidBody(mhd.get_particle()));
    cover_members(rbs.back());
  }
  for (unsigned int i=0; i< atoms.size(); ++i) {
    XYZR::create(atoms[i], 1);
  }
  IMP_NEW(ListSingletonContainer, lsc, (atoms));
  IMP_NEW(ClosePairsScoreState, cpss, (lsc));
  m->add_score_state(cpss);
  IMP_NEW(PairsRestraint, pr,
          (new DistancePairScore(new Linear(1,0)),
           cpss->get_close_pairs_container()));
  m->add_restraint(pr);
  {
    IMP_NEW(QuadraticClosePairsFinder,qcpf, ());
    //lsc->set_particles(atoms);
    cpss->set_close_pairs_finder(qcpf);
    std::cout << "Quadratic:" << std::endl;
    test_one(m, rbs, 10);

    test_one(m, rbs, 100);

    test_one(m, rbs, 1000);

  }
  {
    Particles rbsp(rbs.size());
    for (unsigned int i=0; i< rbs.size(); ++i){
      rbsp[i]= rbs[i].get_particle();
    }
    lsc->set_particles(rbsp);
    IMP_NEW(RigidClosePairsFinder, rcps, ());
    cpss->set_close_pairs_finder(rcps);
    std::cout << "Hierarchy:" << std::endl;
    test_one(m, rbs, 10);

    test_one(m, rbs, 100);

    test_one(m, rbs, 1000);

  }
  return 0;
}
