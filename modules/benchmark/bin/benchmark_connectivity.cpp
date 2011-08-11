/**
 * Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#include <IMP.h>
#include <IMP/core.h>
#include <boost/timer.hpp>
#include <IMP/benchmark/utility.h>
#include <IMP/benchmark/benchmark_macros.h>
#include <IMP/algebra.h>
#include <IMP/container.h>

using namespace IMP;
using namespace IMP::core;
using namespace IMP::algebra;
using namespace IMP::container;

namespace {
void benchmark_it(std::string name, ListSingletonContainer *lsc, Model *m) {
  double runtime;
  double value=0;
  m->evaluate(false);
  IMP_TIME({
      unsigned int np= lsc->get_number_of_particles();
      for (unsigned int i=0; i< np; ++i) {
        XYZ(lsc->get_particle(i))
          .set_coordinates(get_random_vector_in(BoundingBox3D(Vector3D(0,0,0),
                                                        Vector3D(10,10,10))));
      }
      value+= m->evaluate(false);
    }, runtime);
  IMP::benchmark::report(name, runtime, value);
}
}

int main() {
  unsigned int npart=1000;
  set_log_level(SILENT);
  //set_check_level(NONE);
  IMP_NEW(Model, m, ());
  IMP_NEW(HarmonicUpperBound, ub, (1.0, 0.1));
  IMP_NEW(SphereDistancePairScore, ss,(ub));
  Particles ps = create_xyzr_particles(m, npart, .1);
  IMP_NEW(ListSingletonContainer, lsc, (ps));
  {
    ConnectivityRestraint* r= new ConnectivityRestraint(ss, lsc);
    m->add_restraint(r);
    benchmark_it("connectivity slow", lsc, m);
    m->remove_restraint(r);
  }
  {
    IMP_NEW(ConnectingPairContainer, cpc,(lsc, .1));
    IMP_NEW(PairsRestraint, pr, (ss, cpc));
    m->add_restraint(pr);
    benchmark_it("connectivity fast", lsc, m);
    m->remove_restraint(pr);
  }
  return IMP::benchmark::get_return_value();
}
