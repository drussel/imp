/**
 *  \file SingletonScore.cpp  \brief Define SingletonScore
 *
 *  WARNING This file was generated from NAMEScore.cc
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/SingletonScore.h>
#include <IMP/internal/utility.h>
#include <IMP/Restraint.h>
#include <IMP/singleton_macros.h>
#include <IMP/internal/TupleRestraint.h>
IMP_BEGIN_NAMESPACE

SingletonScore::SingletonScore(std::string name):
  Object(name)
{
  /* Implemented here rather than in the header so that PairScore
     symbols are present in the kernel DSO */
}

double SingletonScore::evaluate(Particle* vt,
                                DerivativeAccumulator *da) const {
  return evaluate_index(internal::get_model(vt),
                        internal::get_index(vt),
                        da);
}

// old versions of gcc don't like having the pragma inside the function
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
IMP_DEPRECATED_IGNORE(
double SingletonScore::evaluate_index(Model *m, ParticleIndex vt,
                                      DerivativeAccumulator *da) const {
  // see http://gcc.gnu.org/bugzilla/show_bug.cgi?id=53469
   return evaluate(internal::get_particle(m, vt), da);
})
#pragma GCC diagnostic pop

double SingletonScore::evaluate_indexes(Model *m,
                                        const ParticleIndexes &o,
                                        DerivativeAccumulator *da,
                                        unsigned int lower_bound,
                                        unsigned int upper_bound) const {
  double ret=0;
  for (unsigned int i=lower_bound; i< upper_bound; ++i) {
    ret+= evaluate_index(m, o[i], da);
  }
  return ret;
}


double SingletonScore::evaluate_if_good_index(Model *m,
                                              ParticleIndex vt,
                                              DerivativeAccumulator *da,
                                              double max)
  const {
  IMP_UNUSED(max);
  return evaluate_index(m, vt, da);
}

double SingletonScore::evaluate_if_good_indexes(Model *m,
                                                const ParticleIndexes &o,
                                                DerivativeAccumulator *da,
                                                double max,
                                                unsigned int lower_bound,
                                                unsigned int upper_bound)
  const {
  double ret=0;
  for (unsigned int i=lower_bound; i< upper_bound; ++i) {
    double cur= evaluate_if_good_index(m, o[i], da, max-ret);
    max-=cur;
    ret+=cur;
    if (max<0) break;
  }
  return ret;
}


Restraints
SingletonScore
::do_create_current_decomposition(Model *m,
                                  ParticleIndex vt) const {
  double score=evaluate_index(m, vt, nullptr);
  if (score==0) {
    return Restraints();
  } else {
    return Restraints(1, IMP::internal::create_tuple_restraint(this,
                                                               m,
                                                               vt,
                                                               get_name()));
  }
}

Restraints
SingletonScore
::create_current_decomposition(Model *m,
                               ParticleIndex vt) const {
  return do_create_current_decomposition(m, vt);
}

IMP_INPUTS_DEF(SingletonScore);

IMP_END_NAMESPACE
