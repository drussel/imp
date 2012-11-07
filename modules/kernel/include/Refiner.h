/**
 *  \file IMP/Refiner.h   \brief Refine a particle into a list of particles.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_REFINER_H
#define IMPKERNEL_REFINER_H

#include "kernel_config.h"
#include "base_types.h"
#include "Particle.h"
#include "VersionInfo.h"
#include "RefCounted.h"
#include "internal/IndexingIterator.h"
#include <IMP/base/deprecation_macros.h>

IMP_BEGIN_NAMESPACE

class Particle;
class DerivativeAccumulator;

//! Abstract class to implement hierarchical methods.
/** The job of this class is to take a single particle and, if
    appropriate, return a list of particles. These lists can
    reflect existing relationships, such as the
    IMP::core::LeavesRefiner or arbitrary relationships set up
    for a particular purpose, such as IMP::core::TableRefiner.

    Implementors should see IMP_REFINER().
*/
class IMPEXPORT Refiner : public IMP::base::Object
{
  struct Accessor;
public:
  Refiner(std::string name="Refiner %1%");
  //! Return true if this refiner can refine that particle
  /** This should not throw, so be careful what fields are touched.
   */
  virtual bool get_can_refine(Particle *) const {return false;}

  //! Refine the passed particle into a set of particles.
  /** As a precondition can_refine_particle(a) should be true.
   */
  virtual const ParticlesTemp get_refined(Particle *a) const=0;
  //! Get the ith refined particle.
  /** As a precondition can_refine_particle(a) should be true.
   */
  virtual Particle* get_refined(Particle *a, unsigned int i) const =0;

  /** As a precondition can_refine_particle(a) should be true.
   */
  virtual unsigned int get_number_of_refined(Particle *a) const =0;

#ifndef SWIG
  /** @name Iterating through the set of refined particles

      Using iterators can be more efficient than using the bulk
      get_refined(), however it is not necessarily so.
      @{
  */
  typedef internal::IndexingIterator<Accessor> RefinedIterator;
  RefinedIterator refined_begin(Particle *a) const;
  RefinedIterator refined_end(Particle *a) const;
  /** @} */
#endif

  virtual ModelObjectsTemp get_inputs(Model *m,
                                      const ParticleIndexes &pis) const =0;

#ifdef IMP_USE_DEPRECATED
  /** \deprecated use get_inputs() instead.*/
  IMP_DEPRECATED_WARN ParticlesTemp get_input_particles(Particle *p) const {
   IMP_DEPRECATED_FUNCTION(get_inputs());
   return IMP::get_input_particles(get_inputs(p->get_model(),
                                        ParticleIndexes(1, p->get_index())));
  }
  /** \deprecated use get_inputs() instead.*/
  IMP_DEPRECATED_WARN ContainersTemp get_input_containers(Particle *p) const {
    IMP_DEPRECATED_FUNCTION(get_inputs());
    return IMP::get_input_containers(get_inputs(p->get_model(),
                                          ParticleIndexes(1, p->get_index())));
  }
#endif
};
//! a collection of Refiner objects
IMP_OBJECTS(Refiner, Refiners);

#if !defined(SWIG) && !defined(IMP_DOXYGEN)
struct Refiner::Accessor {
  // can't reference count since swig memory management is broken
  Particle* p_;
  const Refiner* r_;
  Accessor(Particle *p, const Refiner *r): p_(p), r_(r) {}
  Accessor(){}
  typedef Particle *result_type;
  Particle *operator()(unsigned int i) const {
    return r_->get_refined(p_, i);
  }
  bool operator==(const Accessor &o) const {
    return p_==o.p_ && r_==o.r_;
  }
};
#endif

inline Refiner::RefinedIterator Refiner::refined_begin(Particle *a) const {
  return RefinedIterator(Accessor(a, this), 0);
}
inline Refiner::RefinedIterator Refiner::refined_end(Particle *a) const {
  return RefinedIterator(Accessor(a, this), get_number_of_refined(a));
}

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_REFINER_H */
