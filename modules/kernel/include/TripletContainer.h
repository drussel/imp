/**
 *  \file TripletContainer.h    \brief A container for Triplets.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_TRIPLET_CONTAINER_H
#define IMPKERNEL_TRIPLET_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "Particle.h"
#include "container_base.h"
#include "VersionInfo.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "ParticleTuple.h"
#include "TripletScore.h"
#include "TripletModifier.h"
#include "TripletDerivativeModifier.h"
#include "macros.h"

IMP_BEGIN_NAMESPACE

// for swig
class TripletScore;
class TripletModifier;


//! A shared container for Triplets
/** Stores a searchable shared collection of Triplets.
    \ingroup restraints

    \implementationwithoutexample{TripletContainer, IMP_TRIPLET_CONTAINER}
 */
class IMPEXPORT TripletContainer : public Container
{
 protected:
  TripletContainer(){}
  TripletContainer(Model *m,
                     std::string name="TripletContainer %1%");
#ifndef IMP_DOXYGEN
  template <class S>
    double call_evaluate_index(const S *s,
                         const ParticleIndexTriplet& a,
                         DerivativeAccumulator *da) const {
    return s->S::evaluate_index(get_model(), a, da);
  }
  double call_evaluate_index(const TripletScore *s,
                              const ParticleIndexTriplet& a,
                              DerivativeAccumulator *da) const {
    return s->evaluate_index(get_model(), a, da);
  }
  template <class S>
    double call_evaluate_if_good_index(const S *s,
                                 const ParticleIndexTriplet& a,
                                 DerivativeAccumulator *da,
                                 double max) const {
    return s->S::evaluate_if_good_index(get_model(), a, da, max);
  }
  double call_evaluate_if_good_index(const TripletScore *s,
                                      const ParticleIndexTriplet& a,
                                      DerivativeAccumulator *da,
                                      double max) const {
    return s->evaluate_if_good_index(get_model(), a, da, max);
  }
  template <class S>
    void call_apply_index(const S *s,
                    const ParticleIndexTriplet& a) const {
    s->S::apply_index(get_model(), a);
  }
  void call_apply(const TripletModifier *s,
                         const ParticleIndexTriplet& a) const {
    s->apply_index(get_model(), a);
  }
  template <class S>
    void call_apply_index(const S *s,
                           const ParticleIndexTriplet& a,
                           DerivativeAccumulator *&da) const {
    s->S::apply_index(get_model(), a, da);
  }
  void call_apply_index(const TripletDerivativeModifier *s,
                  const ParticleIndexTriplet& a,
                  DerivativeAccumulator &da) const {
    s->apply_index(get_model(), a, da);
  }
#endif
public:
  typedef ParticleTriplet ContainedType;
  typedef ParticleTripletsTemp ContainedTypes;
  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_particle_triplet(const ParticleTriplet& v) const =0;

  ParticleTripletsTemp get_particle_triplets() const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes());
  }
#ifndef IMP_DOXGEN
  //! return the number of Triplets in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_particle_triplets() const {
    return get_number();
  }

  virtual ParticleTriplet get_particle_triplet(unsigned int i) const {
    return get(i);
  }

#endif

  //! Apply a SingletonModifier to the contents
  virtual void apply(const TripletModifier *sm) const=0;
  //! Apply a SingletonModifier to the contents
  virtual void apply(const TripletDerivativeModifier *sm,
                     DerivativeAccumulator &da) const=0;

  //! Evaluate a score on the contents
  virtual double evaluate(const TripletScore *s,
                          DerivativeAccumulator *da) const=0;

  //! Evaluate a score on the contents
  virtual double evaluate_if_good(const TripletScore *s,
                                  DerivativeAccumulator *da,
                                  double max) const=0;

  /** Return true if the contents of the container changed since the last
      evaluate.
  */
  virtual bool get_contents_changed() const=0;

#ifndef IMP_DOXYGEN
  typedef ParticleTriplet value_type;
  ParticleTriplet get(unsigned int i) const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes()[i]);
  }
  ParticleTripletsTemp get() const {
    return IMP::internal::get_particle(get_model(), get_indexes());
  }
  bool get_contains(const ParticleTriplet& v) const {
    return get_contains_particle_triplet(v);
  }
  unsigned int get_number() const {return get_indexes().size();}
  virtual ParticleIndexTriplets get_indexes() const=0;
  virtual ParticleIndexTriplets get_all_possible_indexes() const=0;
  virtual Restraints create_decomposition(TripletScore *s) const=0;
#ifndef SWIG
  virtual bool get_provides_access() const {return false;}
  virtual const ParticleIndexTriplets& get_access() const {
    IMP_THROW("Object not implemented properly.", IndexException);
  }
#endif
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(TripletContainer);
};

IMP_OBJECTS(TripletContainer,TripletContainers);

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_TRIPLET_CONTAINER_H */
