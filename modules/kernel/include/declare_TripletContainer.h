/**
 *  \file IMP/declare_TripletContainer.h
 *  \brief A container for Triplets.
 *
 *  WARNING This file was generated from declare_NAMEContainer.hpp
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_DECLARE_TRIPLET_CONTAINER_H
#define IMPKERNEL_DECLARE_TRIPLET_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "declare_Particle.h"
#include "container_base.h"
#include "internal/container_helpers.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "ParticleTuple.h"
#include <IMP/base/ref_counted_macros.h>
#include <IMP/base/check_macros.h>
#include <IMP/base/Pointer.h>
#include <IMP/base/InputAdaptor.h>
#include <IMP/base/utility_macros.h>
#include <algorithm>


IMP_BEGIN_NAMESPACE
class TripletModifier;
class TripletDerivativeModifier;
class TripletScore;

//! A shared container for Triplets
/** Stores a searchable shared collection of Triplets.
    \headerfile TripletContainer.h "IMP/TripletContainer.h"
    \implementationwithoutexample{TripletContainer, IMP_TRIPLET_CONTAINER}
 */
class IMPEXPORT TripletContainer : public Container
{
  IMP_PROTECTED_CONSTRUCTOR(TripletContainer, (Model *m,
                           std::string name="TripletContainer %1%"), );
public:
  typedef ParticleTriplet ContainedType;
  typedef ParticleTripletsTemp ContainedTypes;
  typedef ParticleIndexTriplets ContainedIndexTypes;
  typedef ParticleIndexTriplet ContainedIndexType;
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
  /** Return the ith ParticleTriplet of the container.*/
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

  /** Get all the indexes contained in the container.*/
  virtual ParticleIndexTriplets get_indexes() const=0;
  /** Get all the indexes that might possibly be contained in the
      container, useful with dynamic containers.*/
  virtual ParticleIndexTriplets get_all_possible_indexes() const=0;

#ifndef IMP_DOXYGEN
  ParticleTripletsTemp get() const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes());
  }

  ParticleTriplet get(unsigned int i) const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes()[i]);
  }
  /** Return true if the container contains the passed ParticleTriplet.*/
  bool get_contains(const ParticleTriplet& v) const {
    return get_contains_particle_triplet(v);
  }
  /** Return true if the container contains the passed ParticleTriplet.*/
  virtual bool get_contains_index(ParticleIndexTriplet v) const {
    return get_contains_particle_triplet(IMP::internal
                                     ::get_particle(get_model(),
                                                    v));
  }
  unsigned int get_number() const {return get_indexes().size();}
#ifndef SWIG
  virtual bool get_provides_access() const {return false;}
  virtual const ParticleIndexTriplets& get_access() const {
    IMP_THROW("Object not implemented properly.", base::IndexException);
  }


  template <class Functor>
    Functor for_each(Functor f) {
    ParticleIndexTriplets vs=get_indexes();
    // use boost range instead
    return std::for_each(vs.begin(), vs.end(), f);
  }

#endif
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(TripletContainer);
};


/** This class allows either a list or a container to be
    accepted as input.
*/
class IMPEXPORT TripletContainerAdaptor:
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
public base::Pointer<TripletContainer>
#else
public base::InputAdaptor
#endif
{
  typedef base::Pointer<TripletContainer> P;
 public:
  TripletContainerAdaptor(){}
  TripletContainerAdaptor(TripletContainer *c);
  template <class C>
  TripletContainerAdaptor(base::internal::PointerBase<C> c): P(c){}
  TripletContainerAdaptor(const ParticleTripletsTemp &t,
                          std::string name="TripletContainerAdaptor%1%");
};


IMP_END_NAMESPACE

#endif  /* IMPKERNEL_DECLARE_TRIPLET_CONTAINER_H */
