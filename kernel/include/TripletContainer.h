/**
 *  \file TripletContainer.h    \brief A container for particle_triplets.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_TRIPLET_CONTAINER_H
#define IMP_TRIPLET_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "Particle.h"
#include "container_base.h"
#include "utility.h"
#include "VersionInfo.h"
#include "base_types.h"
#include "Pointer.h"
#include "VectorOfRefCounted.h"
#include "VersionInfo.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "macros.h"

IMP_BEGIN_NAMESPACE
class TripletModifier;
class TripletScore;


//! A shared container for particle_triplets
/** Stores a searchable shared collection of particle_triplets.
    \ingroup restraints

    \implementationwithoutexample{TripletContainer, IMP_TRIPLET_CONTAINER}
 */
class IMPEXPORT TripletContainer : public Container
{
  internal::OwnerPointer<Container> added_, removed_;
  struct Accessor {
    typedef Accessor This;
    typedef ParticleTriplet result_type;
    typedef unsigned int argument_type;
    result_type operator()(argument_type i) const {
      return o_->get_particle_triplet(i);
    }
    Accessor(TripletContainer *pc): o_(pc){}
    Accessor(): o_(NULL){}
    IMP_COMPARISONS_1(o_);
  private:
    // This should be ref counted, but swig memory management is broken
    TripletContainer* o_;
  };
 protected:
  /** Containers must have containers that keep track of the particles
      which have been added or since the last step. These containers
      must be registered with the parent TripletContainer.

      Containers which are themselves returned by the get_added/removed
      functions do not have to register such containers.
  */
  void set_added_and_removed_containers(TripletContainer* added,
                                        TripletContainer* removed) {
    added_=added;
    removed_=removed;
  }

public:
#ifndef IMP_DOXYGEN
  bool get_is_added_or_removed_container() {
    return !added_;
  }
#endif

  TripletContainer(std::string name="TripletContainer %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_particle_triplet(const ParticleTriplet& v) const =0;
  //! return the number of particle_triplets in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_particle_triplets() const =0;

  ParticleTripletsTemp get_particle_triplets() const {
    return ParticleTripletsTemp(particle_triplets_begin(),
                          particle_triplets_end());
  }
  virtual ParticleTriplet get_particle_triplet(unsigned int i) const=0;

#ifdef IMP_DOXYGEN
  //! An iterator through the contents of the container
  class ParticleTripletIterator;
#else
  typedef internal::IndexingIterator<Accessor> ParticleTripletIterator;
#endif
  //! begin iterating through the particle_triplets
  ParticleTripletIterator particle_triplets_begin() const {
    // Since I can't make the count mutable in Object
    return
      ParticleTripletIterator(Accessor(const_cast<TripletContainer*>(this)),
                        0);
  }
  //! iterate through the particle_triplets
  ParticleTripletIterator particle_triplets_end() const {
    return
      ParticleTripletIterator(Accessor(const_cast<TripletContainer*>(this)),
                        get_number_of_particle_triplets());
    }

  //! Apply a SingletonModifier to the contents
  virtual void apply(const TripletModifier *sm)=0;

  //! Apply a SingletonModifier to the contents
  virtual void apply(const TripletModifier *sm, DerivativeAccumulator &da)=0;

  //! Avaluate a score on the contents
  virtual double evaluate(const TripletScore *s,
                          DerivativeAccumulator *da) const=0;

  /** \name Incremental Scoring
      When incremental scoring is used, the container keeps track of
      changes to it since the last Model::evaluate() call.
      \unstable{ParticleTripletContainer::get_removed_triplets_container()}
      The address of the objects returned should not change over the lifetime
      of this container (but, of course, their contents will).
      @{
  */
  TripletContainer* get_removed_triplets_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_triplets_container() do not "
                    << " track their own added and removed contents.");
    TripletContainer *ret= dynamic_cast<TripletContainer*>(removed_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << removed_->get_name()
                       << " to a TripletContainer.");
    return ret;
  }
  TripletContainer* get_added_triplets_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_triplets_container() do not "
                    << " track their own added and removed contents.");
    TripletContainer *ret= dynamic_cast<TripletContainer*>(added_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << added_->get_name()
                       << " to a TripletContainer.");
    return ret;
  }
  /** Return the change in score (and derivatives) since the last
      evaluate of the current contents of the container.
  */
  virtual double evaluate_change(const TripletScore *o,
                                DerivativeAccumulator *da) const = 0;


  /** Return the score of the last evaluate for the current contents of the
      container.
  */
  virtual double evaluate_prechange(const TripletScore *o,
                                    DerivativeAccumulator *da) const = 0;
  /** @} */

#ifndef IMP_DOXYGEN
  ParticleTriplet get(unsigned int i) const {return get_particle_triplet(i);}
  bool get_contains(const ParticleTriplet& v) const {
    return get_contains_particle_triplet(v);
  }
  unsigned int get_number() const {return get_number_of_particle_triplets();}
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(TripletContainer);
};

IMP_OUTPUT_OPERATOR(TripletContainer);

IMP_OBJECTS(TripletContainer,TripletContainers);

IMP_END_NAMESPACE

#endif  /* IMP_TRIPLET_CONTAINER_H */
