/**
 *  \file PairContainer.h    \brief A container for Pairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_PAIR_CONTAINER_H
#define IMP_PAIR_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "Particle.h"
#include "container_base.h"
#include "VersionInfo.h"
#include "VectorOfRefCounted.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "ParticleTuple.h"
#include "macros.h"

IMP_BEGIN_NAMESPACE
class PairModifier;
class PairScore;

class PairContainer;
typedef std::pair<PairContainer*,
                  PairContainer*> PairContainerPair;

//! A shared container for Pairs
/** Stores a searchable shared collection of Pairs.
    \ingroup restraints

    \implementationwithoutexample{PairContainer, IMP_PAIR_CONTAINER}
 */
class IMPEXPORT PairContainer : public Container
{
  mutable internal::OwnerPointer<Container> added_, removed_;
  struct Accessor {
    typedef ParticlePair result_type;
    typedef unsigned int argument_type;
    result_type operator()(argument_type i) const {
      return o_->get_particle_pair(i);
    }
    Accessor(PairContainer *pc): o_(pc){}
    Accessor(): o_(NULL){}
    IMP_COMPARISONS_1(Accessor, o_);
  private:
    // This should be ref counted, but swig memory management is broken
    PairContainer* o_;
  };
 protected:
  /** Containers must have containers that keep track of the particles
      which have been added or since the last step. These containers
      must be registered with the parent PairContainer.

      Containers which are themselves returned by the get_added/removed
      functions do not have to register such containers.
  */
  virtual PairContainerPair
    get_added_and_removed_containers() const =0;
  bool get_has_added_and_removed_containers() const {
    return (added_ && added_->get_is_shared())
      || (removed_ && removed_->get_is_shared());
  }
  PairContainer(){}
  PairContainer(Model *m,
                     std::string name="PairContainer %1%");
public:
  typedef ParticlePair ContainedType;
  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_particle_pair(const ParticlePair& v) const =0;
  //! return the number of Pairs in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_particle_pairs() const =0;

  ParticlePairsTemp get_particle_pairs() const {
    return ParticlePairsTemp(particle_pairs_begin(),
                              particle_pairs_end());
  }
  virtual ParticlePair get_particle_pair(unsigned int i) const=0;

#ifdef IMP_DOXYGEN
  //! An iterator through the contents of the container
  class ParticlePairIterator;
#else
  typedef internal::IndexingIterator<Accessor> ParticlePairIterator;
#endif
#ifndef SWIG
  //! begin iterating through the Pairs
  ParticlePairIterator particle_pairs_begin() const {
    // Since I can't make the count mutable in Object
    return
      ParticlePairIterator(Accessor(const_cast<PairContainer*>(this)),
                        0);
  }
  //! iterate through the Pairs
  ParticlePairIterator particle_pairs_end() const {
    return
      ParticlePairIterator(Accessor(const_cast<PairContainer*>(this)),
                        get_number_of_particle_pairs());
    }
#endif

  //! Apply a SingletonModifier to the contents
  virtual void apply(const PairModifier *sm)=0;

  //! Apply a SingletonModifier to the contents
  virtual void apply(const PairModifier *sm, DerivativeAccumulator &da)=0;

  //! Evaluate a score on the contents
  virtual double evaluate(const PairScore *s,
                          DerivativeAccumulator *da) const=0;


  /** \name Incremental Scoring
      When incremental scoring is used, the container keeps track of
      changes to it since the last Model::evaluate() call.
      \unstable{PairContainer::get_removed_container()}
      The address of the objects returned should not change over the lifetime
      of this container (but, of course, their contents will).
      @{
  */
  PairContainer* get_removed_container() const {
    // must not be an added or removed container
    get_model();
    if (!added_) {
      std::pair<PairContainer*, PairContainer*>
        cp= get_added_and_removed_containers();
      added_=cp.first;
      removed_=cp.second;
    }
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_container() do not "
                    << " track their own added and removed contents.");
    PairContainer *ret= dynamic_cast<PairContainer*>(removed_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << removed_->get_name()
                       << " to a PairContainer.");
    return ret;
  }
  PairContainer* get_added_container() const {
    // must not be an added or removed container
    if (!added_) {
      std::pair<PairContainer*, PairContainer*>
        cp= get_added_and_removed_containers();
      added_=cp.first;
      removed_=cp.second;
    }
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_container() do not "
                    << " track their own added and removed contents.");
    PairContainer *ret= dynamic_cast<PairContainer*>(added_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << added_->get_name()
                       << " to a PairContainer.");
    return ret;
  }
  /** Return the change in score (and derivatives) since the last
      evaluate of the current contents of the container.
  */
  virtual double evaluate_change(const PairScore *o,
                                DerivativeAccumulator *da) const = 0;


  /** Return the score of the last evaluate for the current contents of the
      container.
  */
  virtual double evaluate_prechange(const PairScore *o,
                                    DerivativeAccumulator *da) const = 0;
  /** @} */

#ifndef IMP_DOXYGEN
  typedef ParticlePair value_type;
  ParticlePair get(unsigned int i) const {return get_particle_pair(i);}
  ParticlePairsTemp get() const {
    return get_particle_pairs();
  }
  bool get_contains(const ParticlePair& v) const {
    return get_contains_particle_pair(v);
  }
  unsigned int get_number() const {return get_number_of_particle_pairs();}
  virtual bool get_provides_access() const {return false;}
  virtual const ParticlePairsTemp& get_access() const {
    IMP_THROW("Object not implemented properly.", IndexException);
  }
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(PairContainer);
};

IMP_OBJECTS(PairContainer,PairContainers);

IMP_END_NAMESPACE

#endif  /* IMP_PAIR_CONTAINER_H */
