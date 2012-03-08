/**
 *  \file internal/particle_helpers.h
 *  \brief A container for Singletons.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_SINGLETON_HELPERS_H
#define IMPCORE_INTERNAL_SINGLETON_HELPERS_H

#include "../core_config.h"
#include <IMP/log.h>
#include <IMP/SingletonContainer.h>
#include <IMP/SingletonModifier.h>
#include <IMP/SingletonScore.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/core/SingletonRestraint.h>
#include <IMP/compatibility/set.h>
#include <IMP/internal/singleton_helpers.h>
#include <algorithm>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikeSingletonContainer: public SingletonContainer {
private:
  ParticleIndexes data_;
  bool sorted_;
  bool dirty_;
  void sort() const {
    std::sort(const_cast<ParticleIndexes&>(data_).begin(),
              const_cast<ParticleIndexes&>(data_).end());
    const_cast<bool&>(sorted_)=true;
  }
protected:
  void update_list(ParticleIndexes &cur) {
    dirty_=true;
    swap(data_, cur);
    sorted_=false;
  }
  void add_to_list(ParticleIndexes &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    // set union assumes things are unique
    cur.erase(std::unique(cur.begin(), cur.end()), cur.end());
    ParticleIndexes newlist;
    std::set_union(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }

  void remove_from_list(ParticleIndexes &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    ParticleIndexes newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }
  template <class F>
    struct AccIf {
    F f_;
    mutable ParticleIndexes rem_;
    AccIf(F f, ParticleIndexes &rem): f_(f), rem_(rem){}
    bool operator()(ParticleIndex cur) const {
      if (f_(cur)) {
        rem_.push_back(cur);
        return true;
      }
      return false;
    }
  };
  template <class F>
  void remove_from_list_if(F f) {
    data_.erase(std::remove_if(data_.begin(), data_.end(), f), data_.end());
    dirty_=true;
  }
  void add_to_list(ParticleIndex cur) {
    if (!sorted_) sort();
    if (!std::binary_search(data_.begin(), data_.end(), cur)) {
      data_.insert(std::lower_bound(data_.begin(), data_.end(),
                                   cur), cur);
      dirty_=true;
    }
  }
  ListLikeSingletonContainer(Model *m, std::string name):
    SingletonContainer(m,name), sorted_(false), dirty_(false){
  }
 public:
  template <class SM>
  void template_apply(const SM *sm,
                      DerivativeAccumulator &da) const {
    sm->apply_indexes(get_model(), data_, da);
 }
  template <class SM>
  void template_apply(const SM *sm) const {
    sm->apply_indexes(get_model(), data_);
  }
  template <class SS>
  double template_evaluate(const SS *s,
                           DerivativeAccumulator *da) const {
    return s->evaluate_indexes(get_model(), data_, da);
  }
  template <class SS>
  double template_evaluate_if_good(const SS *s,
                                   DerivativeAccumulator *da,
                                   double max) const {
    return s->evaluate_if_good_indexes(get_model(), data_, da, max);
  }
  void apply(const SingletonModifier *sm) const {
    sm->apply_indexes(get_model(), data_);
  }
  void apply(const SingletonDerivativeModifier *sm,
             DerivativeAccumulator &da) const {
    sm->apply_indexes(get_model(), data_, da);
  }
  double evaluate(const SingletonScore *s,
                  DerivativeAccumulator *da) const {
    return s->evaluate_indexes(get_model(), data_, da);
  }
  double evaluate_if_good(const SingletonScore *s,
                          DerivativeAccumulator *da,
                          double max) const {
    return s->evaluate_if_good_indexes(get_model(), data_, da, max);
  }
  ParticlesTemp get_contained_particles() const {
    return IMP::internal::flatten(IMP::internal::get_particle(get_model(),
                                                              data_));
  }
  bool get_contains_particle(Particle* p) const {
    if (!sorted_) sort();
    ParticleIndex it= IMP::internal::get_index(p);
    return std::binary_search(data_.begin(), data_.end(), it);
  }
  bool get_contents_changed() const {
    return dirty_;
  }
  IMP_OBJECT(ListLikeSingletonContainer);
  void do_after_evaluate() {
    dirty_=false;
  }
  void do_before_evaluate() {
  }
  bool get_is_up_to_date() const {return true;}

  ParticleIndexes get_indexes() const {
    return data_;
  }
  bool get_provides_access() const {return true;}
  const ParticleIndexes& get_access() const {
    return data_;
  }

  typedef ParticleIndexes::const_iterator const_iterator;
  const_iterator begin() const {
    return data_.begin();
  }
  const_iterator end() const {
    return data_.end();
  }
};


IMPCORE_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_SINGLETON_CONTAINER(Name)                         \
  ParticleIndexes get_all_possible_indexes() const;                     \
  Restraints create_decomposition(SingletonScore *s) const {            \
    ParticleIndexes all= get_all_possible_indexes();                    \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]= new IMP::core::SingletonRestraint(s,                      \
                            IMP::internal::get_particle(get_model(), \
                                          all[i]));                     \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  template <class S>                                                    \
  Restraints create_decomposition_t(S *s) const {                       \
    ParticleIndexes all= get_all_possible_indexes();                    \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]= IMP::create_restraint(s,                            \
                            IMP::internal::get_particle(get_model(), \
                                          all[i]));                     \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT(Name)


#endif  /* IMPCORE_INTERNAL_SINGLETON_HELPERS_H */
