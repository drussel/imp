/**
 *  \file internal/particle_pair_helpers.h
 *  \brief A container for Pairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_PAIR_HELPERS_H
#define IMPCORE_INTERNAL_PAIR_HELPERS_H

#include "../core_config.h"
#include <IMP/PairContainer.h>
#include <IMP/PairModifier.h>
#include <IMP/PairScore.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/core/PairRestraint.h>
#include <IMP/compatibility/set.h>
#include <algorithm>

IMP_BEGIN_INTERNAL_NAMESPACE
template <class Score>
struct SimpleRestraintParentTraits<Score,
       typename boost::enable_if<
         boost::is_base_of<PairScore, Score> >::type> {
  typedef IMP::PairScoreRestraint SimpleRestraint;
  typedef IMP::PairsScoreRestraint SimplesRestraint;
};
IMP_END_INTERNAL_NAMESPACE

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikePairContainer: public PairContainer {
private:
  ParticleIndexPairs data_;
  bool sorted_;
  bool dirty_;
  void sort() const {
    std::sort(const_cast<ParticleIndexPairs&>(data_).begin(),
              const_cast<ParticleIndexPairs&>(data_).end());
    const_cast<bool&>(sorted_)=true;
  }
protected:
  void update_list(ParticleIndexPairs &cur) {
    dirty_=true;
    swap(data_, cur);
    sorted_=false;
  }
  void add_to_list(ParticleIndexPairs &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    // set union assumes things are unique
    cur.erase(std::unique(cur.begin(), cur.end()), cur.end());
    ParticleIndexPairs newlist;
    std::set_union(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }

  void remove_from_list(ParticleIndexPairs &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    ParticleIndexPairs newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }
  template <class F>
    struct AccIf {
    F f_;
    mutable ParticleIndexPairs rem_;
    AccIf(F f, ParticleIndexPairs &rem): f_(f), rem_(rem){}
    bool operator()(const ParticleIndexPair& cur) const {
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
  void add_to_list(const ParticleIndexPair& cur) {
    if (!sorted_) sort();
    if (!std::binary_search(data_.begin(), data_.end(), cur)) {
      data_.insert(std::lower_bound(data_.begin(), data_.end(),
                                   cur), cur);
      dirty_=true;
    }
  }
  ListLikePairContainer(Model *m, std::string name):
    PairContainer(m,name), sorted_(false), dirty_(false){
  }
 public:
  template <class SM>
  void template_apply(const SM *sm,
                      DerivativeAccumulator &da) {
    sm->apply_indexes(get_model(), data_, da);
 }
  template <class SM>
  void template_apply(const SM *sm) {
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
  void apply(const PairModifier *sm) {
    sm->apply_indexes(get_model(), data_);
  }
  void apply(const PairDerivativeModifier *sm,
             DerivativeAccumulator &da) {
    sm->apply_indexes(get_model(), data_, da);
  }
  double evaluate(const PairScore *s,
                  DerivativeAccumulator *da) const {
    return s->evaluate_indexes(get_model(), data_, da);
  }
  double evaluate_if_good(const PairScore *s,
                          DerivativeAccumulator *da,
                          double max) const {
    return s->evaluate_if_good_indexes(get_model(), data_, da, max);
  }
  ParticlesTemp get_contained_particles() const {
    return IMP::internal::flatten(IMP::internal::get_particle(get_model(),
                                                              data_));
  }
  bool get_contains_particle_pair(const ParticlePair& p) const {
    if (!sorted_) sort();
    ParticleIndexPair it= IMP::internal::get_index(p);
    return std::binary_search(data_.begin(), data_.end(), it);
  }
  bool get_contents_changed() const {
    return dirty_;
  }
  IMP_OBJECT(ListLikePairContainer);
  void do_after_evaluate() {
    dirty_=false;
  }
  void do_before_evaluate() {
  }
  bool get_is_up_to_date() const {return true;}

  ParticleIndexPairs get_indexes() const {
    return data_;
  }
  bool get_provides_access() const {return true;}
  const ParticleIndexPairs& get_access() const {
    return data_;
  }

  typedef ParticleIndexPairs::const_iterator const_iterator;
  const_iterator begin() const {
    return data_.begin();
  }
  const_iterator end() const {
    return data_.end();
  }
};


IMPCORE_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_PAIR_CONTAINER(Name)                         \
  ParticleIndexPairs get_all_possible_indexes() const;                     \
  Restraints create_decomposition(PairScore *s) const {            \
    ParticleIndexPairs all= get_all_possible_indexes();                    \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]= new IMP::core::PairRestraint(s,                      \
                            IMP::internal::get_particle(get_model(), \
                                          all[i]));                     \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  template <class S>                                                    \
  Restraints create_decomposition_t(S *s) const {                       \
    ParticleIndexPairs all= get_all_possible_indexes();                    \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]= IMP::core::create_restraint(s,                            \
                            IMP::internal::get_particle(get_model(), \
                                          all[i]));                     \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT(Name)


#endif  /* IMPCORE_INTERNAL_PAIR_HELPERS_H */
