/**
 *  \file internal/particle_triplet_helpers.h
 *  \brief A container for Triplets.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_TRIPLET_HELPERS_H
#define IMPCORE_INTERNAL_TRIPLET_HELPERS_H

#include "../core_config.h"
#include <IMP/TripletContainer.h>
#include <IMP/TripletModifier.h>
#include <IMP/TripletScore.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/compatibility/set.h>
#include <algorithm>

IMP_BEGIN_INTERNAL_NAMESPACE
template <class Score>
struct SimpleRestraintParentTraits<Score,
       typename boost::enable_if<
         boost::is_base_of<TripletScore, Score> >::type> {
  typedef IMP::TripletScoreRestraint SimpleRestraint;
  typedef IMP::TripletsScoreRestraint SimplesRestraint;
};
IMP_END_INTERNAL_NAMESPACE

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikeTripletContainer: public TripletContainer {
private:
  ParticleIndexTriplets data_;
  bool sorted_;
  bool dirty_;
  void sort() const {
    std::sort(const_cast<ParticleIndexTriplets&>(data_).begin(),
              const_cast<ParticleIndexTriplets&>(data_).end());
    const_cast<bool&>(sorted_)=true;
  }
protected:
  void update_list(ParticleIndexTriplets &cur) {
    dirty_=true;
    swap(data_, cur);
    sorted_=false;
  }
  void add_to_list(ParticleIndexTriplets &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    // set union assumes things are unique
    cur.erase(std::unique(cur.begin(), cur.end()), cur.end());
    ParticleIndexTriplets newlist;
    std::set_union(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }

  void remove_from_list(ParticleIndexTriplets &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    ParticleIndexTriplets newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }
  template <class F>
    struct AccIf {
    F f_;
    mutable ParticleIndexTriplets rem_;
    AccIf(F f, ParticleIndexTriplets &rem): f_(f), rem_(rem){}
    bool operator()(const ParticleIndexTriplet& cur) const {
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
  void add_to_list(const ParticleIndexTriplet& cur) {
    if (!sorted_) sort();
    if (!std::binary_search(data_.begin(), data_.end(), cur)) {
      data_.insert(std::lower_bound(data_.begin(), data_.end(),
                                   cur), cur);
      dirty_=true;
    }
  }
  ListLikeTripletContainer(Model *m, std::string name):
    TripletContainer(m,name), sorted_(false), dirty_(false){
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
  void apply(const TripletModifier *sm) {
    sm->apply_indexes(get_model(), data_);
  }
  void apply(const TripletDerivativeModifier *sm,
             DerivativeAccumulator &da) {
    sm->apply_indexes(get_model(), data_, da);
  }
  double evaluate(const TripletScore *s,
                  DerivativeAccumulator *da) const {
    return s->evaluate_indexes(get_model(), data_, da);
  }
  double evaluate_if_good(const TripletScore *s,
                          DerivativeAccumulator *da,
                          double max) const {
    return s->evaluate_if_good_indexes(get_model(), data_, da, max);
  }
  ParticlesTemp get_contained_particles() const {
    return IMP::internal::flatten(IMP::internal::get_particle(get_model(),
                                                              data_));
  }
  bool get_contains_particle_triplet(const ParticleTriplet& p) const {
    if (!sorted_) sort();
    ParticleIndexTriplet it= IMP::internal::get_index(p);
    return std::binary_search(data_.begin(), data_.end(), it);
  }
  unsigned int get_number_of_particle_triplets() const {
    return data_.size();
  }
  ParticleTriplet get_particle_triplet(unsigned int i) const {
    return IMP::internal::get_particle(get_model(), data_[i]);
  }
  bool get_contents_changed() const {
    return dirty_;
  }
  IMP_OBJECT(ListLikeTripletContainer);
  void do_after_evaluate() {
    dirty_=false;
  }
  void do_before_evaluate() {
  }
  bool get_is_up_to_date() const {return true;}

  ParticleIndexTriplets get_indexes() const {
    return data_;
  }
  bool get_provides_access() const {return true;}
  const ParticleIndexTriplets& get_access() const {
    return data_;
  }
};


IMPCORE_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_TRIPLET_CONTAINER(Name)               \
  IMP_OBJECT(Name)


#endif  /* IMPCORE_INTERNAL_TRIPLET_HELPERS_H */
