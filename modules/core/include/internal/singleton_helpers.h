/**
 *  \file internal/particle_helpers.h
 *  \brief A container for Singletons.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_SINGLETON_HELPERS_H
#define IMPCORE_INTERNAL_SINGLETON_HELPERS_H

#include "../core_config.h"
#include <IMP/SingletonContainer.h>
#include <IMP/SingletonModifier.h>
#include <IMP/SingletonScore.h>
#include <IMP/DecomposableRestraint.h>
#include <IMP/internal/container_helpers.h>
#include <algorithm>

IMP_BEGIN_INTERNAL_NAMESPACE
template <class Score>
struct SimpleRestraintParentTraits<Score,
       typename boost::enable_if<
         boost::is_base_of<SingletonScore, Score> >::type> {
  typedef IMP::SingletonScoreRestraint SimpleRestraint;
  typedef IMP::SingletonsScoreRestraint SimplesRestraint;
};
IMP_END_INTERNAL_NAMESPACE

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikeSingletonContainer: public SingletonContainer {
private:
  Particles data_;
  mutable ParticlesTemp index_;
  void update_index() const {
    if (index_.size()==data_.size()) return;
    unsigned int osize=index_.size();
    index_.insert(index_.end(), data_.begin()+osize, data_.end());
    std::sort(index_.begin()+osize, index_.end());
    std::inplace_merge(index_.begin(), index_.begin()+osize, index_.end());
  }
protected:
  ListLikeSingletonContainer *get_added() const {
    if (get_has_added_and_removed_containers()) {
      return dynamic_cast<ListLikeSingletonContainer*>
        (get_added_container());
    } else {
      return NULL;
    }
  }
  ListLikeSingletonContainer *get_removed() const {
    return dynamic_cast<ListLikeSingletonContainer*>
      (get_removed_container());
  }
  ListLikeSingletonContainer(){}
  void update_list(ParticlesTemp &cur) {
    index_.clear();
    IMP_IF_CHECK(USAGE) {
      for (unsigned int i=0; i< cur.size(); ++i) {
        IMP_USAGE_CHECK(
         IMP::internal::is_valid(cur[i]),
         "Passed Singleton cannot be NULL (or None)");
      }
    }
    if (get_added()) {
      std::sort(cur.begin(), cur.end());
      std::sort(data_.begin(), data_.end());
      ParticlesTemp added, removed;
      std::set_difference(cur.begin(), cur.end(),
                          data_.begin(), data_.end(),
                          std::back_inserter(added));
      std::set_difference(data_.begin(), data_.end(),
                          cur.begin(), cur.end(),
                          std::back_inserter(removed));
      get_added()->data_=added;
      get_removed()->data_=removed;
    }
    swap(data_, cur);
  }
  void add_to_list(ParticlesTemp &cur) {
    std::sort(cur.begin(), cur.end());
    ParticlesTemp newlist;
    std::set_union(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(newlist));
    if (get_added()) {
      ParticlesTemp added;
      std::set_intersection(newlist.begin(), newlist.end(),
                            cur.begin(), cur.end(),
                            std::back_inserter(added));
      ListLikeSingletonContainer* ac=get_added();
      ac->data_.insert(ac->data_.end(), added.begin(), added.end());
    }
    swap(data_, newlist);
  }
  void remove_from_list(ParticlesTemp &cur) {
    index_.clear();
    std::sort(cur.begin(), cur.end());
    ParticlesTemp newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    if (get_has_added_and_removed_containers()) {
      ListLikeSingletonContainer* ac=get_removed();
      ac->data_.insert(ac->data_.end(), cur.begin(), cur.end());
    }
  }
  void add_to_list(Particle* cur) {
    if (!std::binary_search(data_.begin(), data_.end(), cur)) {
      data_.push_back(cur);
      if (get_added()) {
        ListLikeSingletonContainer* ac=get_added();
        ac->data_.push_back(cur);
      }
    }
  }
  ListLikeSingletonContainer(Model *m, std::string name):
    SingletonContainer(m,name){
  }
  template <class F>
   void apply_to_contents(F f) const {
#if BOOST_VERSION > 103500
    std::for_each(data_.begin(), data_.end(), f);
#else
    for (unsigned int i=0; i< data_.size(); ++i) {
      Particle* v= data_[i];
      f(v);
    }
#endif
  }
  template <class F>
    typename F::result_type accumulate_over_contents(F f) const {
    typename F::result_type ret=0;
    for (unsigned int i=0; i< data_.size(); ++i) {
#if BOOST_VERSION > 103500
      ret+= f(data_[i]);
#else
      Particle* v= data_[i];
      ret+=f(v);
#endif
    }
    return ret;
  }
 public:
  template <class SM>
  void template_apply(const SM *sm,
                      DerivativeAccumulator &da) {
     apply_to_contents(boost::bind(static_cast<void (SingletonModifier::*)
                        (Particle*,DerivativeAccumulator&) const>
                        (&SingletonModifier::apply), sm, _1, da));
 }
  template <class SM>
  void template_apply(const SM *sm) {
    apply_to_contents(boost::bind(static_cast<void (SingletonModifier::*)
                    (Particle*) const>(&SingletonModifier::apply),
                        sm, _1));
  }
  template <class SS>
  double template_evaluate(const SS *s,
                           DerivativeAccumulator *da) const {
    return accumulate_over_contents(boost::bind(static_cast<double
                                                (SingletonScore::*)
                        (Particle*,DerivativeAccumulator*) const>
                               (&SingletonScore::evaluate), s, _1, da));
  }
  template <class SS>
  double template_evaluate_change(const SS *s,
                                  DerivativeAccumulator *da) const {
     return accumulate_over_contents(boost::bind(static_cast<double
                                                (SingletonScore::*)
                        (Particle*,DerivativeAccumulator*) const>
                       (&SingletonScore::evaluate_change), s, _1, da));
 }
  template <class SS>
  double template_evaluate_prechange(const SS *s,
                                     DerivativeAccumulator *da) const {
    return accumulate_over_contents(boost::bind(static_cast<double
                                                (SingletonScore::*)
                        (Particle*,DerivativeAccumulator*) const>
                    (&SingletonScore::evaluate_prechange), s, _1, da));
  }
  void apply(const SingletonModifier *sm) {
    sm->apply(data_);
  }
  void apply(const SingletonModifier *sm,
             DerivativeAccumulator &da) {
    sm->apply(data_, da);
  }
  double evaluate(const SingletonScore *s,
                  DerivativeAccumulator *da) const {
    return s->evaluate(data_, da);
  }
  double evaluate_change(const SingletonScore *s,
                         DerivativeAccumulator *da) const {
    return s->evaluate_change(data_, da);
  }
  double evaluate_prechange(const SingletonScore *s,
                            DerivativeAccumulator *da) const {
    return s->evaluate_prechange(data_, da);
  }
  ParticlesTemp get_contained_particles() const;
  bool get_contained_particles_changed() const;
  SingletonContainerPair get_added_and_removed_containers() const;
  bool get_contains_particle(Particle* p) const;
  unsigned int get_number_of_particles() const;
  Particle* get_particle(unsigned int i) const;
  IMP_OBJECT(ListLikeSingletonContainer);
  typedef Particles::const_iterator ParticleIterator;
  ParticleIterator particles_begin() const {
    return data_.begin();
  }
  ParticleIterator particles_end() const {
    return data_.end();
  }
  ObjectsTemp get_input_objects() const;
  void do_after_evaluate() {
    if (get_added()) {
      get_added()->data_.clear();
      get_removed()->data_.clear();
    }
  }
  void do_before_evaluate() {
    std::remove_if(data_.begin(), data_.end(),
         IMP::internal::IsInactive());
  }
  bool get_is_up_to_date() const {return true;}
  bool get_provides_access() const {return true;}
  const ParticlesTemp& get_access() const {
    IMP_INTERNAL_CHECK(get_is_up_to_date(),
                       "Container is out of date");
    return data_;
  }
};


IMPCORE_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_SINGLETON_CONTAINER(Name)               \
  IMP_OBJECT(Name)


#endif  /* IMPCORE_INTERNAL_SINGLETON_HELPERS_H */
