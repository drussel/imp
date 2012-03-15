/**
 *  \file internal/core_particle_quad_helpers.h
 *  \brief A container for Quads.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_INTERNAL_LIST_LIKE_QUAD_CONTAINER_H
#define IMPKERNEL_INTERNAL_LIST_LIKE_QUAD_CONTAINER_H

#include "../kernel_config.h"
#include "../QuadContainer.h"
#include "../QuadModifier.h"
#include "../QuadScore.h"
#include "../scoped.h"
#include "container_helpers.h"
#include "quad_helpers.h"
#include <algorithm>


IMP_BEGIN_INTERNAL_NAMESPACE

class IMPEXPORT ListLikeQuadContainer: public QuadContainer {
private:
  ParticleIndexQuads data_;
  bool sorted_;
  bool dirty_;
  void sort() const {
    std::sort(const_cast<ParticleIndexQuads&>(data_).begin(),
              const_cast<ParticleIndexQuads&>(data_).end());
    const_cast<bool&>(sorted_)=true;
  }
protected:
  void update_list(ParticleIndexQuads &cur) {
    dirty_=true;
    swap(data_, cur);
    sorted_=false;
  }
  void add_to_list(ParticleIndexQuads &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    // set union assumes things are unique
    cur.erase(std::unique(cur.begin(), cur.end()), cur.end());
    ParticleIndexQuads newlist;
    std::set_union(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }

  void remove_from_list(ParticleIndexQuads &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    ParticleIndexQuads newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    dirty_=true;
  }
  template <class F>
    struct AccIf {
    F f_;
    mutable ParticleIndexQuads rem_;
    AccIf(F f, ParticleIndexQuads &rem): f_(f), rem_(rem){}
    bool operator()(const ParticleIndexQuad& cur) const {
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
  void add_to_list(const ParticleIndexQuad& cur) {
    if (!sorted_) sort();
    if (!std::binary_search(data_.begin(), data_.end(), cur)) {
      data_.insert(std::lower_bound(data_.begin(), data_.end(),
                                   cur), cur);
      dirty_=true;
    }
  }
  ListLikeQuadContainer(Model *m, std::string name):
    QuadContainer(m,name), sorted_(false), dirty_(false){
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
  void apply(const QuadModifier *sm) const {
    sm->apply_indexes(get_model(), data_);
  }
  void apply(const QuadDerivativeModifier *sm,
             DerivativeAccumulator &da) const {
    sm->apply_indexes(get_model(), data_, da);
  }
  double evaluate(const QuadScore *s,
                  DerivativeAccumulator *da) const {
    return s->evaluate_indexes(get_model(), data_, da);
  }
  double evaluate_if_good(const QuadScore *s,
                          DerivativeAccumulator *da,
                          double max) const {
    return s->evaluate_if_good_indexes(get_model(), data_, da, max);
  }
  ParticlesTemp get_contained_particles() const {
    return IMP::internal::flatten(IMP::internal::get_particle(get_model(),
                                                              data_));
  }
  bool get_contains_particle_quad(const ParticleQuad& p) const {
    if (!sorted_) sort();
    ParticleIndexQuad it= IMP::internal::get_index(p);
    return std::binary_search(data_.begin(), data_.end(), it);
  }
  bool get_contents_changed() const {
    return dirty_;
  }
  IMP_OBJECT(ListLikeQuadContainer);
  void do_after_evaluate() {
    dirty_=false;
  }
  void do_before_evaluate() {
  }
  bool get_is_up_to_date() const {return true;}

  ParticleIndexQuads get_indexes() const {
    return data_;
  }
  bool get_provides_access() const {return true;}
  const ParticleIndexQuads& get_access() const {
    return data_;
  }

  typedef ParticleIndexQuads::const_iterator const_iterator;
  const_iterator begin() const {
    return data_.begin();
  }
  const_iterator end() const {
    return data_.end();
  }
};


IMP_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_QUAD_CONTAINER(Name)                         \
  ParticleIndexQuads get_all_possible_indexes() const;                     \
  Restraints create_decomposition(QuadScore *s) const {            \
    ParticleIndexQuads all= get_all_possible_indexes();                    \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]= IMP::create_restraint(s,                                  \
                            IMP::internal::get_particle(get_model(), \
                                          all[i]));                     \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  template <class S>                                                    \
  Restraints create_decomposition_t(S *s) const {                       \
    ParticleIndexQuads all= get_all_possible_indexes();                    \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]= IMP::create_restraint(s,                            \
                            IMP::internal::get_particle(get_model(), \
                                          all[i]));                     \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT(Name)


#endif  /* IMPKERNEL_INTERNAL_LIST_LIKE_QUAD_CONTAINER_H */