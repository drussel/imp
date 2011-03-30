/**
 *  \file internal/particle_quad_helpers.h
 *  \brief A container for Quads.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_QUAD_HELPERS_H
#define IMPCORE_INTERNAL_QUAD_HELPERS_H

#include "../core_config.h"
#include <IMP/QuadContainer.h>
#include <IMP/QuadModifier.h>
#include <IMP/QuadScore.h>
#include <IMP/internal/container_helpers.h>
#include <algorithm>

IMP_BEGIN_INTERNAL_NAMESPACE
template <class Score>
struct SimpleRestraintParentTraits<Score,
       typename boost::enable_if<
         boost::is_base_of<QuadScore, Score> >::type> {
  typedef IMP::QuadScoreRestraint SimpleRestraint;
  typedef IMP::QuadsScoreRestraint SimplesRestraint;
};
IMP_END_INTERNAL_NAMESPACE

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikeQuadContainer: public QuadContainer {
private:
  ParticleQuads data_;
  bool changed_;
  typedef IMP::internal::Set<ParticleQuad> Index;
  mutable Index index_;
  void update_index() const {
    if (index_.size()==data_.size()) return;
    else {
      index_= Index(data_.begin(), data_.end());
    }
  }
protected:
  ListLikeQuadContainer *get_added() const {
    if (get_has_added_and_removed_containers()) {
      return dynamic_cast<ListLikeQuadContainer*>
        (get_added_container());
    } else {
      return NULL;
    }
  }
  ListLikeQuadContainer *get_removed() const {
    return dynamic_cast<ListLikeQuadContainer*>
      (get_removed_container());
  }
  ListLikeQuadContainer(){}
  void update_list(ParticleQuadsTemp &cur) {
    index_.clear();
    IMP_IF_CHECK(USAGE) {
      for (unsigned int i=0; i< cur.size(); ++i) {
        IMP_USAGE_CHECK(
         IMP::internal::is_valid(cur[i]),
         "Passed Quad cannot be NULL (or None)");
      }
    }
    if (get_added()) {
      std::sort(cur.begin(), cur.end());
      std::sort(data_.begin(), data_.end());
      ParticleQuadsTemp added, removed;
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
    changed_=true;
  }
  void add_to_list(ParticleQuadsTemp &cur) {
    std::sort(cur.begin(), cur.end());
    ParticleQuadsTemp newlist;
    std::set_union(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(newlist));
    if (get_added()) {
      ParticleQuadsTemp added;
      std::set_intersection(newlist.begin(), newlist.end(),
                            cur.begin(), cur.end(),
                            std::back_inserter(added));
      ListLikeQuadContainer* ac=get_added();
      ac->data_.insert(ac->data_.end(), added.begin(), added.end());
    }
    swap(data_, newlist);
    changed_=true;
  }

  template <class Table>
    bool has_permutation(const Table &t, Particle *p) const {
    return t.find(p) != t.end();
  }
  template <class Table>
   bool has_permutation(const Table &t, const ParticlePair &p) const {
    return t.find(p) != t.end() || t.find(ParticlePair(p[1], p[0])) != t.end();
  }
  template <class Table>
   bool has_permutation(const Table &t, const ParticleTriplet &p) const {
    return t.find(p) != t.end()
      || t.find(ParticleTriplet(p[0], p[2], p[1])) != t.end()
      || t.find(ParticleTriplet(p[1], p[0], p[2])) != t.end()
      || t.find(ParticleTriplet(p[1], p[2], p[0])) != t.end()
      || t.find(ParticleTriplet(p[2], p[0], p[1])) != t.end()
      || t.find(ParticleTriplet(p[2], p[1], p[0])) != t.end();
  }
  template <class Table>
   bool has_permutation(const Table &, const ParticleQuad &) const {
    IMP_NOT_IMPLEMENTED;
    return false;
  }

  void add_unordered_to_list(const ParticleQuadsTemp &cur) {
    update_index();
    ParticleQuadsTemp added;
    for (unsigned int i=0; i<cur.size(); ++i) {
      if (has_permutation(index_, cur[i])) {
      } else {
        index_.insert(cur[i]);
        data_.push_back(cur[i]);
        if (get_added()) {
          added.push_back(cur[i]);
        }
      }
    }
    changed_=true;
  }
  void remove_from_list(ParticleQuadsTemp &cur) {
    index_.clear();
    std::sort(cur.begin(), cur.end());
    ParticleQuadsTemp newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    if (get_has_added_and_removed_containers()) {
      ListLikeQuadContainer* ac=get_removed();
      ac->data_.insert(ac->data_.end(), cur.begin(), cur.end());
    }
    changed_=true;
  }
  template <class F>
    struct AccIf {
    F f_;
    mutable ParticleQuadsTemp rem_;
    AccIf(F f, ParticleQuadsTemp &rem): f_(f), rem_(rem){}
    bool operator()(const ParticleQuad& cur) const {
      if (f_(cur)) {
        rem_.push_back(cur);
        return true;
      } return false;
    }
  };
  template <class F>
  void remove_from_list_if(F f) {
    index_.clear();
    unsigned int sz= data_.size();
    if (get_has_added_and_removed_containers()) {
      ParticleQuadsTemp removed;
      data_.remove_if(AccIf<F>(f, removed));
      ListLikeQuadContainer* ac=get_removed();
      ac->data_.insert(ac->data_.end(), removed.begin(), removed.end());
    } else {
      data_.remove_if(f);
    }
    changed_= data_.size() != sz;
  }
  void add_to_list(const ParticleQuad& cur) {
    if (!std::binary_search(data_.begin(), data_.end(), cur)) {
      data_.push_back(cur);
      if (get_added()) {
        ListLikeQuadContainer* ac=get_added();
        ac->data_.push_back(cur);
      }
    }
    changed_=true;
  }
  ListLikeQuadContainer(Model *m, std::string name):
    QuadContainer(m,name), changed_(false){
  }
  template <class F>
   void apply_to_contents(F f) const {
#if BOOST_VERSION > 103500
    std::for_each(data_.begin(), data_.end(), f);
#else
    for (unsigned int i=0; i< data_.size(); ++i) {
      ParticleQuad v= data_[i];
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
      ParticleQuad v= data_[i];
      ret+=f(v);
#endif
    }
    return ret;
  }
 public:
  template <class SM>
  void template_apply(const SM *sm,
                      DerivativeAccumulator &da) {
     apply_to_contents(boost::bind(static_cast<void (QuadModifier::*)
                        (const ParticleQuad&,DerivativeAccumulator&) const>
                        (&QuadModifier::apply), sm, _1, da));
 }
  template <class SM>
  void template_apply(const SM *sm) {
    apply_to_contents(boost::bind(static_cast<void (QuadModifier::*)
                    (const ParticleQuad&) const>(&QuadModifier::apply),
                        sm, _1));
  }
  template <class SS>
  double template_evaluate(const SS *s,
                           DerivativeAccumulator *da) const {
    return accumulate_over_contents(boost::bind(static_cast<double
                                                (QuadScore::*)
                        (const ParticleQuad&,DerivativeAccumulator*) const>
                               (&QuadScore::evaluate), s, _1, da));
  }
  template <class SS>
  double template_evaluate_change(const SS *s,
                                  DerivativeAccumulator *da) const {
     return accumulate_over_contents(boost::bind(static_cast<double
                                                (QuadScore::*)
                        (const ParticleQuad&,DerivativeAccumulator*) const>
                       (&QuadScore::evaluate_change), s, _1, da));
 }
  template <class SS>
  double template_evaluate_prechange(const SS *s,
                                     DerivativeAccumulator *da) const {
    return accumulate_over_contents(boost::bind(static_cast<double
                                                (QuadScore::*)
                        (const ParticleQuad&,DerivativeAccumulator*) const>
                    (&QuadScore::evaluate_prechange), s, _1, da));
  }
  void apply(const QuadModifier *sm) {
    sm->apply(data_);
  }
  void apply(const QuadModifier *sm,
             DerivativeAccumulator &da) {
    sm->apply(data_, da);
  }
  double evaluate(const QuadScore *s,
                  DerivativeAccumulator *da) const {
    return s->evaluate(data_, da);
  }
  double evaluate_change(const QuadScore *s,
                         DerivativeAccumulator *da) const {
    return s->evaluate_change(data_, da);
  }
  double evaluate_prechange(const QuadScore *s,
                            DerivativeAccumulator *da) const {
    return s->evaluate_prechange(data_, da);
  }
  ParticlesTemp get_contained_particles() const;
  bool get_contained_particles_changed() const;
  bool get_listed_particle_quad_changed() const {
    return changed_;
  }
  QuadContainerPair get_added_and_removed_containers() const;
  bool get_contains_particle_quad(const ParticleQuad& p) const;
  unsigned int get_number_of_particle_quads() const;
  ParticleQuad get_particle_quad(unsigned int i) const;
  IMP_OBJECT(ListLikeQuadContainer);
  typedef ParticleQuads::const_iterator ParticleQuadIterator;
  ParticleQuadIterator particle_quads_begin() const {
    return data_.begin();
  }
  ParticleQuadIterator particle_quads_end() const {
    return data_.end();
  }
  ObjectsTemp get_input_objects() const;
  void do_after_evaluate() {
    if (get_added()) {
      get_added()->data_.clear();
      get_removed()->data_.clear();
    }
    changed_=false;
  }
  void do_before_evaluate() {
    std::remove_if(data_.begin(), data_.end(),
         IMP::internal::IsInactive());
  }
  bool get_is_up_to_date() const {return true;}
  bool get_provides_access() const {return true;}
  const ParticleQuadsTemp& get_access() const {
    IMP_INTERNAL_CHECK(get_is_up_to_date(),
                       "Container is out of date");
    return data_;
  }
};


IMPCORE_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_QUAD_CONTAINER(Name)               \
  IMP_OBJECT(Name)


#endif  /* IMPCORE_INTERNAL_QUAD_HELPERS_H */
