/**
 *  \file internal/quad_helpers.h
 *  \brief A container for particle_quads.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_QUAD_HELPERS_H
#define IMPCORE_INTERNAL_QUAD_HELPERS_H

#include "../config.h"
#include <IMP/QuadContainer.h>
#include <IMP/QuadModifier.h>
#include <IMP/internal/container_helpers.h>
#include <algorithm>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikeQuadContainer: public QuadContainer {
private:
  void set_added_and_removed_containers(QuadContainer *,
                                        QuadContainer *){}
  ListLikeQuadContainer *get_added() const {
    return dynamic_cast<ListLikeQuadContainer*>
      (get_added_quads_container());
  }
  ListLikeQuadContainer *get_removed() const {
    return dynamic_cast<ListLikeQuadContainer*>
      (get_removed_quads_container());
  }
  ParticleQuads data_;
protected:
  ListLikeQuadContainer():
    QuadContainer("Added or removed container"){}
  const ParticleQuads &access() const {return data_;}
  void update_list(ParticleQuadsTemp &cur) {
    IMP_IF_CHECK(USAGE) {
      for (unsigned int i=0; i< cur.size(); ++i) {
        IMP_USAGE_CHECK(
         IMP::internal::is_valid(cur[i]),
         "Passed ParticleQuad cannot be NULL (or None)");
      }
    }
    std::sort(cur.begin(), cur.end());
    if (!get_is_added_or_removed_container()) {
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
  }
  void add_to_list(ParticleQuadsTemp &cur) {
    std::sort(cur.begin(), cur.end());
    ParticleQuadsTemp added;
    std::set_difference(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(added));
    unsigned int osz= data_.size();
    data_.insert(data_.end(), added.begin(), added.end());
    std::inplace_merge(data_.begin(), data_.begin()+osz, data_.end());
    if (!get_is_added_or_removed_container()) {
      ListLikeQuadContainer* ac=get_added();
      ac->data_.insert(ac->data_.end(), added.begin(), added.end());
    }
  }
  void add_to_list(const ParticleQuad& cur) {
    data_.insert(std::lower_bound(data_.begin(),
                                  data_.end(), cur), cur);
    if (!get_is_added_or_removed_container()) {
      ListLikeQuadContainer* ac=get_added();
      ac->data_.push_back(cur);
    }
  }
  ListLikeQuadContainer(std::string name): QuadContainer(name){
    QuadContainer::
      set_added_and_removed_containers( new ListLikeQuadContainer(),
                                        new ListLikeQuadContainer());
  }
public:
  ParticleQuad get_particle_quad(unsigned int i) const;
  void apply(const QuadModifier *sm);
  void apply(const QuadModifier *sm,
             DerivativeAccumulator &da);
  double evaluate(const QuadScore *s,
                  DerivativeAccumulator *da) const;
  double evaluate_change(const QuadScore *s,
                         DerivativeAccumulator *da) const;
  double evaluate_prechange(const QuadScore *s,
                            DerivativeAccumulator *da) const;
  unsigned int get_number_of_particle_quads() const;
  bool get_contains_particle_quad(const ParticleQuad& vt) const;
  typedef ParticleQuads::const_iterator ParticleQuadIterator;
  ParticleQuadIterator particle_quads_begin() const {
    return data_.begin();
  }
  ParticleQuadIterator particle_quads_end() const {
    return data_.end();
  }
  ObjectsTemp get_input_objects() const;
  void do_after_evaluate() {
    get_added()->data_.clear();
    get_removed()->data_.clear();
  }
  void do_before_evaluate() {
    std::remove_if(data_.begin(), data_.end(),
         IMP::internal::IsInactive());
  }
  IMP_OBJECT(ListLikeQuadContainer);
  bool get_contained_particles_changed() const;
  ParticlesTemp get_contained_particles() const;
  ContainersTemp get_input_containers() const {
    return ContainersTemp();
  }
};


IMPCORE_END_INTERNAL_NAMESPACE


#define IMP_LISTLIKE_QUAD_CONTAINER(Name)               \
  ContainersTemp get_input_containers() const;               \
  IMP_OBJECT(Name);


#endif  /* IMPCORE_INTERNAL_QUAD_HELPERS_H */
