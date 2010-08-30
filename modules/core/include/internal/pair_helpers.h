/**
 *  \file internal/pair_helpers.h
 *  \brief A container for particle_pairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_PAIR_HELPERS_H
#define IMPCORE_INTERNAL_PAIR_HELPERS_H

#include "../core_config.h"
#include <IMP/PairContainer.h>
#include <IMP/PairModifier.h>
#include <IMP/internal/container_helpers.h>
#include <algorithm>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikePairContainer: public PairContainer {
private:
  ParticlePairs data_;
protected:
  ListLikePairContainer *get_added() const {
    if (get_has_added_and_removed_containers()) {
      return dynamic_cast<ListLikePairContainer*>
        (get_added_container());
    } else {
      return NULL;
    }
  }
  ListLikePairContainer *get_removed() const {
    return dynamic_cast<ListLikePairContainer*>
      (get_removed_container());
  }
  ListLikePairContainer(){}
  void update_list(ParticlePairsTemp &cur) {
    IMP_IF_CHECK(USAGE) {
      for (unsigned int i=0; i< cur.size(); ++i) {
        IMP_USAGE_CHECK(
         IMP::internal::is_valid(cur[i]),
         "Passed ParticlePair cannot be NULL (or None)");
      }
    }
    std::sort(cur.begin(), cur.end());
    if (get_added()) {
      ParticlePairsTemp added, removed;
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
  void add_to_list(ParticlePairsTemp &cur) {
    std::sort(cur.begin(), cur.end());
    ParticlePairsTemp added;
    std::set_difference(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(added));
    unsigned int osz= data_.size();
    data_.insert(data_.end(), added.begin(), added.end());
    std::inplace_merge(data_.begin(), data_.begin()+osz, data_.end());
    if (get_added()) {
      ListLikePairContainer* ac=get_added();
      ac->data_.insert(ac->data_.end(), added.begin(), added.end());
    }
  }
  void remove_from_list(ParticlePairsTemp &cur) {
    std::sort(cur.begin(), cur.end());
    ParticlePairsTemp newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    if (get_has_added_and_removed_containers()) {
      ListLikePairContainer* ac=get_removed();
      ac->data_.insert(ac->data_.end(), cur.begin(), cur.end());
    }
  }
  void add_to_list(const ParticlePair& cur) {
    data_.insert(std::lower_bound(data_.begin(),
                                  data_.end(), cur), cur);
    if (get_added()) {
      ListLikePairContainer* ac=get_added();
      ac->data_.push_back(cur);
    }
  }
  ListLikePairContainer(Model *m, std::string name):
    PairContainer(m,name){
  }
  template <class F>
    F foreach(F f) const {
    return std::for_each(data_.begin(), data_.end(), f);
  }
public:
  IMP_PAIR_CONTAINER(ListLikePairContainer);
  typedef ParticlePairs::const_iterator ParticlePairIterator;
  ParticlePairIterator particle_pairs_begin() const {
    return data_.begin();
  }
  ParticlePairIterator particle_pairs_end() const {
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
  const ParticlePairsTemp& get_access() const {
    IMP_INTERNAL_CHECK(get_is_up_to_date(),
                       "Container is out of date");
    return data_;
  }
};


IMPCORE_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_PAIR_CONTAINER(Name)               \
  IMP_OBJECT(Name)


#endif  /* IMPCORE_INTERNAL_PAIR_HELPERS_H */
