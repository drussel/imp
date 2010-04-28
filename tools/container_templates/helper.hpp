/**
 *  \file internal/groupname_helpers.h
 *  \brief A container for classnames.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_GROUPNAME_HELPERS_H
#define IMPCORE_INTERNAL_GROUPNAME_HELPERS_H

#include "../core_config.h"
#include <IMP/GroupnameContainer.h>
#include <IMP/GroupnameModifier.h>
#include <IMP/internal/container_helpers.h>
#include <algorithm>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT ListLikeGroupnameContainer: public GroupnameContainer {
private:
  void set_added_and_removed_containers(GroupnameContainer *,
                                        GroupnameContainer *){}
  ListLikeGroupnameContainer *get_added() const {
    return dynamic_cast<ListLikeGroupnameContainer*>
      (get_added_groupnames_container());
  }
  ListLikeGroupnameContainer *get_removed() const {
    return dynamic_cast<ListLikeGroupnameContainer*>
      (get_removed_groupnames_container());
  }
  Classnames data_;
protected:
  ListLikeGroupnameContainer(){}
  void update_list(ClassnamesTemp &cur) {
    IMP_IF_CHECK(USAGE) {
      for (unsigned int i=0; i< cur.size(); ++i) {
        IMP_USAGE_CHECK(
         IMP::internal::is_valid(cur[i]),
         "Passed Classname cannot be NULL (or None)");
      }
    }
    std::sort(cur.begin(), cur.end());
    if (!get_is_added_or_removed_container()) {
      ClassnamesTemp added, removed;
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
  void add_to_list(ClassnamesTemp &cur) {
    std::sort(cur.begin(), cur.end());
    ClassnamesTemp added;
    std::set_difference(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(added));
    unsigned int osz= data_.size();
    data_.insert(data_.end(), added.begin(), added.end());
    std::inplace_merge(data_.begin(), data_.begin()+osz, data_.end());
    if (!get_is_added_or_removed_container()) {
      ListLikeGroupnameContainer* ac=get_added();
      ac->data_.insert(ac->data_.end(), added.begin(), added.end());
    }
  }
  void add_to_list(PassValue cur) {
    data_.insert(std::lower_bound(data_.begin(),
                                  data_.end(), cur), cur);
    if (!get_is_added_or_removed_container()) {
      ListLikeGroupnameContainer* ac=get_added();
      ac->data_.push_back(cur);
    }
  }
  ListLikeGroupnameContainer(Model *m, std::string name):
    GroupnameContainer(m,name){
    GroupnameContainer::
      set_added_and_removed_containers( new ListLikeGroupnameContainer(),
                                        new ListLikeGroupnameContainer());
  }
public:
  Value get_classname(unsigned int i) const;
  void apply(const GroupnameModifier *sm);
  void apply(const GroupnameModifier *sm,
             DerivativeAccumulator &da);
  double evaluate(const GroupnameScore *s,
                  DerivativeAccumulator *da) const;
  double evaluate_change(const GroupnameScore *s,
                         DerivativeAccumulator *da) const;
  double evaluate_prechange(const GroupnameScore *s,
                            DerivativeAccumulator *da) const;
  unsigned int get_number_of_classnames() const;
  bool get_contains_classname(PassValue vt) const;
  typedef Classnames::const_iterator ClassnameIterator;
  ClassnameIterator classnames_begin() const {
    return data_.begin();
  }
  ClassnameIterator classnames_end() const {
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
  bool get_is_up_to_date() const {return true;}
  IMP_OBJECT(ListLikeGroupnameContainer);
  bool get_contained_particles_changed() const;
  ParticlesTemp get_contained_particles() const;
  bool get_provides_access() const {return true;}
  const ClassnamesTemp& get_access() const {
    IMP_INTERNAL_CHECK(get_is_up_to_date(),
                       "Container is out of date");
    return data_;
  }
};


IMPCORE_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_GROUPNAME_CONTAINER(Name)               \
  IMP_OBJECT(Name)


#endif  /* IMPCORE_INTERNAL_GROUPNAME_HELPERS_H */
