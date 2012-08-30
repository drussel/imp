/**
 *  \file internal/core_FUNCTIONNAME_helpers.h
 *  \brief A container for CLASSNAMEs.
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_INTERNAL_LIST_LIKE_HEADERNAME_CONTAINER_H
#define IMPKERNEL_INTERNAL_LIST_LIKE_HEADERNAME_CONTAINER_H

#include "../kernel_config.h"
#include "../CLASSNAMEContainer.h"
#include "../CLASSNAMEModifier.h"
#include "../CLASSNAMEScore.h"
#include "../scoped.h"
#include "TupleRestraint.h"
#include "container_helpers.h"
#include "LCCLASSNAME_helpers.h"
#include <algorithm>


IMP_BEGIN_INTERNAL_NAMESPACE

class IMPEXPORT ListLikeCLASSNAMEContainer: public CLASSNAMEContainer {
private:
  PLURALINDEXTYPE data_;
  bool sorted_;
  void sort() const {
    std::sort(const_cast<PLURALINDEXTYPE&>(data_).begin(),
              const_cast<PLURALINDEXTYPE&>(data_).end());
    const_cast<bool&>(sorted_)=true;
  }
  void sort() {
    std::sort(data_.begin(),
              data_.end());
    sorted_=true;
  }
protected:
  void update_list(PLURALINDEXTYPE &cur) {
    Container::set_is_changed(true);
    swap(data_, cur);
    sorted_=false;
  }
  void add_to_list(PLURALINDEXTYPE &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    // set union assumes things are unique
    cur.erase(std::unique(cur.begin(), cur.end()), cur.end());
    PLURALINDEXTYPE newlist;
    std::set_union(cur.begin(), cur.end(),
                        data_.begin(), data_.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    Container::set_is_changed(true);
  }

  void remove_from_list(PLURALINDEXTYPE &cur) {
    if (!sorted_) sort();
    std::sort(cur.begin(), cur.end());
    PLURALINDEXTYPE newlist;
    std::set_difference(data_.begin(), data_.end(),
                        cur.begin(), cur.end(),
                        std::back_inserter(newlist));
    swap(data_, newlist);
    Container::set_is_changed(true);
  }
  template <class F>
  void remove_from_list_if(F f) {
    data_.erase(std::remove_if(data_.begin(), data_.end(), f), data_.end());
    Container::set_is_changed(true);
  }
  void add_to_list(PASSINDEXTYPE cur) {
    if (!sorted_) sort();
    if (!std::binary_search(data_.begin(), data_.end(), cur)) {
      data_.insert(std::lower_bound(data_.begin(), data_.end(),
                                   cur), cur);
      Container::set_is_changed(true);
    }
  }
  ListLikeCLASSNAMEContainer(Model *m, std::string name):
    CLASSNAMEContainer(m,name), sorted_(false){
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
  void apply(const CLASSNAMEModifier *sm) const {
    sm->apply_indexes(get_model(), data_);
  }
  void apply(const CLASSNAMEDerivativeModifier *sm,
             DerivativeAccumulator &da) const {
    sm->apply_indexes(get_model(), data_, da);
  }
  double evaluate(const CLASSNAMEScore *s,
                  DerivativeAccumulator *da) const {
    return s->evaluate_indexes(get_model(), data_, da);
  }
  double evaluate_if_good(const CLASSNAMEScore *s,
                          DerivativeAccumulator *da,
                          double max) const {
    return s->evaluate_if_good_indexes(get_model(), data_, da, max);
  }
  ParticlesTemp get_all_possible_particles() const {
    return IMP::internal::flatten(IMP::internal::get_particle(get_model(),
                                                              data_));
  }
  bool get_contains_FUNCTIONNAME(ARGUMENTTYPE p) const {
    if (!sorted_) sort();
    INDEXTYPE it= IMP::internal::get_index(p);
    return std::binary_search(data_.begin(), data_.end(), it);
  }
  IMP_OBJECT(ListLikeCLASSNAMEContainer);

  PLURALINDEXTYPE get_indexes() const {
    return data_;
  }
  bool get_provides_access() const {return true;}
  const PLURALINDEXTYPE& get_access() const {
    return data_;
  }

  typedef PLURALINDEXTYPE::const_iterator const_iterator;
  const_iterator begin() const {
    return data_.begin();
  }
  const_iterator end() const {
    return data_.end();
  }
};


IMP_END_INTERNAL_NAMESPACE

#define IMP_LISTLIKE_HEADERNAME_CONTAINER(Name)                         \
  public:                                                               \
  ParticlesTemp get_all_possible_particles() const;                     \
  ParticlesTemp get_input_particles() const;                            \
  ContainersTemp get_input_containers() const;                          \
  void do_before_evaluate();                                            \
  PLURALINDEXTYPE get_all_possible_indexes() const;                     \
  IMP_OBJECT(Name)


#endif  /* IMPKERNEL_INTERNAL_LIST_LIKE_HEADERNAME_CONTAINER_H */
