/**
 *  \file core/generic.h    \brief Various important functionality
 *                                       for implementing decorators.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_INTERNAL_GENERIC_H
#define IMPCORE_INTERNAL_GENERIC_H

#include "../core_config.h"
#include <IMP/core/internal/singleton_helpers.h>
#include <IMP/core/internal/pair_helpers.h>
#include <IMP/core/internal/triplet_helpers.h>
#include <IMP/core/internal/quad_helpers.h>
#include <IMP/base_types.h>
#include <IMP/Object.h>
#include <IMP/Constraint.h>
#include <IMP/Restraint.h>
#include <IMP/PairFilter.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE
/** When programming in C++, you can use CoreRestraint instead
    of a SingletonsRestraint, PairsRestraint, etc. The result is
    somewhat faster (20% or so).
*/
template <class Score, class Container>
class ContainerRestraint :
#if defined(SWIG) || defined(IMP_DOXYGEN)
public DecomposableRestraint
#else
public IMP::internal::SimpleRestraintParentTraits<Score>::SimplesRestraint
#endif
{
  IMP::OwnerPointer<Score> ss_;
  IMP::OwnerPointer<Container> pc_;
  IMP::OwnerPointer<IMP::Container> ac_, rc_;
  mutable double score_;
  BOOST_STATIC_ASSERT(!(boost::is_same<Score, SingletonScore>::value));
  BOOST_STATIC_ASSERT(!(boost::is_same<Score, PairScore>::value));
  BOOST_STATIC_ASSERT(!(boost::is_same<Score, TripletScore>::value));
  BOOST_STATIC_ASSERT(!(boost::is_same<Score, QuadScore>::value));
  BOOST_STATIC_ASSERT(!(boost::is_same<Container, SingletonContainer>::value));
  BOOST_STATIC_ASSERT(!(boost::is_same<Container, PairContainer>::value));
  BOOST_STATIC_ASSERT(!(boost::is_same<Container, TripletContainer>::value));
  BOOST_STATIC_ASSERT(!(boost::is_same<Container, QuadContainer>::value));

public:
  ContainerRestraint(Score *ss,
                     Container *pc,
                     std::string name="GroupnamesRestraint %1%");

  IMP_RESTRAINT(ContainerRestraint);

  //! Get the container used to store Particles
  typename Container::ContainedTypes
  get_arguments() const {
    return pc_->get();
  }

  Score* get_score() const {
    return ss_;
  }

  Restraints do_create_decomposition() const;
  Restraints do_create_current_decomposition() const;
};



/** Helper to create a ContainerRestraint without specifying the types. Make
    sure the score and container passed have their real type, not Container
    or PairScore.
    \relatesalso ContainerRestraint
*/
template <class Score, class Container>
inline Restraint *create_restraint(Score *s, Container*c,
                            std::string name=std::string()) {
  if (name==std::string()) {
    name= std::string("Restraint on ") + s->get_name()+ " and "+c->get_name();
  }
  return new ContainerRestraint<Score, Container>(s, c, name);
}









/** Create a constraint tied to particular modifiers and contains. This
    functionality, which is only available in C++ can result in faster
    evaluates.
*/
template <class Before, class After, class Container>
class ContainerConstraint : public Constraint
{
  IMP::OwnerPointer<Before> f_;
  IMP::OwnerPointer<After> af_;
  IMP::OwnerPointer<Container> c_;
public:
  ContainerConstraint(Before *before,
                      After *after, Container *c,
                       std::string name="GroupnameConstraint %1%");

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(After* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(Before* f) {
    f_=f;
  }

  ScoreStates create_decomposition() const;

  IMP_CONSTRAINT(ContainerConstraint);
};


/** \relatesalso ContainerConstraint

    Helper to create a ContainerConstraint.
 */
template <class Before, class After, class Container>
inline Constraint *create_constraint(Before *b, After *a, Container *c,
                              std::string name=std::string()) {
  if (name==std::string()) {
    name= std::string("Constraint on ") + c->get_name();
    if (b) name+= " and  "+b->get_name();
    if (a) name+= " and " +a->get_name();
  }
  return new ContainerConstraint<Before, After, Container>(b, a, c,
                                                           name);
}



/** A templated version of InContainerPairFilter.
 */
template <class Container>
class GenericInContainerPairFilter: public PairFilter {
  Pointer<Container> c_;
public:
  GenericInContainerPairFilter( Container* c,
                                std::string name):
      PairFilter(name),
      c_(c){}
  IMP_PAIR_FILTER(GenericInContainerPairFilter);
};

template <class Container>
inline GenericInContainerPairFilter<Container>*
create_in_container_filter(Container *c, std::string name=std::string()) {
  if (name==std::string()) {
    name= std::string("InContainer ") + c->get_name();
  }
  return new GenericInContainerPairFilter<Container>(c, name);
}


IMPCORE_END_INTERNAL_NAMESPACE


#include "generics_impl.h"

#endif  /* IMPCORE_INTERNAL_GENERIC_H */