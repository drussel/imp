/**
 *  \file container/generic.h    \brief Various important functionality
 *                                       for implementing decorators.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_GENERIC_H
#define IMPCONTAINER_GENERIC_H

#include "container_config.h"
#include <IMP/core/internal/singleton_helpers.h>
#include <IMP/core/internal/pair_helpers.h>
#include <IMP/core/internal/triplet_helpers.h>
#include <IMP/core/internal/quad_helpers.h>
#include <IMP/base_types.h>
#include <IMP/Object.h>
#include <IMP/Constraint.h>
#include <IMP/Restraint.h>
#include <IMP/PairFilter.h>

IMPCONTAINER_BEGIN_NAMESPACE
/** When programming in C++, you can use ContainerRestraint instead
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
public:
  ContainerRestraint(Score *ss,
                     Container *pc,
                     std::string name="GroupnamesRestraint %1%");

  IMP_RESTRAINT(ContainerRestraint);

  //! Get the container used to store Particles
  std::vector<typename Score::Argument> get_arguments() const {
    return pc_->get();
  }

  Score* get_score() const {
    return ss_;
  }

  Restraints create_decomposition() const;
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
template <class Container, class Before, class After>
class ContainerConstraint : public Constraint
{
  IMP::OwnerPointer<Before> f_;
  IMP::OwnerPointer<After> af_;
  IMP::OwnerPointer<Container> c_;
public:
  ContainerConstraint(Container *c, Before *before,
                      After *after,
                       std::string name="GroupnameConstraint %1%");

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(After* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(Before* f) {
    f_=f;
  }

  IMP_CONSTRAINT(ContainerConstraint);
};


/** \relatesalso ContainerConstraint

    Helper to create a ContainerConstraint.
 */
template <class Container, class Before, class After>
inline Constraint *create_constraint(Container *c, Before *b, After *a,
                              std::string name=std::string()) {
  if (name==std::string()) {
    name= std::string("Constraint on ") + c->get_name();
    if (b) name+= " and  "+b->get_name();
    if (a) name+= " and " +a->get_name();
  }
  return new ContainerConstraint<Container, Before, After>(c, b, a,
                                                           name);
}



/** A templated version of InContainerPairFilter.
 */
template <class Container>
class GenericInContainerPairFilter: public PairFilter {
  Pointer<Container> c_;
public:
  GenericInContainerPairFilter( Container* c): c_(c){}
  IMP_PAIR_FILTER(GenericInContainerPairFilter);
};


IMPCONTAINER_END_NAMESPACE


#include "internal/generic_impl.h"

#endif  /* IMPCONTAINER_GENERIC_H */
