/**
 *  \file container/internal/generic_impl.h
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_GENERIC_IMPL_H
#define IMPCONTAINER_GENERIC_IMPL_H
#include <IMP/core/generic.h>

IMPCONTAINER_BEGIN_NAMESPACE

template <class Score, class C>
ContainerRestraint<Score, C>
::ContainerRestraint(Score *ss,
                     C *pc,
                     std::string name):
  IMP::internal::SimpleRestraintParentTraits<Score>::SimplesRestraint(name),
  ss_(ss), pc_(pc) {

}
template <class Score, class C>
double ContainerRestraint<Score, C>
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);
  score_= pc_->template template_evaluate<Score>(ss_.get(), accum);
  return score_;
}

template <class Score, class C>
double ContainerRestraint<Score, C>
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);
  IMP_LOG(VERBOSE, "Scores are " << score_);
  score_+=pc_->template template_evaluate_change<Score>(ss_.get(), accum);
  // compute the base for the added ones
  IMP_LOG(VERBOSE, " " << score_);
  // could be better...
  score_ +=pc_->C::get_added_container()
    ->evaluate_prechange(ss_, accum);
  IMP_LOG(VERBOSE," " << score_);
  if (accum) {
    DerivativeAccumulator nda(*accum, -1);
    score_ -=pc_->C::get_removed_container()
      ->evaluate_prechange(ss_, &nda);
  } else {
    score_ -=pc_->get_removed_container()
      ->evaluate_prechange(ss_, NULL);
  }
  IMP_LOG(VERBOSE," " << score_ << std::endl);
  return score_;
}

template <class Score, class C>
ParticlesTemp ContainerRestraint<Score, C>::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret
    = IMP::internal::get_input_particles(ss_.get(),
                                         pc_->get_contained_particles());
  return ret;
}

template <class Score, class C>
ContainersTemp ContainerRestraint<Score, C>
::get_input_containers() const
{
  ContainersTemp ret
    = IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_contained_particles());
  ret.push_back(pc_);
  return ret;
}


template <class Score, class C>
Restraints ContainerRestraint<Score, C>::get_decomposition() const {
  Restraints ret(pc_->get_number());
  for (unsigned int i=0; i< ret.size(); ++i) {
    ret[i]= core::create_restraint(ss_.get(), pc_->get(i));
  }
  return ret;
}

template <class Score, class C>
void ContainerRestraint<Score, C>::do_show(std::ostream& out) const
{
  out << "score " << *ss_ << std::endl;
  out << "container " << *pc_ << std::endl;
}




template <class Score, class C>
inline Restraint *create_restraint(Pointer<Score> s, Pointer<C> c,
                            std::string name=std::string()) {
  return create_restraint<Score,C>(s.get(), c.get(), name);
}
template <class Score, class C>
inline Restraint *create_restraint(Score* s, Pointer<C> c,
                            std::string name=std::string()) {
  return create_restraint<Score,C>(s, c.get(), name);
}
template <class Score, class C>
inline Restraint *create_restraint(Pointer<Score> s, C* c,
                            std::string name=std::string()) {
  return create_restraint<Score,C>(s.get(), c, name);
}











template <class C, class Before, class After>
ContainerConstraint<C, Before, After>::ContainerConstraint(C *c,
                                         Before *before,
                                         After *after,
                                         std::string name):
  Constraint(name), c_(c) {
  if (before) f_=before;
  if (after) af_=after;
}

template <class C, class Before, class After>
void ContainerConstraint<C, Before, After>::do_update_attributes()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_LOG(TERSE, "Begin ContainerConstraint::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP_CHECK_OBJECT(c_);
  if (c_->get_provides_access()) {
    f_->Before::apply(c_->get_access());
  } else {
    c_->template_apply(f_.get());
  }
  IMP_LOG(TERSE, "End ContainerConstraint::update" << std::endl);
}

template <class C, class Before, class After>
void ContainerConstraint<C, Before, After>
::do_update_derivatives(DerivativeAccumulator *da)
{
  IMP_OBJECT_LOG;
  if (!af_) return;
  IMP_LOG(TERSE, "Begin ContainerConstraint::after_evaluate" << std::endl);
  IMP_CHECK_OBJECT(af_);
  IMP_CHECK_OBJECT(c_);
  if (c_->get_provides_access()) {
    af_->After::apply(c_->get_access(), *da);
  } else {
    c_->template_apply(af_.get(), *da);
  }
  IMP_LOG(TERSE, "End ContainerConstraint::after_evaluate" << std::endl);
}

template <class C, class Before, class After>
ContainersTemp ContainerConstraint<C, Before, After>
::get_input_containers() const {
  return ContainersTemp(1, c_);
}

template <class C, class Before, class After>
ContainersTemp ContainerConstraint<C, Before, After>
::get_output_containers() const {
  return ContainersTemp();
}

template <class C, class Before, class After>
ParticlesTemp ContainerConstraint<C, Before, After>
::get_input_particles() const {
  ParticlesTemp ret;
  if (f_) {
    ret= IMP::internal::get_input_particles(f_.get(),
                                            c_->get_contained_particles());
    ParticlesTemp o= IMP::internal::get_output_particles(f_.get(),
                                            c_->get_contained_particles());
    ret.insert(ret.end(), o.begin(), o.end());
    IMP_IF_CHECK(USAGE) {
      if (af_) {
        ParticlesTemp oret= IMP::internal::get_output_particles(af_.get(),
                                               c_->get_contained_particles());
        std::sort(ret.begin(), ret.end());
        std::sort(oret.begin(), oret.end());
        ParticlesTemp t;
        std::set_union(ret.begin(), ret.end(), oret.begin(), oret.end(),
                       std::back_inserter(t));
        IMP_USAGE_CHECK(t.size() == ret.size(), "The particles written by "
                        << " the after modifier in " << get_name()
                        << " must be a subset of those read by the before "
                        << "modifier. Before: " << Particles(ret)
                        << " and after " << Particles(oret));
      }
    }
  } else {
    ret= IMP::internal::get_output_particles(af_.get(),
                                          c_->get_contained_particles());
  }
  return ret;
}

template <class C, class Before, class After>
ParticlesTemp ContainerConstraint<C, Before, After>
::get_output_particles() const {
  ParticlesTemp ret;
  if (f_) {
    ret= IMP::internal::get_output_particles(f_.get(),
                                       c_->get_contained_particles());
    IMP_IF_CHECK(USAGE) {
      if (af_) {
        ParticlesTemp oret= IMP::internal::get_input_particles(af_.get(),
                                               c_->get_contained_particles());
        ParticlesTemp iret=IMP::internal::get_input_particles(f_.get(),
                                               c_->get_contained_particles());
        iret.insert(iret.end(), ret.begin(), ret.end());
        std::sort(iret.begin(), iret.end());
        std::sort(oret.begin(), oret.end());
        ParticlesTemp t;
        std::set_union(iret.begin(), iret.end(), oret.begin(), oret.end(),
                       std::back_inserter(t));
        IMP_USAGE_CHECK(t.size() == iret.size(), "The particles read by "
                      << " the after modifier in " << get_name() << " must "
                        << "be a subset of those written by the before "
                        << "modifier.");
      }
    }
  } else {
    ret= IMP::internal::get_input_particles(af_.get(),
                                           c_->get_contained_particles());
  }
  return ret;
}

template <class C, class Before, class After>
void ContainerConstraint<C, Before, After>
::do_show(std::ostream &out) const {
  out << "on " << *c_ << std::endl;
  if (f_) out << "before " << *f_ << std::endl;
  if (af_) out << "after " << *af_ << std::endl;
}



template <class C, class Before, class After>
inline Constraint *create_constraint(Pointer<C> c, Pointer<Before> b,
                              Pointer<After> a,
                              std::string name=std::string()) {
  return create_constraint<C, Before, After>(c, b, a, name);
}
template <class C, class Before, class After>
inline Constraint *create_constraint(C* c, Pointer<Before> b,
                              Pointer<After> a,
                              std::string name=std::string()) {
  return create_constraint<C, Before, After>(c, b, a, name);
}
template <class C, class Before, class After>
inline Constraint *create_constraint(Pointer<C> c, Before* b,
                              Pointer<After> a,
                              std::string name=std::string()) {
  return create_constraint<C, Before, After>(c, b, a, name);
}
template <class C, class Before, class After>
inline Constraint *create_constraint(C* c, Before* b,
                              Pointer<After> a,
                              std::string name=std::string()) {
  return create_constraint<C, Before, After>(c, b, a, name);
}
template <class C, class Before, class After>
inline Constraint *create_constraint(Pointer<C> c, Pointer<Before> b,
                              After* a,
                              std::string name=std::string()) {
  return create_constraint<C, Before, After>(c, b, a, name);
}
template <class C, class Before, class After>
inline Constraint *create_constraint(C* c, Pointer<Before> b,
                              After* a,
                              std::string name=std::string()) {
  return create_constraint<C, Before, After>(c, b, a, name);
}
template <class C, class Before, class After>
inline Constraint *create_constraint(Pointer<C> c, Before* b,
                              After* a,
                              std::string name=std::string()) {
  return create_constraint<C, Before, After>(c, b, a, name);
}




template <class C>
bool GenericInContainerPairFilter<C>
::get_contains_particle_pair(const ParticlePair& p) const {
  return c_->C::get_contains_particle_pair(p);
}

template <class C>
ParticlesTemp GenericInContainerPairFilter<C>
::get_input_particles(Particle*) const {
  // not quite right
  return ParticlesTemp();
}
template <class C>
ContainersTemp GenericInContainerPairFilter<C>
::get_input_containers(Particle*) const {
  return ContainersTemp(1, c_);
}

template <class C>
void GenericInContainerPairFilter<C>
::do_show(std::ostream &out) const {
  out << "Filtering from container " << c_->get_name() << std::endl;
}

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_GENERIC_IMPL_H */
