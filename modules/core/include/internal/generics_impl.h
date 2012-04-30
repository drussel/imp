/**
 *  \file container/internal/generic_impl.h
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_INTERNAL_GENERICS_IMPL_H
#define IMPCORE_INTERNAL_GENERICS_IMPL_H
#include <IMP/generic.h>
#include <IMP/internal/create_decomposition.h>
IMPCORE_BEGIN_INTERNAL_NAMESPACE

template <class Score, class C>
ContainerRestraint<Score, C>
::ContainerRestraint(Score *ss,
                     C *pc,
                     std::string name):
  Restraint(pc->get_model(), name),
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
ParticlesTemp ContainerRestraint<Score, C>::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret
    = IMP::internal::get_input_particles(ss_.get(),
                                         pc_->get_all_possible_particles());
  return ret;
}

template <class Score, class C>
ContainersTemp ContainerRestraint<Score, C>
::get_input_containers() const
{
  ContainersTemp ret
    = IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_all_possible_particles());
  ret.push_back(pc_);
  return ret;
}


template <class Score, class C>
Restraints ContainerRestraint<Score, C>::do_create_decomposition() const {
  return IMP::internal::create_decomposition(get_model(),
                                             ss_.get(),
                                             pc_.get(),
                                             get_name());
}

template <class Score, class C>
Restraints
ContainerRestraint<Score, C>::do_create_current_decomposition() const {
  return IMP::internal::create_current_decomposition(get_model(),
                                                     ss_.get(),
                                                     pc_.get(),
                                                     get_name());
}


template <class Score, class C>
void ContainerRestraint<Score, C>::do_show(std::ostream& out) const
{
  out << "score " << *ss_ << std::endl;
  out << "container " << *pc_ << std::endl;
}










template <class Before, class After, class C>
ContainerConstraint< Before, After, C>::ContainerConstraint(
                                         Before *before,
                                         After *after,
                                         C *c,
                                         std::string name):
  Constraint(name), c_(c) {
  if (before) f_=before;
  if (after) af_=after;
}

template <class Before, class After, class C>
void ContainerConstraint<Before, After, C>::do_update_attributes()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_CHECK_OBJECT(f_);
  IMP_CHECK_OBJECT(c_);
  if (c_->get_provides_access()) {
    f_->Before::apply_indexes(get_model(), c_->get_access());
  } else {
    c_->template_apply(f_.get());
  }
}

template <class Before, class After, class C>
void ContainerConstraint<Before, After, C>
::do_update_derivatives(DerivativeAccumulator *da)
{
  IMP_OBJECT_LOG;
  if (!af_) return;
  IMP_CHECK_OBJECT(af_);
  IMP_CHECK_OBJECT(c_);
  if (c_->get_provides_access()) {
    af_->After::apply_indexes(get_model(), c_->get_access(), *da);
  } else {
    c_->template_apply(af_.get(), *da);
  }
}


template <class Before, class After, class C>
ContainersTemp ContainerConstraint<Before, After, C>
::get_input_containers() const {
  return ContainersTemp(1, c_);
}

template <class Before, class After, class C>
ContainersTemp ContainerConstraint<Before, After, C>
::get_output_containers() const {
  return ContainersTemp();
}

template <class Before, class After, class C>
ParticlesTemp ContainerConstraint<Before, After, C>
::get_input_particles() const {
  ParticlesTemp ret;
  if (f_) {
    ret= IMP::internal::get_input_particles(f_.get(),
                                            c_->get_all_possible_particles());
    ParticlesTemp o= IMP::internal::get_output_particles(f_.get(),
                                            c_->get_all_possible_particles());
    ret.insert(ret.end(), o.begin(), o.end());
    IMP_IF_CHECK(USAGE) {
      if (af_) {
        ParticlesTemp oret
            = IMP::internal::get_output_particles(af_.get(),
                                            c_->get_all_possible_particles());
        std::sort(ret.begin(), ret.end());
        std::sort(oret.begin(), oret.end());
        ParticlesTemp t;
        std::set_union(ret.begin(), ret.end(), oret.begin(), oret.end(),
                       std::back_inserter(t));
        IMP_USAGE_CHECK(t.size() == ret.size(), "The particles written by "
                        << " the after modifier in " << get_name()
                        << " must be a subset of those read by the before "
                        << "modifier. Before: " << ret
                        << " and after " << oret);
      }
    }
  } else {
    ret= IMP::internal::get_output_particles(af_.get(),
                                          c_->get_all_possible_particles());
  }
  return ret;
}

template <class Before, class After, class C>
ParticlesTemp ContainerConstraint<Before, After, C>
::get_output_particles() const {
  ParticlesTemp ret;
  if (f_) {
    ret= IMP::internal::get_output_particles(f_.get(),
                                       c_->get_all_possible_particles());
    IMP_IF_CHECK(USAGE) {
      if (af_) {
        ParticlesTemp oret
            = IMP::internal::get_input_particles(af_.get(),
                                           c_->get_all_possible_particles());
        ParticlesTemp iret
            =IMP::internal::get_input_particles(f_.get(),
                                          c_->get_all_possible_particles());
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
                                           c_->get_all_possible_particles());
  }
  return ret;
}

template <class Before, class After, class C>
void ContainerConstraint<Before, After, C>
::do_show(std::ostream &out) const {
  out << "on " << *c_ << std::endl;
  if (f_) out << "before " << *f_ << std::endl;
  if (af_) out << "after " << *af_ << std::endl;
}






template <class C>
int GenericInContainerPairFilter<C>
::get_value(const ParticlePair& p) const {
  return c_->C::get_contains_particle_pair(p)
    || c_->C::get_contains_particle_pair(ParticlePair(p[1], p[0]));
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

IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_INTERNAL_GENERICS_IMPL_H */
