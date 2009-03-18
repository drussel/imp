/**
 *  \file SingletonsRestraint.cpp
 *  \brief Apply a SingletonScore function to a container of Particle*s .
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/SingletonsRestraint.h"
#include <IMP/internal/container_helpers.h>
#include "IMP/core/ListSingletonContainer.h"

#include <IMP/SingletonScore.h>
#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

SingletonsRestraint
::SingletonsRestraint(SingletonScore *ss,
                      const Particles &pc):
  ss_(ss),
  pc_(new ListSingletonContainer(pc))
{
}

SingletonsRestraint
::SingletonsRestraint(SingletonScore *ss,
                      SingletonContainer *pc): ss_(ss), pc_(pc) {

}

Float SingletonsRestraint::evaluate(DerivativeAccumulator *accum)
{

  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  double score=0;


  for (SingletonContainer::ParticleIterator
         it= pc_->particles_begin();
       it != pc_->particles_end(); ++it) {
    double lscore= IMP::internal::ContainerTraits<Particle>
      ::evaluate(ss_, *it, accum);
    IMP_LOG(VERBOSE, IMP::internal::streamable(*it).get_name()
            << " has score " << lscore << std::endl);
    score+=lscore;
  }

  return score;
}


ParticlesList SingletonsRestraint::get_interacting_particles() const
{
  if (!IMP::internal::ContainerTraits<Particle>::is_singleton) {
    ParticlesList ret;
    for (SingletonContainer::ParticleIterator it
           = pc_->particles_begin();
         it != pc_->particles_end(); ++it) {
      ret.push_back(IMP::internal
                    ::ContainerTraits<Particle>::create_set(*it));
    }
    return ret;
  } else {
    return ParticlesList();
  }
}

namespace {
  ListSingletonContainer *
  check_methods(SingletonContainer *pc, std::string str) {
    ListSingletonContainer *ret
      = dynamic_cast<ListSingletonContainer*>(pc);
    if (! ret) {
      std::ostringstream oss;
      oss << "Method SingletonsRestraint::" << str
          << " can only be called if the SingletonContainer "
          << " is a ListSingletonContainer.";
      throw InvalidStateException(oss.str().c_str());
    }
    return ret;
  }
}

void SingletonsRestraint::add_particles(const Particles &ps) {
  ListSingletonContainer *pc= check_methods(pc_.get(), "add_particles");
  pc->add_particles(ps);
}

void SingletonsRestraint::set_particles(const Particles &ps){
  ListSingletonContainer *pc= check_methods(pc_.get(), "add_particles");
  pc->set_particles(ps);
}

void SingletonsRestraint::add_particle(Particle* v){
  ListSingletonContainer *pc= check_methods(pc_.get(), "add_particles");
  pc->add_particle(v);
}

void SingletonsRestraint::clear_particles() {
  ListSingletonContainer *pc= check_methods(pc_.get(), "add_particles");
  pc->clear_particles();
}


void SingletonsRestraint::show(std::ostream& out) const
{
  out << "ContainerRestraint with score function ";
  ss_->show(out);
  out << " and container ";
  pc_->show(out);
  out << std::endl;
}

IMPCORE_END_NAMESPACE
