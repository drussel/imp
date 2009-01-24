/**
 *  \file PairsRestraint.cpp
 *  \brief Apply a PairScore function to a container of ParticlePairs .
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/PairsRestraint.h"
#include <IMP/internal/container_helpers.h>
#include "IMP/core/ListPairContainer.h"

#include <IMP/PairScore.h>
#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

PairsRestraint
::PairsRestraint(PairScore *ss,
                      const ParticlePairs &pc):
  ss_(ss),
  pc_(new ListPairContainer(pc))
{
}

PairsRestraint
::PairsRestraint(PairScore *ss,
                      PairContainer *pc): ss_(ss), pc_(pc) {

}


PairsRestraint::~PairsRestraint()
{
}

Float PairsRestraint::evaluate(DerivativeAccumulator *accum)
{

  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  Float score=0;


  for (PairContainer::ParticlePairIterator
         it= pc_->particle_pairs_begin();
       it != pc_->particle_pairs_end(); ++it) {
    score += IMP::internal::ContainerTraits<ParticlePair>
      ::evaluate(ss_, *it, accum);
  }

  return score;
}


ParticlesList PairsRestraint::get_interacting_particles() const
{
  if (!IMP::internal::ContainerTraits<ParticlePair>::is_singleton) {
    ParticlesList ret;
    for (PairContainer::ParticlePairIterator it
           = pc_->particle_pairs_begin();
         it != pc_->particle_pairs_end(); ++it) {
      ret.push_back(IMP::internal
                    ::ContainerTraits<ParticlePair>::create_set(*it));
    }
    return ret;
  } else {
    return ParticlesList();
  }
}

namespace {
  ListPairContainer *
  check_methods(PairContainer *pc, std::string str) {
    ListPairContainer *ret
      = dynamic_cast<ListPairContainer*>(pc);
    if (! ret) {
      std::ostringstream oss;
      oss << "Method PairsRestraint::" << str
          << " can only be called if the PairContainer "
          << " is a ListPairContainer.";
      throw InvalidStateException(oss.str().c_str());
    }
    return ret;
  }
}

void PairsRestraint::add_particle_pairs(const ParticlePairs &ps) {
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->add_particle_pairs(ps);
}

void PairsRestraint::set_particle_pairs(const ParticlePairs &ps){
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->set_particle_pairs(ps);
}

void PairsRestraint::add_particle_pair(ParticlePair v){
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->add_particle_pair(v);
}

void PairsRestraint::clear_particle_pairs() {
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->clear_particle_pairs();
}


void PairsRestraint::show(std::ostream& out) const
{
  out << "ContainerRestraint with score function ";
  ss_->show(out);
  out << " and container ";
  pc_->show(out);
  out << std::endl;
}

IMPCORE_END_NAMESPACE
