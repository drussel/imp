/**
 *  \file example/ExampleRestraint.cpp
 *  \brief Restrain a list of particle pairs.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/example/ExampleRestraint.h>
#include <IMP/PairScore.h>
#include <IMP/log.h>

IMPEXAMPLE_BEGIN_NAMESPACE

ExampleRestraint::ExampleRestraint(PairScore* score_func,
                                   PairContainer *pc) : pc_(pc),
                                          f_(score_func) {
  pc_->set_was_used(true);
  f_->set_was_used(true);
}

double
ExampleRestraint::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  double score=0;
  for (PairContainer::ParticlePairIterator
       it= pc_->particle_pairs_begin();
       it != pc_->particle_pairs_end(); ++it) {
    score += f_->evaluate(*it, accum);
  }

  return score;
}

/* Return a list of interacting sets. The PairScore defines
   the interactions so defer to it.*/
ParticlesList ExampleRestraint::get_interacting_particles() const
{
  ParticlesList ret;
  for (PairContainer::ParticlePairIterator it
       = pc_->particle_pairs_begin();
       it != pc_->particle_pairs_end(); ++it) {
    ParticlePair pp= *it;
    ParticlesList all=f_->get_interacting_particles(ParticlePair(pp[0],
                                                                 pp[1]));
    ret.insert(ret.end(), all.begin(), all.end());
  }
  return ret;
}

/* We also need to know which particles are used (as some are
   used, but don't create interactions). */
ParticlesTemp ExampleRestraint::get_input_particles() const
{
  ParticlesTemp ret;
  for (PairContainer::ParticlePairIterator it
       = pc_->particle_pairs_begin();
       it != pc_->particle_pairs_end(); ++it) {
    ParticlePair pp= *it;
    ParticlesTemp t= f_->get_input_particles(pp);
    ret.insert(ret.end(), t.begin(), t.end());
  }
  return ret;
}

ContainersTemp ExampleRestraint::get_input_containers() const
{
  return ContainersTemp(1, pc_);
}

void ExampleRestraint::do_show(std::ostream& out) const
{
  out << "function " << *f_ << std::endl;
  out << "container " << *pc_ << std::endl;
}

IMPEXAMPLE_END_NAMESPACE
