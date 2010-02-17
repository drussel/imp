/**
 *  \file ChecksScoreState.cpp
 *  \brief Keep track of the maximumimum change of a set of attributes.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */


#include <IMP/core/ChecksScoreState.h>
#include <IMP/exception.h>
#include <IMP/random.h>
#include <boost/random/uniform_real.hpp>

IMPCORE_BEGIN_NAMESPACE



ChecksScoreState::ChecksScoreState(double prob):
  ScoreState("ChecksScoreState %1%"),
  probability_(prob), count_(0)
{
  IMP_USAGE_CHECK(prob >=0 && prob <=1,
                  "The probability must be a probability.");
}


void ChecksScoreState::do_before_evaluate()
{
  ::boost::uniform_real<> rand(0,1);
  if (rand(random_number_generator) < probability_) {
    set_check_level(USAGE_AND_INTERNAL);
    ++ count_;
  } else {
    set_check_level(NONE);
  }
}
void ChecksScoreState::do_after_evaluate(DerivativeAccumulator*){
}

ParticlesList ChecksScoreState::get_interacting_particles() const {
  return ParticlesList();
}


ContainersTemp ChecksScoreState::get_input_containers() const {
  return ContainersTemp();
}

ContainersTemp ChecksScoreState::get_output_containers() const {
  return ContainersTemp();
}

ParticlesTemp ChecksScoreState::get_input_particles() const {
  return ParticlesTemp();
}

ParticlesTemp ChecksScoreState::get_output_particles() const {
  return ParticlesTemp();
}


void ChecksScoreState::do_show(std::ostream &out) const
{
  out << "probability " << probability_ << std::endl;
}



IMPCORE_END_NAMESPACE
