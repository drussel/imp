/**
 *  \file Model.cpp \brief Storage of a model, its restraints,
 *                         constraints and particles.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/Model.h"
#include "IMP/Particle.h"
#include "IMP/log.h"
#include "IMP/Restraint.h"
#include "IMP/DerivativeAccumulator.h"
#include "IMP/ScoreState.h"

IMP_BEGIN_NAMESPACE

//! Constructor
Model::Model()
{
  iteration_ = 0;
  next_particle_index_=0;
}


//! Destructor
Model::~Model()
{
  IMP_CHECK_OBJECT(this);
  for (ParticleIterator it= particles_begin();
       it != particles_end(); ++it) {
    (*it)->model_ = NULL;
    internal::unref(*it);
  }
}

IMP_LIST_IMPL(Model, Restraint, restraint, Restraint*,
              Restraints,
              {obj->set_model(this);},,
              {obj->set_model(NULL);});

IMP_LIST_IMPL(Model, ScoreState, score_state, ScoreState*,
              ScoreStates,
              {obj->set_model(this);},,
              {obj->set_model(NULL);});


FloatRange Model::get_range(FloatKey k) const {
  IMP_CHECK_OBJECT(this);
  if (ranges_.find(k) != ranges_.end()) {
    return ranges_.find(k)->second;
  } else {
    FloatRange r(std::numeric_limits<Float>::max(),
                -std::numeric_limits<Float>::max());
    for (ParticleConstIterator it= particles_begin();
         it != particles_end(); ++it) {
      if ((*it)->has_attribute(k)) {
        Float v= (*it)->get_value(k);
        r.first = std::min(r.first, v);
        r.second= std::max(r.second, v);
      }
    }
    IMP_LOG(TERSE, "Range for attribute " << k << " is " << r.first
            << " to " << r.second << std::endl);
    return r;
  }
}

Float Model::evaluate(bool calc_derivs)
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(this);
  IMP_LOG(TERSE,
          "Begin Model::evaluate" << std::endl);
  // If calcualting derivatives, first set all derivatives to zero
  if (calc_derivs) {
    for (ParticleIterator pit = particles_begin();
         pit != particles_end(); ++pit) {
      (*pit)->zero_derivatives();
    }

  }

  IMP_LOG(TERSE,
          "Begin update ScoreStates " << std::endl);
  for (ScoreStateIterator it = score_states_begin(); it != score_states_end();
       ++it) {
    IMP_CHECK_OBJECT(*it);
    (*it)->before_evaluate(iteration_);
    IMP_LOG(VERBOSE, "." << std::flush);
  }
  IMP_LOG(TERSE, "End update ScoreStates." << std::endl);

  // evaluate all of the active restraints to get score (and derivatives)
  // for current state of the model
  double score = 0.0;
  DerivativeAccumulator accum;
  DerivativeAccumulator *accpt = (calc_derivs ? &accum : NULL);

  IMP_LOG(TERSE,
          "Begin evaluate restraints "
          << (calc_derivs?"with derivatives":"without derivatives")
          << std::endl);

  for (RestraintIterator it = restraints_begin();
       it != restraints_end(); ++it) {
    IMP_CHECK_OBJECT(*it);
    IMP_LOG(TERSE, "Evaluate restraint "
            << std::endl << **it);
    Float tscore=0;
    if ((*it)->get_is_active()) {
      tscore = (*it)->evaluate(accpt);
    }
    IMP_LOG(TERSE, "Restraint score is " << tscore << std::endl);
    score+= tscore;
  }

  IMP_LOG(TERSE, "End evaluate restraints." << std::endl);

  IMP_LOG(TERSE,
          "Begin after_evaluate of ScoreStates " << std::endl);

  for (ScoreStateIterator it = score_states_begin(); it != score_states_end();
       ++it) {
    IMP_CHECK_OBJECT(*it);
    (*it)->after_evaluate(iteration_, accpt);
    IMP_LOG(VERBOSE, "." << std::flush);
  }

  IMP_LOG(TERSE, "End after_evaluate of ScoreStates." << std::endl);

  IMP_LOG(TERSE, "End Model::evaluate. Final score: " << score << std::endl);

  ++iteration_;
  return score;
}

void Model::show(std::ostream& out) const
{
  out << std::endl << std::endl;
  out << "Model:" << std::endl;

  get_version_info().show(out);

  out << get_number_of_particles() << " particles" << std::endl;
  out << get_number_of_restraints() << " restraints" << std::endl;
  out << get_number_of_score_states() << " score states" << std::endl;

  out << std::endl;
  IMP_CHECK_OBJECT(this);
}

IMP_END_NAMESPACE
