/**
 *  \file MonteCarlo.cpp  \brief Simple Monte Carlo optimizer.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/MonteCarlo.h>

#include <IMP/random.h>
#include <IMP/Model.h>
#include <IMP/ConfigurationSet.h>

#include <limits>
#include <cmath>
#include <boost/random/uniform_real.hpp>

IMPCORE_BEGIN_NAMESPACE

Mover::Mover(std::string name):Object(name) {}

IMP_LIST_IMPL(MonteCarlo, Mover, mover, Mover*, Movers,
              {obj->set_optimizer(this);
                obj->set_was_used(true);
              },{},{});

MonteCarlo::MonteCarlo(Model *m): Optimizer(m, "MonteCarlo"),
                                  temp_(1),
                                  probability_(1),
                                  num_local_steps_(50),
                                  stat_forward_steps_taken_(0),
                                  stat_upward_steps_taken_(0),
                                  stat_num_failures_(0),
                                  return_best_(true) {}


Float MonteCarlo::do_optimize(unsigned int max_steps)
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(this);
  if (get_number_of_movers() ==0) {
    IMP_THROW("Running MonteCarlo without providing any"
              << " movers isn't very useful.",
              ValueException);
  }
  IMP::internal::OwnerPointer<Configuration> best_state
    = new Configuration(get_model());

  int failures=0;
  if (cg_) {
    //cg_->set_score_threshold(get_score_threshold());
    IMP_CHECK_OBJECT(cg_.get());
    IMP_USAGE_CHECK(cg_->get_model() == get_model(),
               "The model used by the local optimizer does not match "\
              " that used by the montecarlo optimizer");
  }
  update_states();
  double prior_energy =get_model()->evaluate(false);
  double best_energy= prior_energy;
  //if (prior_energy < get_score_threshold()) return prior_energy;
  if (get_stop_on_good_score() && get_model()->get_has_good_score()) {
    return prior_energy;
  }

  IMP_LOG(TERSE, "MC Initial energy is " << prior_energy << std::endl);

  ::boost::uniform_real<> rand(0,1);
  for (unsigned int i=0; i< max_steps; ++i) {
    //make it a parameter
    for (MoverIterator it = movers_begin(); it != movers_end(); ++it) {
      IMP_LOG(VERBOSE, "Moving using " << (*it)->get_name() << std::endl);
      IMP_CHECK_OBJECT(*it);
      (*it)->propose_move(probability_);
      IMP_LOG(VERBOSE, "end\n");
    }
    Float next_energy;
    try {
      if (cg_ && num_local_steps_!= 0) {
        IMP_LOG(TERSE,
                "MC Performing local optimization from "
                << get_model()->evaluate(false) << std::endl);
        CreateLogContext clc("mc local optimization");
        {
          IMP_CHECK_OBJECT(cg_.get());

          // if incremental, turn off non-dirty particles
          if (get_model()->get_is_incremental()) {
            bool has_changed=false;
            get_model()->update();
            SaveOptimizeds si(ParticlesTemp(get_model()->particles_begin(),
                                            get_model()->particles_end()));
            for (Model::ParticleIterator it= get_model()->particles_begin();
                 it != get_model()->particles_end(); ++it) {
              if (!(*it)->get_is_changed()) {
                for (Particle::FloatKeyIterator oit= (*it)->float_keys_begin();
                     oit != (*it)->float_keys_end(); ++oit) {
                  (*it)->set_is_optimized(*oit, false);
                }
              } else {
                has_changed=true;
                IMP_LOG(VERBOSE, "Particle " << (*it)->get_name()
                        << " was changed " << **it << std::endl);
              }
            }
            if (has_changed) {
              next_energy =cg_->optimize(num_local_steps_);
            } else {
              IMP_LOG(TERSE, "empty move" << std::endl);
              next_energy=prior_energy;
            }
          } else {
            next_energy =cg_->optimize(num_local_steps_);
            IMP_IF_CHECK(USAGE) {
              double me= get_model()->evaluate(false);
              if(0) std::cout << me;
              IMP_USAGE_CHECK((next_energy-me) < .01*(next_energy+me)+.01,
                              "Energies don't match after local opt. "
                              << "Got " << me << " but computed "
                              << next_energy << (next_energy-me)
                              << .01*(next_energy+me)+.01);
            }
          }
      }
      IMP_LOG(TERSE, "To energy " << next_energy << " equals "
              << get_model()->evaluate(false)
              << " done "<< std::endl);
      } else {
        next_energy =  get_model()->evaluate(false);
      }
    } catch (const ModelException &e) {
      // make sure the move is rejected if the model gets in
          // an invalid state
          ++failures;
          next_energy= std::numeric_limits<double>::infinity();
        }
    bool accept=false;
    if  (next_energy < prior_energy) {
      accept=true;
    } else {
      Float diff= next_energy- prior_energy;
      Float e= std::exp(-diff/temp_);
      Float r= rand(random_number_generator);
      IMP_LOG(VERBOSE, diff << " " << temp_ << " " << e << " " << r
              << std::endl);
      if (e > r) {
        accept=true;
        ++stat_upward_steps_taken_;
      }
    }
    IMP_LOG(TERSE,  "MC Prior energy is " << prior_energy
            << " and next is " << next_energy << " "
            << "(" << get_score_threshold() << ") ");
    if (accept) {
      IMP_LOG(TERSE,  " accept" << std::endl);
      ++stat_forward_steps_taken_;
      prior_energy= next_energy;
      if (return_best_ && next_energy < best_energy) {
        best_energy= next_energy;
        IMP_LOG(TERSE, "Saving state with energy " << best_energy << std::endl);
        //std::cout << "best energy is first " << best_energy << std::endl;
        best_state= new Configuration(get_model());
        //best_state->load_configuration();
        /*IMP_IF_CHECK(USAGE) {
          std::cout << "best energy is " << best_energy << std::endl;
          best_state->load_configuration();
          std::cout << "best energy is now " << best_energy << std::endl;
          }*/
      }
      update_states();
      if (get_stop_on_good_score() && get_model()->get_has_good_score()) {
        break;
      }
    } else {
      IMP_LOG(TERSE,  " reject" << std::endl);
      for (MoverIterator it = movers_begin(); it != movers_end(); ++it) {
        (*it)->reset_move();
      }
      ++stat_num_failures_;
    }
  }


  IMP_LOG(TERSE, "MC Final energy is " << prior_energy
          << " after " << failures << " failures" << std::endl);
  if (return_best_) {
    //std::cout << "Final score is " << get_model()->evaluate(false)
    //<< std::endl;
    best_state->load_configuration();
    IMP_LOG(TERSE, "MC Returning energy " << best_energy << std::endl);
    IMP_IF_CHECK(USAGE) {
      IMP_CHECK_CODE(double e= get_model()->evaluate(false));
      IMP_LOG(TERSE, "MC Got " << e << std::endl);
      IMP_INTERNAL_CHECK(std::abs(best_energy - e)
                         < .01+.1* std::abs(best_energy +e),
                         "Energies do not match "
                         << best_energy << " vs " << e << std::endl);
    }
    return best_energy;
  } else {
    double ret= get_model()->evaluate(false); //force coordinate update
    IMP_INTERNAL_CHECK(ret < std::numeric_limits<double>::max(),
                       "Don't return rejected conformation");
    return ret;
  }
}


void MonteCarlo::set_local_optimizer(Optimizer* cg)
{
  cg_= cg;
  cg_->set_model(get_model());
}

void MonteCarlo::do_show(std::ostream &out) const
{
  out << "forward steps " << stat_forward_steps_taken_
      << " -" << stat_num_failures_ << std::endl;
}

IMPCORE_END_NAMESPACE
