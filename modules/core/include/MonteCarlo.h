/**
 *  \file MonteCarlo.h    \brief Simple Monte Carlo optimizer.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_MONTE_CARLO_H
#define IMPCORE_MONTE_CARLO_H

#include "core_config.h"
#include "Mover.h"
#include "core_macros.h"
#include <IMP/Optimizer.h>
#include <IMP/container_macros.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/algebra/vector_search.h>
#include <IMP/Configuration.h>
#include <IMP/PairFilter.h>
#include <IMP/PairScore.h>

#include <boost/random/uniform_real.hpp>

IMPCORE_BEGIN_NAMESPACE


//! A Monte Carlo optimizer.
/** The optimizer uses a set of Mover objects to propose steps. Currently
    each Mover is called at each Monte Carlo iteration. If you only want to
    use one mover at a time, use a SerialMover.
    The movers propose some modification, which is then
    accepted or rejected based on the Metropolis criterion. Optionally, a
    number of local optimization steps are taken before the MonteCarlo step
    is accepted or rejected.

    By default, the lowest score state encountered is returned.

    \see Mover
 */
class IMPCOREEXPORT MonteCarlo: public Optimizer
{
public:
  MonteCarlo(Model *m=nullptr);

  IMP_OPTIMIZER(MonteCarlo);
 public:
  /** By default, the optimizer returns the lowest score state
      found so far. If, instead, you wish to return the last accepted
      state, set return best to false.
  */
  void set_return_best(bool tf) {
    return_best_=tf;
  }

  /** \name kT
      The kT value has to be on the same scale as the differences
      in energy between good and bad states (and so the default is
      likely to not be a good choice).
      @{
  */
  void set_kt(Float t) {
    IMP_INTERNAL_CHECK(t>0, "Temperature must be positive");
    temp_=t;
  }
  Float get_kt() const {
    return temp_;
  }
  /** @} */
  /** Return the energy of last accepted state.
   */
  double get_last_accepted_energy() const {
    return last_energy_;
  }

  /** If return best is on, you can get the best energy
      found so far.*/
  double get_best_accepted_energy() const {
    IMP_USAGE_CHECK(return_best_, "Getting the best energy"
                    << " requires return best being on.");
    return best_energy_;
  }

  //! Set the probability of each move being made
  /** Make this low if the space is rough and there are many particles.
      The movers should make each individual move with this probability.
      That is, a NormalMover with 100 particles will move each particle
      with probability p.
   */
  void set_move_probability(Float p) {
    IMP_USAGE_CHECK(p > 0 && p <= 1, "Not a valid probability");
    probability_=p;
  }
  double get_move_probability() const {
    return probability_;
  }
  /** \name Statistics
      @{
   */
  //! Return how many times the optimizer has succeeded in taking a step
  unsigned int get_number_of_forward_steps() const {
    return stat_forward_steps_taken_;
  }
  //! Return how many times the optimizer has stepped to higher energy
  unsigned int get_number_of_upward_steps() const {
    return stat_upward_steps_taken_;
  }
  /** @} */

  /** Computations can be acceletating by throwing out
      the tails of the distribution of accepted moves. To
      do this, specific a maximum acceptable difference
      between the before and after scores.
  */
  void set_maximum_difference(double d) {
    max_difference_=d;
  }

  double get_maximum_difference() const {
    return max_difference_;
  }
  /** @name Movers

       The following methods are used to manipulate the list of Movers.
       Each mover is called at each optimization step, giving it a chance
       to change the current configuration.
       @{
  */
  IMP_LIST_ACTION(public, Mover, Movers, mover, movers, Mover*, Movers,
                  {obj->set_optimizer(this);
                    obj->set_was_used(true);
                  },{},{obj->set_optimizer(nullptr);});
  /** @} */


  //! Return the average number of restraints per evaluate
  double get_average_number_of_incremental_restraints() const {
    return static_cast<double>(incremental_restraint_evals_)
      / incremental_evals_;
  }


  /** \name Incremental
      Efficient evaluation of non-bonded list based restraints is
      a bit tricky with incremental evaluation. To aid this, we
      offer a temporary solution where by you give a PairScore
      and a display upper bound (on the centers for the time being)
      to the MC object. It will then make sure this is applied
      properly.

      The bounding box is a hack for now.
      @{
  */
  /** Set whether to use incremental evaluate or evaluate all restraints
      each time. This cannot be changed during optimization.
  */
  void set_use_incremental_evaluate(bool tf) {
    eval_incremental_=tf;
  }
  bool get_use_incremental_evaluate() const {
    return eval_incremental_;
  }
  //! This is experimental and unstable
  void set_close_pair_score(PairScore *ps,
                            double distance,
                            const ParticlesTemp &particles,
                            const PairFilters &filters);
  /** @} */
 protected:
  /** Note that if return best is true, this will save the current
      state of the model. Also, if the move is accepted, the
      optimizer states will be updated.
  */
  bool do_accept_or_reject_move(double score, double last);
  bool do_accept_or_reject_move(double score)
  {
      return do_accept_or_reject_move(score, get_last_accepted_energy());
  }

  ParticlesTemp do_move(double probability);
  //! a class that inherits from this should override this method
  virtual void do_step();
  //! Get the current energy
  /** By default it just calls Optimizer::evaluate() if there is
      no maximum allowed difference or Optimizer::evaluate_if_below()
      if there is. Classes which override this method should be
      similarly aware for efficiency.

      The list of moved particles is passed.
   */
  virtual double do_evaluate(const ParticlesTemp &moved) const {
    IMP_UNUSED(moved);
    if (get_use_incremental_evaluate() ) {
      if (get_maximum_difference()
          < std::numeric_limits<double>::max()) {
        return evaluate_incremental_if_below(IMP::internal::get_index(moved),
                                             last_energy_+max_difference_);
      } else {
        ParticleIndexes pis=IMP::internal::get_index(moved);
        IMP_INTERNAL_CHECK(pis.size()==moved.size(), "Sizes don't match");
        return evaluate_incremental(pis);
      }
    } else {
      if (get_maximum_difference()
          < std::numeric_limits<double>::max()) {
        return evaluate_if_below(false, last_energy_+max_difference_);
      } else {
        return evaluate(false);
      }
    }
  }
  //! Only for incremental evaluation
  double evaluate_non_bonded(const ParticleIndexes &moved) const;
private:
  //! Evaluate the score of the model (or of a subset of the restraints
  //! if desired.
  double evaluate_incremental(const ParticleIndexes &moved) const;

  //! Evaluate the score of the model (or of a subset of the restraints
  //! if desired.
  double evaluate_incremental_if_below(const ParticleIndexes &moved,
                           double max) const;

  void setup_incremental();
  void teardown_incremental();
  void rollback_incremental();

  double temp_;
  double last_energy_;
  double best_energy_;
  double max_difference_;
  Float probability_;
  unsigned int stat_forward_steps_taken_;
  unsigned int stat_upward_steps_taken_;
  unsigned int stat_num_failures_;
  bool return_best_;
  IMP::OwnerPointer<Configuration> best_;
  ::boost::uniform_real<> rand_;

  // incremental
  Restraints flattened_restraints_;
  bool eval_incremental_;
  mutable Floats incremental_scores_;
  mutable Floats old_incremental_scores_;
  mutable Ints old_incremental_score_indexes_;
  compatibility::checked_vector<Ints> incremental_used_;


  struct NBLScore {
    OwnerPointer<PairScore> score_;
    typedef std::pair<ParticleIndex, double> ScorePair;
    typedef compatibility::checked_vector<ScorePair> ScorePairs;
    // first on the particle index then list the neighbors
    mutable compatibility::checked_vector<ScorePairs> cache_;
    double distance_;
    mutable double prior_, old_prior_;
    ParticleIndexes pis_;
    PairFilters filters_;
    mutable compatibility::checked_vector<std::pair<ParticleIndexPair,
                                                    double> > removed_;
    mutable ParticleIndexes added_;
    NBLScore(){}
    NBLScore(PairScore *ps,
             double distance,
             const ParticlesTemp &particles,
             const PairFilters &filters);
    void add_pair(ParticleIndex a, ParticleIndex b, double s) const;
    double get_score(Model *m, ParticleIndex moved,
                     const ParticleIndexes& nearby) const;
    void roll_back(Model *m, ParticleIndex moved);
    void initialize(Model *m,  ParticleIndexPairs all);
  };
  Ints nbl_incremental_used_;
  Ints to_dnn_;
  Ints from_dnn_;
  mutable ParticleIndex moved_;
  OwnerPointer<algebra::DynamicNearestNeighbor3D> dnn_;
  NBLScore nbl_;

  mutable unsigned int incremental_restraint_evals_;
  mutable unsigned int incremental_evals_;
};



//! This variant of Monte Carlo that relaxes after each move
class IMPCOREEXPORT MonteCarloWithLocalOptimization: public MonteCarlo
{
  IMP::OwnerPointer<Optimizer> opt_;
  unsigned int num_local_;
public:
  MonteCarloWithLocalOptimization(Optimizer *opt,
                               unsigned int steps);

  unsigned int get_number_of_steps() const {
    return num_local_;
  }

  Optimizer* get_local_optimizer() const {
    return opt_;
  }

  IMP_MONTE_CARLO(MonteCarloWithLocalOptimization);
};

//! This variant of Monte Carlo uses basis hopping
/** Basin hopping is where, after a move, a local optimizer is used to relax
    the model before the energy computation. However, the pre-relaxation state
    of the model is used as the starting point for the next step. The idea
    is that models are accepted or rejected based on the score of the nearest
    local minima, but they can still climb the barriers in between as the model
    is not reset to the minima after each step.
 */
class IMPCOREEXPORT MonteCarloWithBasinHopping:
public MonteCarloWithLocalOptimization
{
public:
  MonteCarloWithBasinHopping(Optimizer *opt, unsigned int ns);

  IMP_MONTE_CARLO(MonteCarloWithBasinHopping);
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MONTE_CARLO_H */
