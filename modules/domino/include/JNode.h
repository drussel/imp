/**
 *  \file JNode.h   \brief Handles all functionalities of a junction tree node.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPDOMINO_JNODE_H
#define IMPDOMINO_JNODE_H

#include "domino_config.h"
#include "DiscreteSampler.h"
#include "CombState.h"
#include <IMP/base_types.h>
#include "RestraintEvaluatorI.h"
#include <vector>
#include <sstream>
#include <algorithm>

IMPDOMINO_BEGIN_NAMESPACE

//! Handles all functionalities of a junction tree node.
/** Holds multiple particles, their states and the current marginalization
    state.
 */
class IMPDOMINOEXPORT JNode
{
public:
  //! Constructor
  /** \param [in] p    a vector of IMP particles that are part of the node
      \param [in] node_ind  The index of the JNode
   */
  JNode(const Particles &p, int node_ind);
  ~JNode() {
    //    free(opt_state);
  }
  void set_restraint_evaluator(RestraintEvaluatorI *rstr_eval) {
    rstr_eval_=rstr_eval;
  }
  //! Set the discrete sampling space of each of particles in the node
  /** \param [in] ds the sampler data
   */
  void init_sampling(DiscreteSampler &ds);

  //! Get the set of intersecting particles between two nodes
  /** \param [in] other   the second node
      \param[out] in      the intersection set
   */
  void get_intersection(const JNode &other, Particles &in) const;
  void get_intersection2(const JNode &other, Particles in) const {}

  //! checks if the input set of particles is part of the node
  /** \param [in] p   a set of particles
      \return True if the node contains the input set of nodes, False otherwise
   */
  bool is_part(const Particles &p) const;

  //! Fill states as encoded in the node for the input subset of particles
  /** \param[in] particles   a set of particles
      \param[in] states      the dataset to be filled with states.
   */
  void populate_states_of_particles(Particles *particles,
          Combinations *states);
  //! Adds the restraint values to all combinations
  /**\param[in] r      the  restraint
      \param[in] ps     the particles participate in the restraint at the
                        hierarhcy level encoded in the graph
      \param[in] weight the weight of the restraint
   */
  void realize(Restraint *r, Particles *ps, Float weight);
  //! Finds the minimum combination in the node.
  /** \param[in]   move_to_state     true if the model should move to the new
                                     state
      \param[in] num_of_solutions
      \return all of the combinations that reach the global minimum
   */
  std::vector<CombState *> * find_minimum(bool move_to_state = false,
                                          unsigned int num_of_solutions=1);

  CombState* get_state(unsigned int index, bool move_to_state = false);

  void show(std::ostream& out = std::cout) const;
  void show_sampling_space(std::ostream& out = std::cout) const;
  unsigned int get_node_index() const {
    return node_ind_;
  }
  const Particles *get_particles() const {
    return &particles_;
  }
  //! Return the optimal score for the separator, for the given separator
  //! find the optimal combination of the rest of the components.
  /**
     \param[in] s a combination of some of the particles in the node
     \param[in] move_to_state True if should move to the state with the
                              minimum score.
  */
  std::vector<CombState *> min_marginalize(const CombState &s,
      bool move_to_state = false);

  //! Update the potentials
  /** \param[in] old_score_separators
      \param[in] new_score_separators
      \param[in] intersection_particles
   */
  void update_potentials(
    const std::map<std::string, float>  &old_score_separators,
    const std::map<std::string, float>  &new_score_separators,
    const Particles &intersection_particles);
  const DiscreteSampler* get_sampler() {
    return ds_;
  }
  //! Move the system to the state encoded in the class
  void move2state(CombState *cs);
  void clear();

  //! Get the score for this combination
  Float get_score(const CombState &comb);

protected:

  Particles particles_; //a sorted list of particles that are part of the node
  unsigned int node_ind_;
  Combinations comb_states_;
  std::vector<std::string> comb_states_keys_;
  DiscreteSampler *ds_;
  RestraintEvaluatorI *rstr_eval_;
};


IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_JNODE_H */
