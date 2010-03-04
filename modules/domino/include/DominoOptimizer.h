/**
 *  \file DominoOptimizer.h   \brief An exact inference optimizer
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPDOMINO_DOMINO_OPTIMIZER_H
#define IMPDOMINO_DOMINO_OPTIMIZER_H

#include "config.h"
#include "RestraintGraph.h"
#include "JunctionTree.h"
#include "RestraintEvaluatorI.h"
#include "CombState.h"
#include <IMP/RestraintSet.h>

#include <IMP/Optimizer.h>
#include <IMP/Restraint.h>
#include <IMP/Model.h>

IMPDOMINO_BEGIN_NAMESPACE

//! An exact inference optimizer.
/** \ingroup optimizer
 */
class IMPDOMINOEXPORT DominoOptimizer : public Optimizer
{
  typedef boost::tuple<Restraint *,Particles,Float> OptTuple;
public:
  //! Constructor
  /**
  \param[in] jt A special type of tree decomposition that holds the
                graph on which inference will be run
  \param[in] m  The model
   */
  //TODO - add here much more!
  DominoOptimizer(const JunctionTree &jt, Model *m,
                  RestraintEvaluatorI *r_eval);

  IMP_OPTIMIZER(DominoOptimizer);

  void set_sampling_space(DiscreteSampler *ds);
  //TODO : set a discrete sampling space for one node
  //void set_sampling_space(int node_ind, DiscreteSampler *ds);

  void show_restraint_graph(std::ostream& out = std::cout) const {
    g_->show(out);
  }
  void show_optimum_configuration(unsigned int conf_ind,
                                   std::ostream& out = std::cout) const {
    out<<"OPT_CONF:";
    g_->get_opt_combination(conf_ind)->show(out);
  }
  DiscreteSampler *get_sampling_space() const {return ds_;}
  RestraintGraph  *get_graph() const {return g_;}
  void set_restraint_evaluator(RestraintEvaluatorI *rstr_eval) {
    rstr_eval_=rstr_eval;
  }
  inline unsigned int get_number_of_solutions() const {
      return num_of_solutions_;}
  inline void set_number_of_solutions(unsigned int n){num_of_solutions_=n;}
  void move_to_opt_comb(unsigned int i) const;
  //! Add a restraint that should be used in the optimization procedure.
  /**
  /param[in] r   the restraint
  /note The input restraint can also be a RestraintSet, a recursive addition
        of restraints is performed in these cases.
   */
  void add_restraint(Restraint *r);
  //! Add a restraint that should be used in the optimization procedure.
  /**
  To support hierarchy, some time the particles of the restraint are a
  refined set of the particles represented in the restraint graph.
  In this function the user set the particles of the restraint (as interpert
  by the restraint graph). It is up to the user to make sure that ps represents
  a coarser set of particles than the ones actually used in the restraint.
   */
  void add_restraint(Restraint *r,Particles ps,float weight=1.);

  //! A function used mostly for debugging and learning purposes.
  //! Should NOT be used for real optimization protocol.
  /**
  \param[in] states to be filled with all of the combinations encoded
                    in the restraint graph
  \param[in] calc_score should scores be calculated for each combination
  \param[in] use_rsr_eval if True use RestraintEvaluator
                             else use model.evaluate()
   */
  void exhaustive_enumeration(CombStates &states, bool calc_score=false,
                              bool use_rsr_eval=false);

  //! A function used mostly for debugging and learning purposes.
  //! Should NOT be used for real optimization protocol.
  /**
  \param[in] states states to score
  \param[out] scores the i'th entry will contain the values
                     of all restraint for states[i]
   */
  void score_combinations(const CombStates &states,
                          std::vector<Floats> &scores);

protected:
  //! Recursivly add restraints
  /*
    \param[in] rs a restraint which can also be a restraint set
    \param[in] weight is a recursive multiplication of all weights
               from parent RestraintSet
  */
  void add_restraint_recursive(Restraint *rs, Float weight);

  void clear(); //TODO implement!
  //! Creates a new node and add it to the graphs that take part in the
  //! optimization
  /*
    \param[in] node_index the index of the node
    \param[in] particles_ind  the particles that are part of the node
  */
  void add_jt_node(int node_index, std::vector<Int>  &particles_ind,
                   Model &m);
  //! Adds an undirected edge between a pair of nodes
  /** \param[in] node1_ind  the index of the first node
      \param[in] node2_ind  the index of the second node
   */
  void add_jt_edge(int node1_ind, int node2_ind);

  void initialize_jt_graph(int number_of_nodes);
  DiscreteSampler *ds_;
  RestraintGraph *g_;
  unsigned int num_of_solutions_;
  std::vector<OptTuple> rs_;
  RestraintEvaluatorI *rstr_eval_;
};
IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_DOMINO_OPTIMIZER_H */
