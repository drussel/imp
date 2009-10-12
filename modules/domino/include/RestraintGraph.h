/**
 *  \file RestraintGraph.h
 *  \brief creates a MRF from a set of particles and restraints
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPDOMINO_RESTRAINT_GRAPH_H
#define IMPDOMINO_RESTRAINT_GRAPH_H

#include "config.h"
#include "JNode.h"
#include "JEdge.h"
#include "DiscreteSampler.h"
#include "JunctionTree.h"

#include <IMP/Model.h>
#include <IMP/Restraint.h>

#include <vector>
#include <map>
#include <sstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
IMPDOMINO_BEGIN_NAMESPACE

// it is declared in several places
#ifndef IMP_SWIG
//! The key for the string Domino uses as a unique index
IMPDOMINOEXPORT StringKey node_name_key();
#endif

template < typename TimeMap > class dfs_time_visitor :
      public boost::default_dfs_visitor
{
  typedef typename boost::property_traits <TimeMap >::value_type T;
public:
  dfs_time_visitor(TimeMap dmap, TimeMap fmap, T & t)
      :  m_dtimemap(dmap), m_ftimemap(fmap), m_time(t) {
  }
  template < typename Vertex, typename Graph >
  void discover_vertex(Vertex u, const Graph & g) const {
    put(m_dtimemap, u, m_time++);
  }
  template < typename Vertex, typename Graph >
  void finish_vertex(Vertex u, const Graph & g) const {
    put(m_ftimemap, u, m_time++);
  }
  TimeMap m_dtimemap;
  TimeMap m_ftimemap;
  T & m_time;
};

// should think about something more general here, since each level of
// hierarchy will have its own state.
//

//! RestraintGraph
class IMPDOMINOEXPORT RestraintGraph
{


  typedef std::pair<unsigned int, unsigned int> Pair;
  typedef boost::adjacency_list < boost::vecS, boost::vecS,
    boost::undirectedS, boost::no_property,
    boost::property<boost::edge_weight_t,
    boost::vecS> > Graph;

public:
  //! Constructor
  /** \param[in] jt Holds the junction tree that represent the
                    system dependencies
      \param[in] mdl The IMP model
   */
  RestraintGraph(const JunctionTree &jt,Model *mdl);
  //    void clear_states();
  //void set_model(IMP::Model *m_);
  void initialize_graph(int number_of_nodes);
  void move_model2global_minimum() const;
  CombState * get_minimum_comb() const;
  //! Creates a new node and add it to the graph
  /** \param[in] node_index the index of the node
      \param[in] particles  the particles that are part of the node
   */
  void add_node(unsigned int node_index, Particles &particles);

  //! Creates an undirected edge between two nodes
  /** \param[in] node1_ind  the index of the first node
      \param[in] node2_ind  the index of the second node
   */
  void add_edge(unsigned int node1_ind, unsigned int node2_ind);
  void set_sampling_space(DiscreteSampler &ds);

  //! Initalize potentials according to the input restraint set.
  /** \param[in] r      the  restraint
      \param[in] ps     the particles participate in the restraint at the
                        hierarhcy level encoded in the graph
      \param[in] weight the weight of the restraint
   */
  void initialize_potentials(Restraint *r, Particles *ps, Float weight);
  unsigned int number_of_nodes() const {
    return  num_vertices(g_);
  }
  unsigned int number_of_edges() const {
    return  num_edges(g_);
  }
  //! Find the top solutions
  /**
     /param[in] num_of_solutions the number of top solutions to report
   */
  void infer(unsigned int num_of_solutions=1);

  //! Show the restraint graph
  void show(std::ostream& out = std::cout) const;
  //! Prints the value of each restraint encoded in the graph for a state of
  //! the global minimum
  void analyse(std::ostream& out = std::cout) const;
  //! Sets the optimizable attributes of the optimizable components to the
  //! values that build the minimum of the scoring function when the state
  //! of the root of the junction tree is of a spcific index.
  /**
   */
  void show_sampling_space(std::ostream& out = std::cout) const;

  //  float move_model2state(unsigned int state_index) const;

  Float get_minimum_score() const {
    std::stringstream err_msg;
    err_msg << "RestraintGraph::get_minimum_score the graph has not been"
            << " infered";
    IMP_INTERNAL_CHECK(infered_, err_msg.str());
    return (*(min_combs_->begin()))->get_total_score();
  }

  void clear();
  //! Get the i'th best combination
  /**
  \param[in] i the i'th best combination
  \exception if no combinations have been infered or if i is out of range.
  \return the i'th best combination
   */
  const CombState *get_opt_combination(unsigned int i) const;
  //! \return a node that contains the input set of particles.
  /** It might be that there is more than one such one. The function returns
      the first it finds.
     \param[in] p the set of particles
     \return a node that contains the input set of particles
     \exception IMP exception if none of the graph nodes contain the given
                set of particles.
   */
  JNode * get_node(const Particles &p);
protected:
  //! Load junction tree and set the restraint graph
  /**
  \param[in] jt contains the junction tree data
  \param[in] mdl
  \note The function uses the particle name attribute as identifier
   */
  void load_data(const JunctionTree &jt,Model *mdl);

  //! Determine a DFS
  /** \param[in]  root_ind the index of the node from which the DFS starts
      Stores the discover order of the nodes.
                  discover_time[i] is the discover time in the DFS of node
                  with index i.
   */
  void dfs_order(unsigned int root_ind);
  void move_model2state_rec(unsigned int father_ind,
                            CombState &best_state) const;

  JEdge* get_edge(unsigned int n1, unsigned int n2) const {
    return edge_data_.find(get_edge_key(n1, n2))->second;
  }

  //! Recursive Collect Evidence, father is the cluster that invoked
  //! CollectEvidence
  /** \param[in]  father_ind the index of the node to start collecting from
   */
  unsigned int collect_evidence(unsigned int father_ind);

  //! Recursive Distribution of evidence. father is the cluster that
  //! invoked distribute_evidence
  /** \param[in]  father_ind  the index of the node to start collecting from
   */
  void  distribute_evidence(unsigned int father_ind);

  //! Recursive Distribution of minimum state.
  /** \param[in]  father_ind the index of the node to start the min_dist from
      \param[in]  min_comb       the minimum combination so far.
      Each child node will add the states of its particles.
   */
  void distribute_minimum(unsigned int father_ind, CombState *min_comb);

  //! Updates node with index w based on the evidence in the node with index v
  /**
   */
  void update(unsigned int w, unsigned int v);
  Pair get_edge_key(unsigned int node1_ind, unsigned int node2_ind) const;
 protected:
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  void clear_infered_data();

  std::map<Pair, JEdge *> edge_data_;
  std::vector<JNode *> node_data_; // the i'th entry holds the i'th jnode.
  //  Model *m_;
  Graph g_;
  std::map<int, int> particle2node; //for quick graph building
  std::vector<int> node2particle;
  //inference support data structures
  std::vector<unsigned int> discover_time_;
  // discover_order[i] , the discover time of node number i
  unsigned int root_;
  bool infered_;
  std::vector<CombState *> *min_combs_;
};

IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_RESTRAINT_GRAPH_H */
