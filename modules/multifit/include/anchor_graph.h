/**
 *  \file anchor_graph.h
 *  \brief anchor graph utilities
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */
#ifndef IMPMULTIFIT_ANCHOR_GRAPH_H
#define IMPMULTIFIT_ANCHOR_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <IMP/algebra/Vector3D.h>
#include "multifit_config.h"
#include "FittingSolutionRecord.h"
#include <IMP/Object.h>

IMPMULTIFIT_BEGIN_NAMESPACE

class IMPMULTIFITEXPORT ProbabilisticAnchorGraph : public Object {
// Property types
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::property<boost::vertex_index_t, int> VertexIndexProperty;
// Graph type
typedef boost::adjacency_list<boost::vecS, boost::vecS,
                              boost::undirectedS,
                              VertexIndexProperty,
                              EdgeWeightProperty> AnchorGraph;
typedef boost::graph_traits<AnchorGraph> GTraits;
typedef boost::graph_traits<AnchorGraph const> Const_GTraits;
typedef GTraits::vertex_descriptor GVertex;
typedef GTraits::edge_descriptor GEdge;
public:
  ProbabilisticAnchorGraph(algebra::Vector3Ds anchor_positions);

  void add_edge(int i,int j) {
    boost::add_edge(id2node_[i],id2node_[j],g_);
  }
  //! Set the probability of a component to be located at each acnhor position
  /**
     \param[in] comp_ind the component index
     \param[in] comp_cen the position of the component centroid
     \param[in] sols the fitting solutions of the component
   */
  void set_component_probabilities_on_anchors(
                                int comp_ind,
                                algebra::Vector3D comp_center,
                                multifit::FittingSolutionRecords sols);
  void show(std::ostream& out=std::cout) const;
  unsigned int get_number_of_anchors() const {return boost::num_vertices(g_);}
  unsigned int get_number_of_edges() const {return boost::num_edges(g_);}
  IMP_OBJECT_INLINE(ProbabilisticAnchorGraph, show(out),{});
private:
  AnchorGraph g_;
  std::vector<Floats> anchor_to_comp_probabilities_;
  algebra::Vector3Ds positions_;
  std::vector<GVertex> id2node_;
};

IMP_OBJECTS(ProbabilisticAnchorGraph,ProbabilisticAnchorGraphs);
IMPMULTIFIT_END_NAMESPACE
#endif  /* IMPMULTIFIT_ANCHOR_GRAPH_H */
