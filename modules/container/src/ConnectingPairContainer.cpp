/**
 *  \file ConnectingPairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. Connecting rights reserved.
 *
 */

#include "IMP/container/ConnectingPairContainer.h"
#include <IMP/core/internal/DifferenceSingletonContainer.h>
#include <IMP/container/ListPairContainer.h>
#include <IMP/core/internal/pair_helpers.h>
#include <IMP/core/internal/close_pairs_helpers.h>
#include <IMP/PairModifier.h>
#include <IMP/algebra/vector_search.h>
#if BOOST_VERSION > 103900
#include <boost/property_map/property_map.hpp>
#else
#include <boost/property_map.hpp>
#include <boost/vector_property_map.hpp>
#endif
#include <boost/pending/disjoint_sets.hpp>
#include <algorithm>

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>



IMPCONTAINER_BEGIN_NAMESPACE

namespace {
  typedef boost::vector_property_map<unsigned int> LIndex;
  typedef LIndex Parent;
  typedef boost::disjoint_sets<LIndex,Parent> UF;
  void build_graph(SingletonContainer *sc, ParticlePairsTemp &out, UF &uf) {
    std::vector<algebra::VectorD<3> > vs(sc->get_number_of_particles());
    IMP_FOREACH_SINGLETON(sc,
                          vs[_2]= core::XYZ(_1).get_coordinates(););
    IMP_NEW(algebra::NearestNeighborD<3>, nn, (vs));
    unsigned int nnn=static_cast<unsigned int>(
                                std::sqrt(static_cast<double>(vs.size()))+1);
    for (unsigned int i=0; i< vs.size(); ++i) {
      Ints ni=nn->get_nearest_neighbors(i, nnn);
      unsigned int si= uf.find_set(i);
      for (unsigned int j=0; j< ni.size(); ++j) {
        unsigned int sj= uf.find_set(ni[j]);
        if (sj != si) {
          uf.union_set(si, sj); // more efficient call
          out.push_back(ParticlePair(sc->get_particle(i),
                                     sc->get_particle(ni[j])));
          break;
        }
      }
    }
    //if (uf.count_sets() > 1) {
    for (unsigned int i=1; i< vs.size(); ++i) {
      int si=uf.find_set(i);
      int si0= uf.find_set(0);
      if (si != si0) {
        out.push_back(ParticlePair(sc->get_particle(si),
                                   sc->get_particle(si0)));
        uf.union_set(si, si0);
      }
    }
    //}
  }



  /*typedef boost::adjacency_list<boost::vecS, boost::vecS,
                        boost::undirectedS, boost::no_property,
                        boost::property<boost::edge_weight_t, double> > Graph;*/
  typedef boost::adjacency_matrix<boost::undirectedS, boost::no_property,
                        boost::property<boost::edge_weight_t, double> > Graph;
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef Graph::edge_property_type Weight;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  void compute_mst(const SingletonContainer *sc,
                   ParticlePairsTemp &out) {
    static unsigned int nnn=10;

    std::vector<algebra::VectorD<3> > vs(sc->get_number_of_particles());
    IMP_FOREACH_SINGLETON(sc,
                          vs[_2]= core::XYZ(_1).get_coordinates(););
    IMP_NEW(algebra::NearestNeighborD<3>, nn, (vs));
    ///unsigned int nnn=static_cast<unsigned int>(std::sqrt(vs.size())+1);
    Graph g(vs.size());
    for (unsigned int i=0; i< vs.size(); ++i) {
      core::XYZR di(sc->get_particle(i));
      Ints ni=nn->get_nearest_neighbors(i, nnn);
      for (unsigned int j=0; j< ni.size(); ++j) {
        core::XYZR dj(sc->get_particle(ni[j]));
        double d= algebra::get_distance(di.get_sphere(), dj.get_sphere());
        boost::add_edge(i, ni[j], Weight(d), g);
      }
    }
    // count sets as we go along
    std::vector<Edge> mst(vs.size()-1);
    boost::kruskal_minimum_spanning_tree(g, mst.begin());

    for (unsigned int index=0; index< mst.size(); ++index) {
      int i= boost::target(mst[index], g);
      int j= boost::source(mst[index], g);
      out.push_back(ParticlePair(sc->get_particle(i), sc->get_particle(j)));
    }
  }

}



ConnectingPairContainer::ConnectingPairContainer(SingletonContainer *c,
                                                 double error):
  IMP::core::internal::ListLikePairContainer(c->get_model(),
                                             "ConnectingPairContainer"),
  error_(error),
  mst_(true) {
  initialize(c);
}

void ConnectingPairContainer::initialize(SingletonContainer *sc) {
  sc_=sc;
  fill_list(true);
  Model *m=sc->get_particle(0)->get_model();
  mv_= new core::internal::XYZRMovedSingletonContainer(sc, error_);
  initialize_active_container(m);
}

IMP_ACTIVE_CONTAINER_DEF(ConnectingPairContainer,);

ParticlesTemp ConnectingPairContainer::get_state_input_particles() const {
  return sc_->get_particles();
}


ContainersTemp ConnectingPairContainer::get_state_input_containers() const {
  ContainersTemp ret(2);
  ret[0]=sc_;
  ret[1]=mv_;
  return ret;
}


void ConnectingPairContainer::fill_list(bool /*first*/) {
  // if we have a list and nothing moved further than error do nothing
  // otherwise rebuild
  ParticlePairsTemp new_list;

  if (mst_) {
    compute_mst(sc_, new_list);
  } else {
    LIndex index;
    Parent parent;
    UF uf(index, parent);
    unsigned int sz= sc_->get_number_of_particles();
    for (unsigned int i=0; i< sz; ++i) {
      uf.make_set(i);
    }
    build_graph(sc_, new_list, uf);
  }
  update_list(new_list);
}

void ConnectingPairContainer::do_before_evaluate() {
  if (mv_->get_number_of_particles() != 0) {
    fill_list(false);
    mv_->reset();
  }
}


void ConnectingPairContainer::do_after_evaluate() {
  IMP::core::internal::ListLikePairContainer::do_after_evaluate();
}


void ConnectingPairContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "container " << *sc_ << std::endl;
}


IMPCONTAINER_END_NAMESPACE
