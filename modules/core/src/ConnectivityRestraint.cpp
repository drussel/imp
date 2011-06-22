/**
 *  \file ConnectivityRestraint.cpp  \brief Connectivity restraint.
 *
 *  Restrict max distance between at least one pair of particles of any
 *  two distinct types.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/ConnectivityRestraint.h>

#include <IMP/Model.h>
#include <IMP/Particle.h>
#include <IMP/log.h>
#include <IMP/PairScore.h>
#include <IMP/core/PairRestraint.h>
#include <IMP/core/internal/CoreListSingletonContainer.h>

#include <climits>

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>

#include <limits>

IMPCORE_BEGIN_NAMESPACE

ConnectivityRestraint::ConnectivityRestraint(PairScore *ps,
                                             SingletonContainer *sc):
  Restraint("ConnectivityRestraint %1%"),
  ps_(ps)
{
  if (sc) {
    sc_= sc;
  } else {
  }
}

namespace {
  internal::CoreListSingletonContainer* get_list(SingletonContainer *sc) {
    internal::CoreListSingletonContainer *ret
      = dynamic_cast<internal::CoreListSingletonContainer*>(sc);
    if (!ret) {
      IMP_THROW("Can only use the set and add methods when no container"
                << " was passed on construction of ConnectivityRestraint.",
                ValueException);
    }
    return ret;
  }
}

void ConnectivityRestraint::set_particles(const Particles &ps) {
  if (!sc_ && !ps.empty()) {
    sc_= new internal::CoreListSingletonContainer(ps[0]->get_model(),
                                                  "connectivity list");
  }
  get_list(sc_)->set_particles(ps);
}

void ConnectivityRestraint::add_particles(const Particles &ps) {
  if (!sc_&& !ps.empty()) {
    sc_= new internal::CoreListSingletonContainer(ps[0]->get_model(),
                                                  "connectivity list");
  }
  get_list(sc_)->add_particles(ps);
}

void ConnectivityRestraint::add_particle(Particle *ps) {
  if (!sc_) {
    sc_= new internal::CoreListSingletonContainer(ps->get_model(),
                                                  "connectivity list");
  }
  get_list(sc_)->add_particle(ps);
}

namespace {
  /*typedef boost::adjacency_list<boost::vecS, boost::vecS,
                        boost::undirectedS, boost::no_property,
                        boost::property<boost::edge_weight_t, double> > Graph;*/
  typedef boost::adjacency_matrix<boost::undirectedS, boost::no_property,
                        boost::property<boost::edge_weight_t, double> > Graph;
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef Graph::edge_property_type Weight;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  void compute_mst(const SingletonContainer *a,
                   PairScore *ps,
                   Graph &g,
                   std::vector<Edge> &mst) {
    try {
      for (unsigned int i=0; i< a->get_number_of_particles(); ++i) {
        for (unsigned int j=0; j<i; ++j) {
          double d= ps->evaluate(ParticlePair(a->get_particle(i),
                                              a->get_particle(j)), NULL);
          IMP_LOG(VERBOSE, "ConnectivityRestraint edge between "
                  << a->get_particle(i)->get_name() << " and "
                  << a->get_particle(j)->get_name() << " with weight "
                  << d << std::endl);
          /*Edge e =*/ boost::add_edge(i, j, Weight(d), g);
          //boost::put(boost::edge_weight_t(), g, e, d);
        }
      }
      mst.resize(a->get_number_of_particles()-1);
    } catch (std::bad_alloc&c) {
      IMP_FAILURE("Out of memory in ConnectivityRestraint.");
    }
    boost::kruskal_minimum_spanning_tree(g, mst.begin());

    /*
    boost::property_map<Graph, boost::vertex_index_t>::type indexmap
      = get(boost::vertex_index, g);
    boost::prim_minimum_spanning_tree(g, indexmap);
    for (unsigned int i=0; i< a->get_number_of_particles(); ++i) {
      Vertex cv= boost::vertex(i,g);
      if (boost::get(indexmap, cv) !=  cv) {
        mst.push_back(boost::edge(cv, boost::get(indexmap, cv), g).first);
      }
      }*/
  }

}


double
ConnectivityRestraint::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_CHECK_OBJECT(ps_.get());
  IMP_OBJECT_LOG;
  std::vector<Edge> mst;
  if (!sc_) return 0;
  Graph g(sc_->get_number_of_particles());
  compute_mst(sc_, ps_, g, mst);
  double sum=0;
  // could be more clever if accum is NULL
  for (unsigned int index=0; index< mst.size(); ++index) {
    int i= boost::target(mst[index], g);
    int j= boost::source(mst[index], g);
    IMP_LOG(VERBOSE, "ConnectivityRestraint edge between "
            << sc_->get_particle(i)->get_name()
            << " and " << sc_->get_particle(j)->get_name() << std::endl);
    if (accum) {
      sum+= ps_->evaluate(ParticlePair(sc_->get_particle(i),
                                       sc_->get_particle(j)),
                          accum);
    } else {
      sum += boost::get(boost::edge_weight_t(), g, mst[index]);
    }
  }
  return sum;
}


Restraints ConnectivityRestraint::get_instant_decomposition() const {
  ParticlePairs pp= get_connected_pairs();
  Restraints ret(pp.size());
  for (unsigned int i=0; i< pp.size(); ++i) {
    IMP_NEW(PairRestraint, pr, (ps_, pp[i]));
    std::ostringstream oss;
    oss << get_name() << " " << i;
    pr->set_name(oss.str());
    ret[i]=pr;
  }
  return ret;
}


ParticlePairs ConnectivityRestraint::get_connected_pairs() const {
  IMP_CHECK_OBJECT(ps_.get());
  std::vector<Edge> mst;
  Graph g(sc_->get_number_of_particles());
  compute_mst(sc_, ps_, g, mst);
  ParticlePairs ret(mst.size());
  for (unsigned int index=0; index< mst.size(); ++index) {
    int i= boost::target(mst[index], g);
    int j= boost::source(mst[index], g);
    ret.set(index, ParticlePair(sc_->get_particle(i),
                                sc_->get_particle(j)));
  }
  return ret;
}

ParticlesTemp ConnectivityRestraint::get_input_particles() const {
  if (!sc_) return ParticlesTemp();
  ParticlesTemp ret;
  IMP_FOREACH_SINGLETON(sc_, {
      ParticlesTemp cs = ps_->get_input_particles(_1);
      ret.insert(ret.end(), cs.begin(), cs.end());
    });
  return ret;
}

ContainersTemp ConnectivityRestraint::get_input_containers() const {
  if (!sc_) return ContainersTemp();
  ContainersTemp ret;
  IMP_FOREACH_SINGLETON(sc_, {
      ContainersTemp cs
        = ps_->get_input_containers(_1);
      ret.insert(ret.end(), cs.begin(), cs.end());
    });
  return ret;
}


void ConnectivityRestraint::do_show(std::ostream& out) const
{
  if (!sc_) {
    out << "container is NULL" << std::endl;
  } else {
    out << "container is " << *sc_ << std::endl;
  }
}

IMPCORE_END_NAMESPACE
