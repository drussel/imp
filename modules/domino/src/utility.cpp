/**
 *  \file interaction_graph.cpp
 *  \brief Score particles with respect to a tunnel.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/domino/utility.h>
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/Restraint.h>
#include <IMP/ScoreState.h>
#include <IMP/internal/map.h>
#include <boost/graph/graphviz.hpp>
#include <IMP/internal/graph_utility.h>
#include <IMP/domino/internal/restraint_evaluator.h>
#include <IMP/RestraintSet.h>
#include <IMP/domino/particle_states.h>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>
#if BOOST_VERSION > 103900
#include <boost/property_map/property_map.hpp>
#else
#include <boost/property_map.hpp>
#include <boost/vector_property_map.hpp>
#endif


IMPDOMINO_BEGIN_NAMESPACE



template <class Graph>
class DirectCollectVisitor: public boost::default_dfs_visitor {
  typename boost::property_map<Graph,
                      boost::vertex_name_t>::const_type vm_;
  ParticlesTemp &vals_;
public:
  const ParticlesTemp &get_collected() {
    std::sort(vals_.begin(), vals_.end());
    vals_.erase(std::unique(vals_.begin(), vals_.end()), vals_.end());
    return vals_;
  }
  DirectCollectVisitor(const Graph &g, ParticlesTemp &vals): vals_(vals)
    {
      vm_=boost::get(boost::vertex_name, g);
    }
  void discover_vertex(typename boost::graph_traits<Graph>::vertex_descriptor u,
                       const Graph&) {
    Object *o= vm_[u];
    //std::cout << "Visiting " << o->get_name() << std::endl;
    Particle *p=dynamic_cast<Particle*>(o);
    if (p) {
      vals_.push_back(p);
    }
  }
};


namespace {
 typedef boost::graph_traits<DependencyGraph> DGTraits;
  typedef DGTraits::vertex_descriptor DGVertex;
  typedef boost::property_map<DependencyGraph,
                              boost::vertex_name_t>::type DGVertexMap;
  typedef boost::property_map<DependencyGraph,
                              boost::vertex_name_t>::const_type
  DGConstVertexMap;
}





ParticlesTemp get_dependent_particles(Particle *p,
                                      const ParticlesTemp &all,
                                      const DependencyGraph &dg) {
  // find p in graph, ick
  DGConstVertexMap dpm= boost::get(boost::vertex_name, dg);
  std::pair<DGTraits::vertex_iterator, DGTraits::vertex_iterator> be
    = boost::vertices(dg);
  IMP::internal::Set<Object*> block(all.begin(), all.end());
  boost::vector_property_map<int> color(boost::num_vertices(dg));
  int start=-1;
  for (; be.first != be.second; ++be.first) {
    if (dpm[*be.first]==p) {
      start=*be.first;
    } else if (block.find(dpm[*be.first]) != block.end()) {
      // block traversal though the other nodes
      color[*be.first]= boost::color_traits<int>::black();
    }
  }
  if (start==-1) {
    return ParticlesTemp();
  }
  ParticlesTemp pt;
  DirectCollectVisitor<DependencyGraph> cv(dg, pt);
  boost::depth_first_visit(dg, start, cv, color);
  return cv.get_collected();
}

ParticlesTemp get_dependent_particles(Particle *p,
                                      const ParticlesTemp &all) {
  Model *m= p->get_model();
  DependencyGraph dg
    = get_dependency_graph(get_restraints(m->restraints_begin(),
                                          m->restraints_end()));
  return get_dependent_particles(p, all, dg);
}



void load_particle_states(const Subset &s,
                          const SubsetState &ss,
                          const ParticleStatesTable *pst) {
  internal::load_particle_states(s.begin(), s.end(), ss, pst);
  if (s.size()!=0) {
    s[0]->get_model()->update();
  }
}
namespace {
RestraintsAndWeights get_restraints_and_weights(const Subset &s,
                              const ParticleStatesTable *pst,
                              const DependencyGraph &dg,
                              RestraintSet *rs) {
  RestraintsAndWeights rw= get_restraints_and_weights(rs);
  Subset other=pst->get_subset();
  ParticlesTemp oms;
  std::set_difference(other.begin(), other.end(),
                      s.begin(), s.end(),
                      std::back_inserter(oms));
  IMP::internal::Map<Restraint*, int> index
    = IMP::internal::get_graph_index<Restraint>(dg);
  Ints to_remove;
  for (unsigned int i=0; i< rw.first.size(); ++i) {
    if (IMP::internal::get_has_ancestor(dg, index[rw.first[i]], oms)) {
      to_remove.push_back(i);
    }
  }
  for (int i=to_remove.size()-1; i >=0; --i) {
    rw.first.erase(rw.first.begin()+to_remove[i]);
    rw.second.erase(rw.second.begin()+to_remove[i]);
  }
  return rw;
}
}

RestraintsTemp get_restraints(const Subset &s,
                              const ParticleStatesTable *pst,
                              const DependencyGraph &dg,
                              RestraintSet *rs) {
  return get_restraints_and_weights(s, pst, dg, rs).first;
}



Ints get_partial_index(const ParticlesTemp &particles,
               const Subset &subset, const Subsets &excluded) {
  for (unsigned int i=0; i< excluded.size(); ++i) {
    bool all=true;
    for (unsigned int j=0; j< particles.size(); ++j) {
      if (!std::binary_search(excluded[i].begin(), excluded[i].end(),
                              particles[j])) {
        all=false;
        break;
      }
    }
    if (all) {
      return Ints();
    }
  }
  Ints ret(particles.size(), -1);
  for (unsigned int i=0; i< particles.size(); ++i) {
    Subset::const_iterator it= std::lower_bound(subset.begin(),
                                                subset.end(), particles[i]);
    if (it!= subset.end() && *it == particles[i]) {
      ret[i]= it-subset.begin();
    }
  }
  IMP_IF_LOG(VERBOSE) {
    IMP_LOG(VERBOSE, "Returning ");
    for (unsigned int i=0; i< ret.size(); ++i) {
      IMP_LOG(VERBOSE, ret[i] << " ");
    }
    IMP_LOG(VERBOSE, "for ");
     for (unsigned int i=0; i< particles.size(); ++i) {
       IMP_LOG(VERBOSE, particles[i]->get_name() << " ");
     }
     IMP_LOG(VERBOSE, " subset " << subset << std::endl);
  }
  return ret;
}

Ints get_index(const ParticlesTemp &particles,
               const Subset &subset, const Subsets &excluded) {
  Ints pi= get_partial_index(particles, subset, excluded);
  if (std::find(pi.begin(), pi.end(), -1) != pi.end()) return Ints();
  else return pi;
}




IMPDOMINO_END_NAMESPACE
