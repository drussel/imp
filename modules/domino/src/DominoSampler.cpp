/**
 *  \file ConjugateGradients.cpp  \brief Simple conjugate gradients optimizer.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/domino/DominoSampler.h>
#include <IMP/container/ListSingletonContainer.h>
#include <IMP/domino/utility.h>
#include <IMP/domino/internal/tree_inference.h>
#include <IMP/domino/optimize_restraints.h>

#include <IMP/internal/graph_utility.h>
#include <IMP/file.h>
#include <boost/graph/connected_components.hpp>
#include <IMP/domino/internal/loopy_inference.h>
#if BOOST_VERSION > 103900
#include <boost/property_map/property_map.hpp>
#else
#include <boost/property_map.hpp>
#include <boost/vector_property_map.hpp>
#endif


IMPDOMINO_BEGIN_NAMESPACE


DominoSampler::DominoSampler(Model *m, ParticleStatesTable* pst,
                             std::string name):
  DiscreteSampler(m, pst, name), has_sg_(false), csf_(false) {
}

DominoSampler::DominoSampler(Model *m, std::string name):
  DiscreteSampler(m, new ParticleStatesTable(), name), has_sg_(false),
  csf_(false){
}



namespace {

  bool get_is_tree(const SubsetGraph &g) {
    // check connected components too
    if  (boost::num_edges(g)+1 != boost::num_vertices(g)) {
      IMP_LOG(TERSE, "Graph has " << boost::num_edges(g)
              << " and " << boost::num_vertices(g) << " and so is not a tree"
              << std::endl);
      return false;
    } else {
      std::vector<int> comp(boost::num_vertices(g));
      int cc= boost::connected_components(g, &comp[0]);
      IMP_LOG(TERSE, "Graph has " << cc
              << " components"
              << std::endl);
      return cc==1;
    }
  }
}

SubsetStates DominoSampler
::do_get_sample_states(const Subset &known_particles) const {
  IMP_LOG(TERSE, "Sampling with " << known_particles.size()
          << " particles as " << known_particles << std::endl);
  IMP_USAGE_CHECK(known_particles.size()>0, "No particles to sample");
  Pointer<RestraintSet> rs= get_model()->get_root_restraint_set();
    OptimizeRestraints ro(rs, get_particle_states_table());
  ParticlesTemp pt(known_particles.begin(), known_particles.end());
  SubsetGraph jt;
  if (has_sg_) {
    jt= sg_;
  } else {
    ParticlesTemp kppt(known_particles.begin(),
                       known_particles.end());
    jt= get_junction_tree(get_interaction_graph(rs,
                                                get_particle_states_table()));
  }
  IMP_IF_LOG(VERBOSE) {
    IMP_LOG(VERBOSE, "Subset graph is ");
    //std::ostringstream oss;
    IMP::internal::show_as_graphviz(jt, std::cout);
    //oss << std::endl;
    //IMP_LOG(TERSE, oss.str() << std::endl);
  }
  IMP_IF_CHECK(USAGE) {
    IMP::internal::Set<Particle*> used;
    boost::property_map< SubsetGraph, boost::vertex_name_t>::type subset_map=
      boost::get(boost::vertex_name, jt);
    for (unsigned int i=0; i< boost::num_vertices(jt); ++i) {
      Subset s= boost::get(subset_map, i);
      used.insert(s.begin(), s.end());
    }
    IMP_USAGE_CHECK(used.size()==known_particles.size(),
                    "Unexpected number of particles found in graph. Expected "
                    << known_particles.size() << " found " << used.size());
  }
  SubsetFilterTables sfts= get_subset_filter_tables_to_use(rs,
                                             get_particle_states_table());
  IMP_IF_LOG(TERSE) {
    IMP_LOG(TERSE, "Filtering with ");
    for (unsigned int i=0; i< sfts.size(); ++i) {
      IMP_LOG(TERSE, sfts[i]->get_name() << " ");
    }
    IMP_LOG(TERSE, std::endl);
  }
  IMP::internal::OwnerPointer<SubsetStatesTable> sst
    = DiscreteSampler::get_subset_states_table_to_use(sfts);

  SubsetStates final_solutions;
  if (get_is_tree(jt)) {
    ListSubsetFilterTable* lsft=NULL;
    if (csf_) {
      lsft= new ListSubsetFilterTable(get_particle_states_table());
      sfts.push_back(lsft);
    }
    internal::InferenceStatistics stats;
    final_solutions
      = internal::get_best_conformations(jt, 0,
                                         known_particles,
                                         sfts, sst, lsft, stats,
                                         get_maximum_number_of_states());
    if (lsft) {
      IMP_LOG(TERSE, lsft->get_ok_rate()
              << " were ok with the cross set filtering"
              << std::endl);
    }
    stats=internal::InferenceStatistics();
  } else {
    final_solutions
      = internal::loopy_get_best_conformations(jt, known_particles,
                                               sfts, sst,
                                               get_maximum_number_of_states());
  }
  return final_solutions;
}

void DominoSampler::set_subset_graph(const SubsetGraph &sg) {
  IMP_IF_CHECK(USAGE) {
    std::vector<int> comp(boost::num_vertices(sg));
    IMP_CHECK_CODE(int cc= boost::connected_components(sg, &comp[0]));
    IMP_USAGE_CHECK(cc==1, "Graph must have exactly one connected component."
                    << " It has " << cc);
  }
  sg_=sg;
  has_sg_=true;
}

void DominoSampler::do_show(std::ostream &out) const {
}

IMPDOMINO_END_NAMESPACE
