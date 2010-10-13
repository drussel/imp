/**
 *  \file RestraintGraph.h
 *  \brief creates a MRF from a set of particles and restraints
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPDOMINO2_INTERNAL_INFERENCE_UTILITY_H
#define IMPDOMINO2_INTERNAL_INFERENCE_UTILITY_H

#include "../domino2_config.h"
#include "../utility.h"
#include "../subset_filters.h"
#include "../subset_states.h"
#include "../subset_graphs.h"

#include <IMP/Model.h>
#include <IMP/ScoreState.h>
#include <IMP/Restraint.h>

#include <vector>
#include <IMP/internal/map.h>
#include <sstream>

IMPDOMINO2_BEGIN_NAMESPACE
class SubsetEvaluatorTable;
class SubsetStatesTable;
IMPDOMINO2_END_NAMESPACE

IMPDOMINO2_BEGIN_INTERNAL_NAMESPACE


inline Subset get_intersection(const Subset &a, const Subset &b) {
  ParticlesTemp rs;
  std::set_intersection(a.begin(), a.end(),
                        b.begin(), b.end(),
                        std::back_inserter(rs));
  Subset ret(rs, true);
  return ret;
}

inline Subset get_union(const Subset &a, const Subset &b) {
  ParticlesTemp rs;
  std::set_union(a.begin(), a.end(),
                 b.begin(), b.end(),
                 std::back_inserter(rs));
  Subset ret(rs, true);
  return ret;
}

inline Subset get_difference(const Subset &a, const Subset &b) {
  ParticlesTemp rs;
  std::set_difference(a.begin(), a.end(),
                      b.begin(), b.end(),
                      std::back_inserter(rs));
  Subset ret(rs, true);
  return ret;
}



typedef IMP::internal::Map<Particle*, int> ParticleIndex;

inline ParticleIndex get_index(const Subset &s) {
  ParticleIndex ret;
  for (unsigned int i=0; i< s.size(); ++i) {
    ret[s[i]]=i;
  }
  return ret;
}


inline Ints get_index(const Subset &s, const Subset &subs) {
  Ints ret(subs.size(), -1);
  for (unsigned int i=0; i< subs.size(); ++i) {
    for (unsigned int j=0; j< s.size(); ++j) {
      if (s[j] == subs[i]) {
        ret[i]=j;
      }
    }
  }
  IMP_IF_CHECK(USAGE) {
    for (unsigned int i=0; i< ret.size(); ++i) {
      IMP_USAGE_CHECK(ret[i] >=0, "Second is not is not a subset of first.");
    }
  }
  return ret;
}

struct NodeData {
  SubsetStates subset_states;
};


inline std::ostream &operator<<(std::ostream &out, const NodeData &nd) {
  for (SubsetStates::const_iterator it = nd.subset_states.begin();
       it != nd.subset_states.end(); ++it) {
    out << *it << std::endl;
  }
  return out;
}

inline NodeData get_node_data(const Subset &s, const SubsetStatesTable *sst) {
  NodeData ret;
  ret.subset_states= sst->get_subset_states(s);
  return ret;
}


struct EdgeData {
  Subset intersection_subset;
  Subset union_subset;
  SubsetFilters filters;
};
typedef std::vector<EdgeData> EdgeDatas;

inline std::ostream &operator<<(std::ostream &out, const EdgeData &nd) {
  out << nd.intersection_subset << " " << nd.union_subset << std::endl;
  return out;
}

inline EdgeData get_edge_data(const Subset&s0,
                              const Subset&s1,
                              const SubsetFilterTables &sft) {
  EdgeData ret;
  ret.union_subset= get_union(s0, s1);
  ret.intersection_subset= get_intersection(s0,s1);
  Subsets excluded;
  excluded.push_back(s0);
  excluded.push_back(s1);
  for (unsigned int i=0; i< sft.size(); ++i) {
    SubsetFilter* sf= sft[i]->get_subset_filter(ret.union_subset,
                                                excluded);
    if (sf) {
      ret.filters.push_back(sf);
    }
  }
  return ret;
}



typedef boost::property_map< SubsetGraph, boost::vertex_name_t>::const_type
SubsetMap;

typedef boost::graph_traits<SubsetGraph>::adjacency_iterator NeighborIterator;

//! return true if the two states are equal on the entries in the lists
inline bool get_are_equal(const SubsetState &ss0,
                          const Ints &i0,
                          const SubsetState &ss1,
                          const Ints &i1) {
  IMP_USAGE_CHECK(i0.size() == i1.size(), "Sizes don't match");
  for (unsigned int i=0; i< i0.size(); ++i) {
    if (ss0[i0[i]] != ss1[i1[i]]) return false;
  }
  return true;
}


IMPDOMINO2EXPORT
SubsetState get_merged_subset_state(const Subset &s,
                                      const SubsetState &ss0,
                                      const Ints &i0,
                                      const SubsetState &ss1,
                                    const Ints &i1) ;

IMPDOMINO2EXPORT NodeData
get_union(const Subset &s0, const Subset &s1,
          const NodeData &nd0, const NodeData &nd1,
          const EdgeData &ed);

inline void update_list_subset_filter_table(ListSubsetFilterTable *lsft,
                                     const Subset &s,
                                     const SubsetStates &states) {
  for (unsigned int i=0; i< s.size(); ++i) {
    boost::dynamic_bitset<> bs(lsft->get_number_of_particle_states(s[i]));
    bs.reset();
    for (unsigned int j=0; j< states.size(); ++j) {
      bs.set(states[j][i]);
    }
    lsft->intersect(s[i], bs);
  }
}


IMPDOMINO2_END_INTERNAL_NAMESPACE

#endif  /* IMPDOMINO2_INTERNAL_INFERENCE_UTILITY_H */
