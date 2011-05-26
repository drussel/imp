/**
 *  \file RestraintGraph.cpp
 *  \brief creates a MRF from a set of particles and restraints
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/domino/internal/inference_utility.h>
#include <IMP/domino/assignment_tables.h>
#include <algorithm>
#include <boost/graph/copy.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/progress.hpp>


IMPDOMINO_BEGIN_INTERNAL_NAMESPACE
void fill_merged_assignments(const Subset &first_subset,
                             AssignmentContainer *first,
                             const Subset &second_subset,
                             AssignmentContainer *second,
                             const SubsetFilterTables &filters,
                             ListSubsetFilterTable *lsft,
                             InferenceStatistics &stats,
                             unsigned int max,
                             AssignmentContainer *out) {
  IMP::Pointer<AssignmentContainer> outp(out);
  IMP::internal::OwnerPointer<AssignmentContainer> firstp(first),
    secondp(second);
  IMP_FUNCTION_LOG;
  EdgeData ed= get_edge_data(first_subset, second_subset, filters);
  fill_union(first_subset, second_subset, first, second,
             ed, max, out);
  stats.add_subset(ed.union_subset, out);
  if (lsft) update_list_subset_filter_table(lsft, ed.union_subset,
                                            out);
  /*using namespace IMP;
  IMP_LOG(VERBOSE, "After merge, set is " << merged_subset
  << " and data is\n" << ret << std::endl);*/
}
void fill_leaf_assignments(const Subset &merged_subset,
                           const AssignmentsTable *states,
                           ListSubsetFilterTable *lsft,
                           InferenceStatistics &stats,
                           AssignmentContainer *out) {
  IMP::Pointer<AssignmentContainer> outp(out);
  IMP_FUNCTION_LOG;
  IMP_LOG(VERBOSE, "Looking at leaf " << merged_subset << std::endl);
  fill_node_data(merged_subset, states, out);
  if (lsft) update_list_subset_filter_table(lsft, merged_subset,
                                            out);
  //using namespace IMP;
  //IMP_LOG(VERBOSE, "Subset data is\n" << ret << std::endl);
  stats.add_subset(merged_subset, out);
}

namespace {

  void
  fill_best_conformations_internal(const MergeTree &jt,
                                  unsigned int root,
                                  const Subset& all,
                                  const AssignmentsTable *states,
                                  const SubsetFilterTables &filters,
                                  ListSubsetFilterTable *lsft,
                                  InferenceStatistics &stats,
                                  unsigned int max,
                                  boost::progress_display *progress,
                                  AssignmentContainer *out) {
    IMP::Pointer<AssignmentContainer> outp(out);
    typedef boost::property_map< MergeTree, boost::vertex_name_t>::const_type
      SubsetMap;
    typedef boost::graph_traits<MergeTree>::adjacency_iterator
      NeighborIterator;

    SubsetMap subset_map= boost::get(boost::vertex_name, jt);
    IMP_FUNCTION_LOG;
    std::pair<NeighborIterator, NeighborIterator> be
      = boost::adjacent_vertices(root, jt);
    if (std::distance(be.first, be.second)==0) {
      fill_leaf_assignments(boost::get(subset_map, root),
                            states, lsft, stats, out);
    } else {
      // merge
      IMP_INTERNAL_CHECK(std::distance(be.first, be.second)==2,
                         "Not a binary tree");
      int firsti=*be.first;
      int secondi= *(++be.first);
      IMP_NEW(PackedAssignmentContainer, cpd0, ());
      IMP_NEW(PackedAssignmentContainer, cpd1, ());
      fill_best_conformations_internal(jt, firsti, all, states,
                                          filters, lsft,
                                           stats, max, progress, cpd0);
      fill_best_conformations_internal(jt, secondi, all, states,
                                       filters, lsft,
                                       stats, max, progress,
                                       cpd1);
      fill_merged_assignments(boost::get(subset_map, firsti),
                              cpd0,
                              boost::get(subset_map, secondi),
                              cpd1,
                              filters, lsft, stats, max,
                              out);
      if (progress) {
        ++(*progress);
      }
    }
  }
}

void fill_best_conformations(const MergeTree &mt,
                             int root,
                             const Subset& all_particles,
                             const SubsetFilterTables &filters,
                             const AssignmentsTable *states,
                             ListSubsetFilterTable *lsft,
                             InferenceStatistics &stats,
                             unsigned int max,
                             AssignmentContainer *out) {
  IMP::Pointer<AssignmentContainer> outp(out);
  boost::scoped_ptr<boost::progress_display> progress;
  if (get_log_level() == PROGRESS) {
    progress.reset(new boost::progress_display(boost::num_vertices(mt)));
  }
  return fill_best_conformations_internal(mt, root,
                                         all_particles,states,
                                         filters,
                                         lsft,
                                         stats, max,
                                         progress.get(),
                                         out);
}


IMPDOMINO_END_INTERNAL_NAMESPACE
