/**
 *  \file domino2/subset_filters.h
 *  \brief A beyesian infererence-based sampler.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPDOMINO2_SUBSET_FILTERS_H
#define IMPDOMINO2_SUBSET_FILTERS_H

#include "particle_states.h"
#include "SubsetState.h"
#include "particle_states.h"
#include "internal/restraint_evaluator.h"
#include "Subset.h"
#include "domino2_macros.h"
#include "domino2_config.h"
#include <IMP/Object.h>
#include <IMP/Pointer.h>
#include <IMP/internal/map.h>
#include <IMP/Configuration.h>
#include <IMP/Model.h>
#include <IMP/macros.h>
#include <boost/dynamic_bitset.hpp>


IMPDOMINO2_BEGIN_NAMESPACE

/** An instance of this type is created by the
    SubsetFilterTable::get_subset_filter method(). It's job
    is to reject some of the SubsetStates correspinding to the
    Subset it was created with. It has one
    method of interest, get_is_ok() which true if the state
    passes the filter.

    The passed SubsetState has the particles ordered in the
    same order as they were in the Subset that was passed to the
    table in order to create the filter.
*/
class IMPDOMINO2EXPORT SubsetFilter: public Object {
public:
  SubsetFilter(std::string name= "SubsetFilter %1%");
  //! Return true if the given state passes this filter for the Subset
  //! it was created with
  virtual bool get_is_ok(const SubsetState& state) const=0;
  //! The strength is a rough metric of how this filter restricts the subset
  /** It is still kind of nebulous, but as a rough guide, it should be
      the fraction of the states that are eliminated by the filter.
   */
  virtual double get_strength() const=0;
  virtual ~SubsetFilter();
};

IMP_OBJECTS(SubsetFilter, SubsetFilters);


/** A SubsetFilterTable class which produces SubsetFilter objects upon
    demand. When the get_subset_filter() method is called, it is passed
    the Subset that is to be filtered. It is also passed subsets of
    that Subset which have previously been filtered (and so don't need
    to be checked again).

    For example, if the passed set is {a,b,c} and the prior_subsets
    are {a,b} and {b,c}, then only properties than involve a and c need
    to be checked, as ones involve a and b and b and c have already been
    checked previously.
*/
class IMPDOMINO2EXPORT SubsetFilterTable: public Object {
 public:
  SubsetFilterTable(std::string name="SubsetFilterTable%1%"): Object(name){}
  /** Return a SubsetFilter which acts on the Subset s, given that all
      the prior_subsets have already been filtered.
   */
  virtual SubsetFilter* get_subset_filter(const Subset &s,
                                          const Subsets &prior_subsets) const=0;
  virtual ~SubsetFilterTable();
};

IMP_OBJECTS(SubsetFilterTable, SubsetFilterTables);

class RestraintScoreSubsetFilterTable;

/** A restraint score based SubsetFilter.
    See RestraintScoreSubsetFilterTable.
 */
class IMPDOMINO2EXPORT RestraintScoreSubsetFilter: public SubsetFilter {
  Pointer<const internal::ModelData> keepalive_;
  const internal::SubsetData &data_;
  double max_;
  friend class RestraintScoreSubsetFilterTable;
  RestraintScoreSubsetFilter(const internal::ModelData *t,
                             const internal::SubsetData &data,
                             double max):
    SubsetFilter("Restraint score filter"),
    keepalive_(t), data_(data),
    max_(max) {
  }
public:
  IMP_SUBSET_FILTER(RestraintScoreSubsetFilter);
};


//! Filter a configuration of the subset using the Model thresholds
/** This filter table creates filters using the maximum scores
    set in the Model for various restraints.
 */
class IMPDOMINO2EXPORT RestraintScoreSubsetFilterTable:
  public SubsetFilterTable {
  struct StatsPrinter:public Pointer<internal::ModelData>{
  public:
    StatsPrinter(internal::ModelData *mset):
      Pointer<internal::ModelData>(mset){}
    ~StatsPrinter();
  };
  StatsPrinter mset_;
 public:
  RestraintScoreSubsetFilterTable(RestraintSet *rs,
                                  ParticleStatesTable *pst);
  RestraintScoreSubsetFilterTable(Model *rs,
                                  ParticleStatesTable *pst);
  IMP_SUBSET_FILTER_TABLE(RestraintScoreSubsetFilterTable);
};

IMP_OBJECTS(RestraintScoreSubsetFilterTable,
            RestraintScoreSubsetFilterTables);

/** \brief Do not allow two particles to be in the same state.

    If a ParticleStatesTable is passed, then two particles cannot
    be in the same state if they have the same ParticleStates,
    otherwise, if a ParticlePairs is passed then pairs found in the
    list are not allowed to have the same state index.
 */
class IMPDOMINO2EXPORT PermutationSubsetFilterTable:
  public SubsetFilterTable {
  Pointer<ParticleStatesTable> pst_;
  const ParticlePairsTemp pairs_;
public:
  PermutationSubsetFilterTable(ParticleStatesTable *pst);
  PermutationSubsetFilterTable(const ParticlePairsTemp &pairs);
  IMP_SUBSET_FILTER_TABLE(PermutationSubsetFilterTable);
};

IMP_OBJECTS(PermutationSubsetFilterTable,
            PermutationSubsetFilterTables);


/** \brief Force two particles to be in the same state.

    If a ParticleStatesTable is passed, then two particles must
    be in the same state if they have the same ParticleStates,
    otherwise, if a ParticlePairs is passed then pairs found in the
    list are excluded.
 */
class IMPDOMINO2EXPORT EqualitySubsetFilterTable:
  public SubsetFilterTable {
  Pointer<ParticleStatesTable> pst_;
  const ParticlePairsTemp pairs_;
public:
  EqualitySubsetFilterTable(ParticleStatesTable *pst);
  EqualitySubsetFilterTable(const ParticlePairsTemp &pairs);
  IMP_SUBSET_FILTER_TABLE(EqualitySubsetFilterTable);
};

IMP_OBJECTS(EqualitySubsetFilterTable,
            EqualitySubsetFilterTables);


/** \brief Maintain an explicit list of what states each particle
    is allowed to have.

    This filter maintains a list for each particle storing whether
    that particle is allowed to be in a certain state or not.
 */
class IMPDOMINO2EXPORT ListSubsetFilterTable:
  public SubsetFilterTable {
 public:
#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  mutable IMP::internal::Map<Particle*,int > map_;
  mutable std::vector< boost::dynamic_bitset<> > states_;
  Pointer<ParticleStatesTable> pst_;
  mutable double num_ok_, num_test_;
  int get_index(Particle*p) const;
  void intersect(Particle*p, const boost::dynamic_bitset<> &s);
#endif
 public:
  ListSubsetFilterTable(ParticleStatesTable *pst);
  double get_ok_rate() const {
    return num_ok_/num_test_;
  }
  unsigned int get_number_of_particle_states(Particle *p) const {
    int i= get_index(p);
    return states_[i].size();
  }
  IMP_SUBSET_FILTER_TABLE(ListSubsetFilterTable);
};

IMP_OBJECTS(ListSubsetFilterTable,
            ListSubsetFilterTables);


IMPDOMINO2_END_NAMESPACE

#endif  /* IMPDOMINO2_SUBSET_FILTERS_H */
