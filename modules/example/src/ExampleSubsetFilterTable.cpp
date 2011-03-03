/**
 *  \file ExampleSubsetFilterTable.cpp
 *  \brief A Score on the distance between a pair of particles.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#include <IMP/example/ExampleSubsetFilterTable.h>
#include <iterator>

IMPEXAMPLE_BEGIN_NAMESPACE

namespace {
  // the actual filter object
  class  ExampleSubsetFilter: public domino::SubsetFilter {
    Ints values_;
  public:
    ExampleSubsetFilter(const Ints &vs): values_(vs) {
    }
    IMP_SUBSET_FILTER(ExampleSubsetFilter);
  };
  bool ExampleSubsetFilter::get_is_ok(const domino::SubsetState &state) const{
      IMP_OBJECT_LOG;
      for (unsigned int i=0; i< state.size(); ++i) {
        if (values_[i] >=0 && state[i] != values_[i]) return false;
      }
      return true;
    }
  void ExampleSubsetFilter::do_show(std::ostream &out) const {
    std::copy(values_.begin(), values_.end(),
              std::ostream_iterator<int>(out, " "));
    out << std::endl;
  }
}

ExampleSubsetFilterTable::ExampleSubsetFilterTable(const ParticlesTemp &ps) {
  for (unsigned int i=0; i< ps.size(); ++i) {
    index_[ps[i]]=i;
  }
}

domino::SubsetFilter*
ExampleSubsetFilterTable::get_subset_filter(const domino::Subset&s,
                                  const domino::Subsets &) const {
  /* In general, the excluded subsets are subsets that have already
   been filtered.  The table is free to reduce its calculations based
   on that knowledge. Here we are lazy and do not. */
  Ints values(s.size(), -1);
  for (unsigned int i=0; i< s.size(); ++i) {
    values[i]= index_.find(s[i])->second;
  }
  return new ExampleSubsetFilter(values);
}

double ExampleSubsetFilterTable::get_strength(const domino::Subset&,
                                  const domino::Subsets &) const {
  // this is a really strong filter
  // for a weaker one, the number should be roughly the fraction
  // of states that the filter eliminates
  return 1;
}

void ExampleSubsetFilterTable::do_show(std::ostream &) const {
}

IMPEXAMPLE_END_NAMESPACE
