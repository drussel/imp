/**
 *  \file test_cache.cpp
 *  \brief A nullptr-initialized pointer to an \imp Object.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/base/cache.h>
#include <IMP/base/random.h>
#include <boost/random/uniform_int.hpp>
struct PlusOne {
  typedef int result_type;
  typedef int argument_type;
  result_type operator()(argument_type a) const {
    return a+1;
  }
};

int main(int, char *[]) {
  IMP::base::LRUCache<PlusOne> table(PlusOne(), 10);
  boost::uniform_int<> ui(0,20);
  for (unsigned int i=0; i< 10; ++i) {
    int in=i;
    int out=table.get(in);
    assert(in+1==out);
    using IMP::base::Showable;
    std::cout << Showable(table.get_keys()) << std::endl;
  }
  for (unsigned int i=0; i< 10; ++i) {
    int in=i;
    int out=table.get(in);
    assert(in+1==out);
    using IMP::base::Showable;
    std::cout << Showable(table.get_keys()) << std::endl;
  }
  assert(table.get_hit_rate()==.5);
  for (unsigned int i=0; i< 100; ++i) {
    int in=ui(IMP::base::random_number_generator);
    int out=table.get(in);
    assert(in+1==out);
    using IMP::base::Showable;
    std::cout << Showable(table.get_keys()) << std::endl;
  }
  return 0;
}