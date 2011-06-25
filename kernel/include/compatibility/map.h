/**
 *  \file compatibility/map.h
 *  \brief Declare an efficient stl-compatible map
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_COMPATIBILITY_MAP_H
#define IMP_COMPATIBILITY_MAP_H

#include "compatibility_config.h"

// creates warnings in clang and we only use clang for diagnostics anyway
#if BOOST_VERSION > 103500 && !defined(__clang__)
#define IMP_USE_BOOST_MAP 1
#else
#define IMP_USE_BOOST_MAP 0
#endif

#if IMP_USE_BOOST_MAP
#include <boost/unordered_map.hpp>
#include "hash.h"
#else
#include <map>
#endif


IMP_BEGIN_COMPATIBILITY_NAMESPACE

/** This class chooses the best of STL compatible non-orderedf
    map available. This will, in general, be a hash map if it
    is available or std::map if it is not.
*/
template <class Key, class Data>
class map:
#if IMP_USE_BOOST_MAP
  public boost::unordered_map<Key, Data>
#else
  public std::map<Key, Data>
#endif
{
#if IMP_USE_BOOST_MAP
  typedef boost::unordered_map<Key, Data> P;
#else
  typedef std::map<Key, Data> P;
#endif
public:
  map(){}
  template <class It>
  map(It b, It e): P(b,e){}
};

IMP_END_COMPATIBILITY_NAMESPACE

#endif  /* IMP_COMPATIBILITY_MAP_H */
