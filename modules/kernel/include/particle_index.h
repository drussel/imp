/**
 *  \file IMP/particle_index.h
 *  \brief Various general useful functions for IMP.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_PARTICLE_INDEX_H
#define IMPKERNEL_PARTICLE_INDEX_H

#include "kernel_config.h"
#include <boost/array.hpp>
#include <IMP/base/base_macros.h>
#include <IMP/base/Index.h>

IMP_BEGIN_NAMESPACE
#ifndef IMP_DOXYGEN

class ParticleIndexTag{};
typedef base::Index<ParticleIndexTag> ParticleIndex;
typedef vector<ParticleIndex> ParticleIndexes;



//! A class to store a tuple of particles.
/** Only the constructor with the correct number of arguments for the
    dimensionality can be used.

    \note ParticleTuple objects are ordered.
*/
template <unsigned int D>
class ParticleIndexTuple
#if !defined(SWIG)
: public boost::array<ParticleIndex, D>
#endif
{
  typedef boost::array<ParticleIndex, D> P;
  int compare(const ParticleIndexTuple<D> &o) const {
    for (unsigned int i=0;i<D; ++i) {
      if (P::operator[](i) < o[i]) return -1;
      else if (P::operator[](i) > o[i]) return 1;
    }
    return 0;
  }
public:
  static unsigned int get_dimension() {return D;};
  ParticleIndexTuple(){
    for (unsigned int i=0; i< D; ++i) {
      P::operator[](i)=base::get_invalid_index<ParticleIndexTag>();
    }
  }
  ParticleIndexTuple(ParticleIndex x, ParticleIndex y) {
    IMP_USAGE_CHECK(D==2, "Need " << D << " to construct a "
              << D << "-tuple.");
    P::operator[](0) = x;
    P::operator[](1) = y;
  }
  ParticleIndexTuple(ParticleIndex x, ParticleIndex y, ParticleIndex z) {
    IMP_USAGE_CHECK(D==3, "Need " << D << " to construct a "
              << D << "-tuple.");
    P::operator[](0) = x;
    P::operator[](1) = y;
    P::operator[](2) = z;
  }
  ParticleIndexTuple(ParticleIndex x0, ParticleIndex x1,
                     ParticleIndex x2, ParticleIndex x3) {
    IMP_USAGE_CHECK(D==4, "Need " << D << " to construct a "
              << D << "-tuple.");
    P::operator[](0) = x0;
    P::operator[](1) = x1;
    P::operator[](2) = x2;
    P::operator[](3) = x3;
  }
  /*ParticleIndex *get(unsigned int i) const {
    return P::operator[](i);
    }*/
  IMP_HASHABLE_INLINE(ParticleIndexTuple<D>, std::size_t seed = 0;
               for (unsigned int i=0; i< D; ++i) {
                 boost::hash_combine(seed,
                                     P::operator[](i));
               }
               return seed;);
  IMP_COMPARISONS(ParticleIndexTuple<D>);
  std::string get_name() const {
    std::ostringstream out;
    out << "(";
    for (unsigned int i=0; i< D; ++i) {
      if (i>0) {
        out << ", ";
      }
      out << P::operator[](i);
    }
    out << ")";
    return out.str();
  }
  IMP_SHOWABLE_INLINE(ParticleIndexTuple, {
      out << get_name();
    });
};

#if !defined(SWIG)
template<unsigned int D>
std::ostream &operator<<(std::ostream &out,
                         const ParticleIndexTuple<D> &d) {
  d.show(out);
  return out;
}
#endif

typedef ParticleIndexTuple<2> ParticleIndexPair;
typedef ParticleIndexTuple<3> ParticleIndexTriplet;
typedef ParticleIndexTuple<4> ParticleIndexQuad;

IMP_VALUES(ParticleIndexPair, ParticleIndexPairs);
IMP_VALUES(ParticleIndexTriplet, ParticleIndexTriplets);
IMP_VALUES(ParticleIndexQuad, ParticleIndexQuads);



#endif
IMP_END_NAMESPACE

#endif  /* IMPKERNEL_PARTICLE_INDEX_H */
