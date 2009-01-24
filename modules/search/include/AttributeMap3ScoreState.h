/**
 * \file AttributeMap3ScoreState.h
 * \brief Maintain a map between attribute values and particles.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 * WARNING: This header is generated by a script from
*  modules/search/tools/templates/map.hpp
 */

#ifndef IMPSEARCH_ATTRIBUTE_MAP_3SCORE_STATE_H
#define IMPSEARCH_ATTRIBUTE_MAP_3SCORE_STATE_H

#include "config.h"
#include "internal/search_version_info.h"
#include "internal/MapHelper.h"
#include <IMP/SingletonContainer.h>
#include <IMP/core/config.h>
#include <IMP/Particle.h>
#include <IMP/ScoreState.h>

#include <iostream>
#include <algorithm>

// switch to using tuples rather than multiple arguments
#include <boost/tuple/tuple.hpp>

namespace IMP {
  // for swig
  class SingletonContainer;
}

IMPSEARCH_BEGIN_NAMESPACE

//! Maintain a mapping between attribute values and particles.
/** This class allows you to look of which particles from a set have
    attributes which a particular set of values. That is, if you
    initialize the map with the FloatKey("residue search") attribute,
    you quickly find the particles with that search.

    The template arguments are the types of the attributes you wish
    to build the map on. They can be of type Int, String, or Particle.
*/
template <class AttributeType0,
class AttributeType1,
class AttributeType2>
class AttributeMap3ScoreState : public ScoreState
{
  typedef
  typename boost::tuple<typename internal::GetKey<AttributeType0>::Key,
                                typename internal::GetKey<AttributeType1>::Key,
                                typename internal::GetKey<AttributeType2>::Key>
    Keys;
  typedef typename boost::tuple<AttributeType0,
                                AttributeType1,
                                AttributeType2>
    Values;

  struct Bin {
    Values values;
    Particles particles;
    Bin(const Values &vs): values(vs){}
    bool operator<(const Bin &o) const {
      if (boost::get<0>(values) < boost::get<0>(o.values)) return true;
      else if (boost::get<0>(values) > boost::get<0>(o.values)) return false;
      else if (boost::get<1>(values) < boost::get<1>(o.values)) return true;
      else if (boost::get<1>(values) > boost::get<1>(o.values)) return false;
      else if (boost::get<2>(values) < boost::get<2>(o.values)) return true;
      else if (boost::get<2>(values) > boost::get<2>(o.values)) return false;
      return false;
    }
    bool operator==(const Bin &o) const {
      return boost::get<0>(values) == boost::get<0>(o.values)
           && boost::get<1>(values) == boost::get<1>(o.values)
           && boost::get<2>(values) == boost::get<2>(o.values);
    }
  };
  typedef std::vector<Bin> Map;
  Map map_;
  Pointer<IMP::SingletonContainer> pc_;
  Keys keys_;

  typename Map::const_iterator get_iterator(const Values &vs) const {
    return std::lower_bound(map_.begin(), map_.end(), Bin(vs));
  }

  const Particles &get_particles_internal(const Values &vs) const {
    static Particles empty;
    Bin bvs(vs);
    typename Map::const_iterator it = get_iterator(vs);
    if (it!= map_.end() && *it == bvs) {
      return it->particles;
    } else {
      return empty;
    }
  }

  Particles &get_particles_nonconst(const Values &vs) {
    Bin bvs(vs);
    typename Map::iterator it = std::lower_bound(map_.begin(), map_.end(),
                                                 bvs);
    IMP_assert(it != map_.end(), "Value not found in map on update");
    IMP_assert(*it == bvs, "Values to not match on map update");
    return it->particles;
  }

  Values get_values(Particle *p) const {
    return Values(p->get_value(boost::get<0>(keys_)),
                  p->get_value(boost::get<1>(keys_)),
                  p->get_value(boost::get<2>(keys_)));
  }


public:
  typedef typename internal::GetKey<AttributeType0>::Key Key0;
  typedef AttributeType0 Value0;
  typedef typename internal::GetKey<AttributeType1>::Key Key1;
  typedef AttributeType1 Value1;
  typedef typename internal::GetKey<AttributeType2>::Key Key2;
  typedef AttributeType2 Value2;

  //! Create the score state searching one attribute.
  /** The first argument is the list of particles to search,
      the remainder are the attributes to use. The number of key arguments
      must match the number of provided value types in the template
      parameter list.
  */
  AttributeMap3ScoreState(IMP::SingletonContainer* pc,
                          Key0 k0,
                          Key1 k1,
                          Key2 k2): pc_(pc) {
    keys_= Keys( k0,  k1,  k2);
  }


  //! Find particles which match the given attributes
  const Particles &get_particles(Value0 v0,
                                 Value1 v1,
                                 Value2 v2)  const {
    Values vs( v0,  v1,  v2);
    return get_particles_internal(vs);
  }

  //! Find particles which match the given attributes
  /** It is an error if there is not a unique particle.
   */
  const Particle* get_particle(Value0 v0,
                                 Value1 v1,
                                 Value2 v2)  const {
    Values vs( v0,  v1,  v2);
    IMP_assert(get_particles_internal(vs).size() == 1,
               "There must be exactly one particle.");
    return get_particles_internal(vs)[0];
  }

  //! Find all the particles in the range
  /** The range is defined via a lexicographical ordering. The first set
      of arguments is the lower bound and the second set is the upper bound.
      The range is half open (so the upper bound is not included).
   */
  const Particles get_particles(Value0 v0,
                                 Value1 v1,
                                 Value2 v2,
                                Value0 vu0,
                                 Value1 vu1,
                                 Value2 vu2) const {
    Values vs0( v0,  v1,  v2), vs1( vu0,  vu1,  vu2);
    typename Map::const_iterator it =get_iterator(vs0), ite= get_iterator(vs1);
    Particles ret;
    for (; it != ite; ++it) {
      ret.insert(ret.end(), it->particles.begin(), it->particles.end());
    }
    return ret;
  }

  /* We can't use the macro since SWIG won't instantiate do_before_evaluate
     since it doesn't call it directly.
  */
  virtual void show(std::ostream &out=std::cout) const {
    out << "AttributeMapScoreState on "
        << pc_ << std::endl;
  }
  virtual IMP::VersionInfo get_version_info() const {
    return internal::search_version_info;
  }


  /* make this public so SWIG instantiates it*/
  virtual void do_before_evaluate() {
    map_.clear();
    map_.reserve(pc_->get_number_of_particles());
    for (IMP::SingletonContainer::ParticleIterator
           it= pc_->particles_begin();
         it != pc_->particles_end(); ++it) {
      Values vs= get_values(*it);
      map_.push_back(vs);
    }
    std::sort(map_.begin(), map_.end());
    map_.erase(std::unique(map_.begin(), map_.end()), map_.end());
    for (IMP::SingletonContainer::ParticleIterator
           it= pc_->particles_begin();
         it != pc_->particles_end(); ++it) {
      Values vs= get_values(*it);
      get_particles_nonconst(vs).push_back(*it);
    }
  }

  ~AttributeMap3ScoreState(){}

};

IMPSEARCH_END_NAMESPACE

#endif  /* IMPSEARCH_ATTRIBUTE_MAP_3SCORE_STATE_H */
