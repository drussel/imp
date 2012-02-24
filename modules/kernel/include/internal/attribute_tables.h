/**
 *  \file interna/constants.h    \brief Various useful constants.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_INTERNAL_ATTRIBUTE_TABLES_H
#define IMPKERNEL_INTERNAL_ATTRIBUTE_TABLES_H

#include "../kernel_config.h"
#include <boost/dynamic_bitset.hpp>
#include "../Key.h"
#include "../utility.h"
#include <IMP/base/exception.h>
#include <IMP/base/log.h>
#include <IMP/algebra/Sphere3D.h>

#define IMP_CHECK_MASK(mask, particle_index, message)                   \
  IMP_USAGE_CHECK(!mask || mask->size() >                               \
                  get_as_unsigned_int(particle_index),                  \
                  "For some reason the mask is too small.");            \
  IMP_USAGE_CHECK(!mask || (*mask)[get_as_unsigned_int(particle_index)], \
                  message << " at particle " << particle_index)




IMP_BEGIN_INTERNAL_NAMESPACE

typedef boost::dynamic_bitset<> Mask;

template <class Traits>
class BasicAttributeTable {
public:
  typedef typename Traits::Key Key;
private:
  vector<typename Traits::Container > data_;
#if IMP_BUILD < IMP_FAST
  Mask *read_mask_, *write_mask_, *add_remove_mask_;
#endif
  compatibility::set<Key> caches_;

   void do_add_attribute(Key k, ParticleIndex particle,
                           typename Traits::PassValue value) {
     using IMP::operator<<;
     IMP_USAGE_CHECK(Traits::get_is_valid(value), "Can't set to invalid value: "
                     << value << " for attribute " << k);
    if (data_.size() <= k.get_index()) {
      data_.resize(k.get_index()+1);
    }
    base::resize_to_fit(data_[k.get_index()], particle,
                        Traits::get_invalid());
    data_[k.get_index()][particle]=value;
  }
public:
  void swap_with(BasicAttributeTable<Traits> &o) {
    IMP_SWAP_MEMBER(data_);
    IMP_SWAP_MEMBER(caches_);
  }

#if IMP_BUILD < IMP_FAST
  void set_masks(Mask *read_mask,
                 Mask *write_mask,
                 Mask *add_remove_mask) {
    read_mask_=read_mask;
    write_mask_=write_mask;
    add_remove_mask_=add_remove_mask;
  }
#endif

  BasicAttributeTable()
#if IMP_BUILD < IMP_FAST
  : read_mask_(nullptr), write_mask_(nullptr), add_remove_mask_(nullptr)
#endif
  {}

  void add_attribute(Key k, ParticleIndex particle,
                     typename Traits::PassValue value) {
    IMP_CHECK_MASK(add_remove_mask_, particle,
                   "Changing the attributes is not permitted now");
    do_add_attribute(k, particle, value);
  }
  void add_cache_attribute(Key k, ParticleIndex particle,
                           typename Traits::PassValue value) {
    caches_.insert(k);
    do_add_attribute(k, particle, value);
  }
  void clear_caches(ParticleIndex particle) {
    for (typename compatibility::set<Key>::const_iterator it=caches_.begin();
         it != caches_.end(); ++it) {
      if (data_.size() > it->get_index()
          && data_[it->get_index()].size() > get_as_unsigned_int(particle)) {
        data_[it->get_index()][particle]
            = Traits::get_invalid();
      }
    }
  }
  void remove_attribute(Key k, ParticleIndex particle) {
    IMP_CHECK_MASK(add_remove_mask_, particle,
                   "Changing the attributes is not permitted now");
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Can't remove attribute if it isn't there");
    data_[k.get_index()][particle]=Traits::get_invalid();
  }
  bool get_has_attribute(Key k, ParticleIndex particle) const {
    if (data_.size() <= k.get_index()) return false;
    else if (data_[k.get_index()].size()
             <= get_as_unsigned_int(particle)) return false;
    else return Traits::get_is_valid(data_[k.get_index()]
                                     [particle]);
  }
  void set_attribute(Key k, ParticleIndex particle,
                     typename Traits::PassValue value) {
#if IMP_BUILD < IMP_FAST
    if (caches_.find(k)==caches_.end()) {
      IMP_CHECK_MASK( write_mask_, particle,
                      "Changing the attribute values is not permitted now");
    }
#endif
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Setting invalid attribute: " << k
                    << " of particle " << particle);
    data_[k.get_index()][particle]= value;
  }
  typename Traits::PassValue get_attribute(Key k,
                                           ParticleIndex particle,
                                           bool checked=true) const {
    if (checked) {
      IMP_CHECK_MASK(read_mask_, particle,
                     "Reading the attribute values is not permitted now");
    }
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Requested invalid attribute: " << k
                    << " of particle " << particle);
    return data_[k.get_index()][particle];
  }
  typename Traits::Container::reference access_attribute(Key k,
                                                       ParticleIndex particle) {
    IMP_CHECK_MASK(write_mask_, particle,
                   "Writing the attribute values is not permitted now");
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Requested invalid attribute: " << k
                    << " of particle " << particle);
    return data_[k.get_index()][particle];
  }
  std::pair<typename Traits::Value,
            typename Traits::Value> get_range_internal(Key k) const {
    std::pair<typename Traits::Value,
              typename Traits::Value>  ret;
    IMP_USAGE_CHECK(data_.size() > k.get_index()
                    && data_[k.get_index()].size()!=0,
                    "Cannot request range of an unused key.");
    bool init=false;
    for (unsigned int i=0; i< data_[k.get_index()].size(); ++i) {
      if (Traits::get_is_valid(data_[k.get_index()][ParticleIndex(i)])) {
        if (!init) {
          ret.first= data_[k.get_index()][ParticleIndex(i)];
          ret.second= data_[k.get_index()][ParticleIndex(i)];
          init=true;
        } else {
          ret.first=Traits::min(ret.first,
                                data_[k.get_index()][ParticleIndex(i)]);
          ret.second=Traits::max(ret.second,
                                 data_[k.get_index()][ParticleIndex(i)]);
        }
      }
    }
    return ret;
  }
  void clear_attributes(ParticleIndex particle) {
    IMP_CHECK_MASK(add_remove_mask_, particle,
                   "Clearing the attribute values is not permitted now");
    for (unsigned int i=0; i< data_.size(); ++i) {
      if (data_[i].size() > get_as_unsigned_int(particle)) {
        data_[i][particle]= Traits::get_invalid();
      }
    }
  }

  IMP::vector<Key>
  get_attribute_keys(ParticleIndex particle) const {
    vector<Key> ret;
    for (unsigned int i=0; i< data_.size(); ++i) {
      if (data_[i].size() > get_as_unsigned_int(particle)
          && Traits::get_is_valid(data_[i][particle])) {
        ret.push_back(Key(i));
      }
    }
    return ret;
  }
  void fill(typename Traits::PassValue value) {
    for (unsigned int i=0; i< data_.size(); ++i) {
      std::fill(data_[i].begin(), data_[i].end(), value);
    }
  }
  unsigned int size() const {return data_.size();}
  unsigned int size(unsigned int i) const {return data_[i].size();}
};
IMP_SWAP_1(BasicAttributeTable);



class FloatAttributeTable {
  //vector<algebra::Sphere3D> spheres_;
  //vector<algebra::Sphere3D> sphere_derivatives_;
  base::IndexVector<ParticleIndexTag, algebra::Sphere3D> spheres_;
  base::IndexVector<ParticleIndexTag, algebra::Sphere3D> sphere_derivatives_;
  base::IndexVector<ParticleIndexTag, algebra::Vector3D> internal_coordinates_;
  base::IndexVector<ParticleIndexTag, algebra::Vector3D>
  internal_coordinate_derivatives_;
  BasicAttributeTable<internal::FloatAttributeTableTraits> data_;
  BasicAttributeTable<internal::FloatAttributeTableTraits> derivatives_;
  // make use bitset
  BasicAttributeTable<internal::BoolAttributeTableTraits> optimizeds_;
  FloatRanges ranges_;
#if IMP_BUILD < IMP_FAST
  Mask *read_mask_, *write_mask_,*add_remove_mask_,
                *read_derivatives_mask_, *write_derivatives_mask_;
#endif
  algebra::Sphere3D get_invalid_sphere() const {
    double iv= internal::FloatAttributeTableTraits::get_invalid();
    algebra::Sphere3D ivs(algebra::Vector3D(iv, iv, iv), iv);
    return ivs;
  }

public:
  void swap_with(FloatAttributeTable&o) {
    using IMP::swap;
    using std::swap;
    IMP_SWAP_MEMBER(spheres_);
    IMP_SWAP_MEMBER(sphere_derivatives_);
    IMP_SWAP_MEMBER(data_);
    IMP_SWAP_MEMBER(derivatives_);
    IMP_SWAP_MEMBER(optimizeds_);
    IMP_SWAP_MEMBER(internal_coordinates_);
    IMP_SWAP_MEMBER(internal_coordinate_derivatives_);
  }
  FloatAttributeTable()
#if IMP_BUILD < IMP_FAST
  : read_mask_(nullptr), write_mask_(nullptr),
    add_remove_mask_(nullptr),
    read_derivatives_mask_(nullptr),
    write_derivatives_mask_(nullptr)
#endif
{}
#if IMP_BUILD < IMP_FAST
  void set_masks(Mask *read_mask,
                 Mask *write_mask,
                 Mask *add_remove_mask,
                 Mask *read_derivatives_mask,
                 Mask *write_derivatives_mask) {
    data_.set_masks(read_mask, write_mask, add_remove_mask);
    derivatives_.set_masks(read_derivatives_mask, write_derivatives_mask,
                           add_remove_mask);
    optimizeds_.set_masks(read_mask, write_mask, add_remove_mask);
    read_mask_=read_mask;
    write_mask_=write_mask;
    add_remove_mask_=add_remove_mask;
    read_derivatives_mask_=read_derivatives_mask;
    write_derivatives_mask_=write_derivatives_mask;
  }
#endif

  // make sure you know what you are doing
  algebra::Sphere3D& get_sphere(ParticleIndex particle) {
    IMP_CHECK_MASK(read_mask_, particle,
                   "Reading the attribute values is not permitted now");
    return spheres_[particle];
  }

  algebra::Vector3D& get_internal_coordinates(ParticleIndex particle) {
    IMP_CHECK_MASK(read_mask_, particle,
                   "Reading the attribute values is not permitted now");
    IMP_USAGE_CHECK(internal_coordinates_[particle][0]
                    !=internal::FloatAttributeTableTraits::get_invalid(),
                    "No internal coordinates");
    IMP_USAGE_CHECK(internal_coordinates_[particle][1]
                    !=internal::FloatAttributeTableTraits::get_invalid(),
                    "No internal coordinates");
    IMP_USAGE_CHECK(internal_coordinates_[particle][2]
                    !=internal::FloatAttributeTableTraits::get_invalid(),
                    "No internal coordinates");
    return internal_coordinates_[particle];
  }


  void add_to_coordinate_derivatives(ParticleIndex particle,
                                     const algebra::Vector3D &v,
                                     const DerivativeAccumulator &da) {
    IMP_CHECK_MASK(write_derivatives_mask_, particle,
                   "Changing the attribute derivatives is not permitted now");
    IMP_USAGE_CHECK(get_has_attribute(FloatKey(0), particle),
                    "Particle does not have coordinates");
    sphere_derivatives_[particle][0]+=da(v[0]);
    sphere_derivatives_[particle][1]+=da(v[1]);
    sphere_derivatives_[particle][2]+=da(v[2]);
  }

  void add_to_internal_coordinate_derivatives(ParticleIndex particle,
                                     const algebra::Vector3D &v,
                                     const DerivativeAccumulator &da) {
    IMP_CHECK_MASK(write_derivatives_mask_, particle,
                   "Changing the attribute derivatives is not permitted now");
    IMP_USAGE_CHECK(get_has_attribute(FloatKey(0), particle),
                    "Particle does not have coordinates");
    internal_coordinate_derivatives_[particle][0]
        +=da(v[0]);
    internal_coordinate_derivatives_[particle][1]
        +=da(v[1]);
    internal_coordinate_derivatives_[particle][2]
        +=da(v[2]);
  }

  const algebra::Vector3D&
  get_coordinate_derivatives(ParticleIndex particle) const {
    IMP_CHECK_MASK(read_derivatives_mask_, particle,
                   "Reading the attribute derivatives is not permitted now");
    IMP_USAGE_CHECK(get_has_attribute(FloatKey(0), particle),
                    "Particle does not have coordinates");
    return sphere_derivatives_[particle].get_center();
    }
  void zero_derivatives() {
    /*std::fill(sphere_derivatives_.begin(), sphere_derivatives_.end(),
      algebra::Sphere3D(algebra::Vector3D(0,0,0), 0));*/
    // make more efficient
    std::fill(sphere_derivatives_.begin(),
              sphere_derivatives_.end(),
              algebra::Sphere3D(algebra::Vector3D(0,0,0),0));
    std::fill(internal_coordinate_derivatives_.begin(),
              internal_coordinate_derivatives_.end(),
              algebra::Vector3D(0,0,0));
    derivatives_.fill(0);
  }
  void clear_caches(ParticleIndex ) {
  }
  void add_cache_attribute(FloatKey , ParticleIndex, double ){
    IMP_NOT_IMPLEMENTED;
  }
  void remove_attribute(FloatKey k, ParticleIndex particle) {
    IMP_CHECK_MASK(add_remove_mask_, particle,
                   "Changing the attributes is not permitted now");
    if (k.get_index() < 4) {
      IMP_CHECK_MASK(add_remove_mask_, particle,
                   "Changing attributes is not permitted now");
      spheres_[particle][k.get_index()]
        = internal::FloatAttributeTableTraits::get_invalid();
      sphere_derivatives_[particle][k.get_index()]
        = internal::FloatAttributeTableTraits::get_invalid();
    } else if (k.get_index() < 7) {
      IMP_CHECK_MASK(add_remove_mask_, particle,
                   "Changing attributes is not permitted now");
      internal_coordinates_[particle][k.get_index()-4]
        = internal::FloatAttributeTableTraits::get_invalid();
      internal_coordinate_derivatives_[particle]
          [k.get_index()-4]
        = internal::FloatAttributeTableTraits::get_invalid();
    } else {
      data_.remove_attribute(FloatKey(k.get_index()-7), particle);
      derivatives_.remove_attribute(FloatKey(k.get_index()-7), particle);
    }
    if (optimizeds_.get_has_attribute(k, particle)) {
      optimizeds_.remove_attribute(k, particle);
    }
  }
  bool get_is_optimized(FloatKey k, ParticleIndex particle) const {
    return optimizeds_.get_has_attribute(k, particle);
  }
  // check NOT_EVALUATING
  void set_is_optimized(FloatKey k, ParticleIndex particle, bool tf) {
    if (tf && !optimizeds_.get_has_attribute(k, particle)) {
      optimizeds_.add_attribute(k, particle, true);
    } else if (!tf && optimizeds_.get_has_attribute(k, particle)){
      optimizeds_.remove_attribute(k, particle);
    }
  }
  // check AFTER_EVALUATE, NOT_EVALUATING
  double get_derivative(FloatKey k, ParticleIndex particle,
                        bool checked=true) const {
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Can't get derivative that isn't there");
    if (k.get_index() < 4) {
      if (checked) {
        IMP_CHECK_MASK(read_derivatives_mask_, particle,
                       "Reading the derivatives is not permitted now");
      }
      return sphere_derivatives_[particle][k.get_index()];
    } else if (k.get_index() < 7) {
      if (checked) {
        IMP_CHECK_MASK(read_derivatives_mask_, particle,
                       "Reading the derivatives is not permitted now");
      }
      return internal_coordinate_derivatives_[particle]
          [k.get_index()-4];
    } else {
      return derivatives_.get_attribute(FloatKey(k.get_index()-7), particle,
                                        checked);
    }
  }
  // check can change EVALUATE, AFTER_EVALUATE< NOT_EVALUATING
  void add_to_derivative(FloatKey k, ParticleIndex particle, double v,
                         const DerivativeAccumulator &da) {
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Can't get derivative that isn't there");
    if (k.get_index() < 4) {
      IMP_CHECK_MASK(write_derivatives_mask_, particle,
                     "Writing the derivatives is not permitted now");
      sphere_derivatives_[particle][k.get_index()]+=da(v);;
    } else if (k.get_index() < 7) {
      IMP_CHECK_MASK(write_derivatives_mask_, particle,
                     "Writing the derivatives is not permitted now");
      internal_coordinate_derivatives_[particle]
          [k.get_index()-4]+=da(v);;
    } else {
      FloatKey nk(k.get_index()-7);
      derivatives_.set_attribute(nk, particle,
                                 derivatives_.get_attribute(nk,
                                                            particle)+da(v));
    }
  }
  void add_attribute(FloatKey k, ParticleIndex particle, double v,
                     bool opt=false) {
    IMP_CHECK_MASK(add_remove_mask_, particle,
                     "Changing the attributes is not permitted now");
    IMP_USAGE_CHECK(!get_has_attribute(k, particle),
                    "Can't add attribute that is there");
    if (k.get_index() <4) {
      if (spheres_.size() <= get_as_unsigned_int(particle)) {
        spheres_.resize(get_as_unsigned_int(particle)+1, get_invalid_sphere());
        sphere_derivatives_.resize(get_as_unsigned_int(particle)+1,
                                   get_invalid_sphere());
      }
      spheres_[particle][k.get_index()]=v;
    } else if (k.get_index() <7) {
      if (internal_coordinates_.size() <= get_as_unsigned_int(particle)) {
        internal_coordinates_.resize(get_as_unsigned_int(particle)+1,
                                     get_invalid_sphere().get_center());
        internal_coordinate_derivatives_.
            resize(get_as_unsigned_int(particle)+1,
                 get_invalid_sphere().get_center());
      }
      internal_coordinates_[particle][k.get_index()-4]=v;
    } else {
      FloatKey nk(k.get_index()-7);
      data_.add_attribute(nk, particle, v);
      derivatives_.add_attribute(nk, particle, 0);
    }
    if (opt) optimizeds_.add_attribute(k, particle, true);
    ranges_.resize(std::max(ranges_.size(),
                            static_cast<size_t>(k.get_index()+1)),
                   FloatRange(-std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max()));
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Can't attribute was not added");
  }
  bool get_has_attribute(FloatKey k, ParticleIndex particle) const {
    if (k.get_index() < 4) {
      if (spheres_.size() <= get_as_unsigned_int(particle)) return false;
      else if (!internal::FloatAttributeTableTraits
               ::get_is_valid(spheres_[particle]
                              [k.get_index()])){
        return false;
      }
      return true;
    } else if (k.get_index() < 7) {
      if (internal_coordinates_.size() <= get_as_unsigned_int(particle)) {
        return false;
      }
      else if (!internal::FloatAttributeTableTraits
               ::get_is_valid(internal_coordinates_
                              [particle]
                              [k.get_index()-4])){
        return false;
      }
      return true;
    } else {
      return data_.get_has_attribute(FloatKey(k.get_index()-7), particle);
    }
  }
  void set_attribute(FloatKey k, ParticleIndex particle,
                     double v) {
    IMP_CHECK_MASK(write_mask_, particle,
                     "Changing the attribute values is not permitted now");
    IMP_USAGE_CHECK(internal::FloatAttributeTableTraits::get_is_valid(v),
                    "Can't set attribute to invalid value");
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Can't set attribute that is not there");
    if (k.get_index() <4) {
      spheres_[particle][k.get_index()]=v;
    } else if (k.get_index() <7) {
      spheres_[particle][k.get_index()-4]=v;
    } else {
      data_.set_attribute(FloatKey(k.get_index()-7), particle, v);
    }
  }
  double get_attribute(FloatKey k,
                       ParticleIndex particle,
                       bool checked=true) const {
    if (checked) {
      IMP_CHECK_MASK(read_mask_, particle,
                     "Reading the attribute values is not permitted now");
    }
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Can't get attribute that is not there");
    if (k.get_index()<4) {
      return spheres_[particle][k.get_index()];
    } else if (k.get_index()<7) {
      return spheres_[particle][k.get_index()-4];
    } else {
      return data_.get_attribute(FloatKey(k.get_index()-7), particle, checked);
    }
  }
  double& access_attribute(FloatKey k,
                       ParticleIndex particle) {
    IMP_CHECK_MASK(write_mask_, particle,
                   "Writing the attribute values is not permitted now");
    IMP_USAGE_CHECK(get_has_attribute(k, particle),
                    "Can't get attribute that is not there");
    if (k.get_index()<4) {
      return spheres_[particle][k.get_index()];
    } else if (k.get_index()<7) {
      return spheres_[particle][k.get_index()-4];
    } else {
      return data_.access_attribute(FloatKey(k.get_index()-7), particle);
    }
  }
  struct FloatIndex
  {
    ParticleIndex p_;
    FloatKey k_;
    FloatIndex(FloatKey k, ParticleIndex p): p_(p), k_(k){}
    FloatIndex(): p_(base::get_invalid_index<ParticleIndexTag>()) {}
  };
  IMP_BUILTIN_VALUES(FloatIndex, FloatIndexes);
 FloatIndexes get_optimized_attributes() const {
    FloatIndexes ret;
    for (unsigned int i=0; i< optimizeds_.size(); ++i) {
      for (unsigned int j=0; j< optimizeds_.size(i); ++j) {
        if (optimizeds_.get_has_attribute(FloatKey(i), ParticleIndex(j))) {
          ret.push_back(FloatIndex(FloatKey(i), ParticleIndex(j)));
        }
      }
    }
    return ret;
  }
  void set_range(FloatKey k, FloatRange fr) {
    ranges_[k.get_index()]=fr;
  }
  FloatRange get_range(FloatKey k) {
    FloatRange ret= ranges_[k.get_index()];
    if (ret.first==-std::numeric_limits<double>::max()) {
      if (k.get_index() <4) {
        std::swap(ret.first, ret.second);
        for (unsigned int i=0; i< spheres_.size(); ++i) {
          if (internal::FloatAttributeTableTraits
              ::get_is_valid(spheres_[ParticleIndex(i)][k.get_index()])) {
            ret.first= std::min(ret.first, spheres_[ParticleIndex(i)]
                                [k.get_index()]);
            ret.second= std::max(ret.second, spheres_[ParticleIndex(i)]
                                 [k.get_index()]);
          }
        }
        return ret;
      } else if (k.get_index() < 7) {
        std::swap(ret.first, ret.second);
        for (unsigned int i=0; i< internal_coordinates_.size(); ++i) {
          if (internal::FloatAttributeTableTraits
              ::get_is_valid(internal_coordinates_[ParticleIndex(i)]
                             [k.get_index()-4])) {
            ret.first= std::min(ret.first,
                                internal_coordinates_[ParticleIndex(i)]
                                [k.get_index()-4]);
            ret.second= std::max(ret.second,
                                 internal_coordinates_[ParticleIndex(i)]
                                 [k.get_index()-4]);
          }
        }
        return ret;

      } else {
        return data_.get_range_internal(FloatKey(k.get_index()-7));
      }
    } else {
      return ret;
    }
  }
  void clear_attributes(ParticleIndex particle) {
    if (spheres_.size()> get_as_unsigned_int(particle)) {
      spheres_[particle]= get_invalid_sphere();
      sphere_derivatives_[particle]=get_invalid_sphere();
    }
    if (internal_coordinates_.size()> get_as_unsigned_int(particle)) {
      internal_coordinates_[particle]
          = get_invalid_sphere().get_center();
      internal_coordinate_derivatives_[particle]
        =get_invalid_sphere().get_center();
    }
    data_.clear_attributes(particle);
    derivatives_.clear_attributes(particle);
    optimizeds_.clear_attributes(particle);
  }
  FloatKeys get_attribute_keys(ParticleIndex particle) const {
    FloatKeys ret=data_.get_attribute_keys(particle);
    for (unsigned int i=0; i< ret.size(); ++i) {
      ret[i]= FloatKey(ret[i].get_index()+7);
    }
    for (unsigned int i=0; i< 7; ++i) {
      if (get_has_attribute(FloatKey(i),particle)) {
        ret.push_back(FloatKey(i));
      }
    }
    return ret;
  }
};

IMP_SWAP(FloatAttributeTable);

typedef BasicAttributeTable<internal::StringAttributeTableTraits>
StringAttributeTable;
typedef BasicAttributeTable<internal::IntAttributeTableTraits>
IntAttributeTable;
typedef BasicAttributeTable<internal::ObjectAttributeTableTraits>
ObjectAttributeTable;
typedef BasicAttributeTable<internal::IntsAttributeTableTraits>
IntsAttributeTable;
typedef BasicAttributeTable<internal::ObjectsAttributeTableTraits>
ObjectsAttributeTable;
typedef BasicAttributeTable<internal::ParticleAttributeTableTraits>
ParticleAttributeTable;
typedef BasicAttributeTable<internal::ParticlesAttributeTableTraits>
ParticlesAttributeTable;


struct Masks {
#if IMP_BUILD < IMP_FAST
  mutable Mask read_mask_, write_mask_, add_remove_mask_,
    read_derivatives_mask_, write_derivatives_mask_;
#endif
};
IMP_END_INTERNAL_NAMESPACE

#ifndef SWIG
#define IMP_MODEL_IMPORT(Base)                  \
  using Base::add_attribute;                    \
  using Base::add_cache_attribute;              \
  using Base::remove_attribute;                 \
  using Base::get_has_attribute;                \
  using Base::set_attribute;                    \
  using Base::get_attribute;                    \
  using Base::access_attribute
#else
#define IMP_MODEL_IMPORT(Base)
#endif

#endif  /* IMPKERNEL_INTERNAL_ATTRIBUTE_TABLES_H */
