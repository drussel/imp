/**
 *  \file ArrayOnAttributesHelper.h
 *  \brief Various methods for managing an array of attributes
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_ARRAY_ON_ATTRIBUTES_HELPER_H
#define IMPCORE_ARRAY_ON_ATTRIBUTES_HELPER_H

#include "../core_config.h"
#include "../core_macros.h"
#include "IMP/internal/IndexingIterator.h"

#include <IMP/base_types.h>
#include <IMP/VectorOfRefCounted.h>
#include <IMP/Particle.h>

#include <string>
#include <sstream>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

struct ArrayDataBase: public RefCounted {
  virtual ~ArrayDataBase(){}
};


struct Empty{};

template <class Key, class Value, class Data>
struct ArrayData: public ArrayDataBase, public Data {
  typedef ArrayData This;
  ArrayData(std::string p):
    num_key((p+"_number").c_str()),
    prefix(p) {
  }
  ~ArrayData(){}
  std::vector<Key> keys;
  const IntKey num_key;
  const std::string prefix;
  IMP_COMPARISONS_1(prefix);
};

template <class K, class V, class D>
std::ostream &operator<<(std::ostream &out, const ArrayData<K,V,D> &d) {
  out << d.prefix;
  return out;
}

// to avoid having memory leaks reported
IMPCOREEXPORT extern VectorOfRefCounted<ArrayDataBase*> array_datas;


template <class KeyT, class ValueT, class DataT=Empty>
struct ArrayOnAttributesHelper {
  typedef KeyT Key;
  typedef ValueT Value;
  typedef ArrayData<Key, Value, DataT> Data;

  DataT& get_data() {return static_cast<DataT&>(*data_);}
  const DataT& get_data() const {return static_cast<DataT&>(*data_);}

  ArrayOnAttributesHelper(){}
  ArrayOnAttributesHelper(std::string p): data_(new Data(p)){
    array_datas.push_back(data_);
  }

  template <class T>
  void audit_value(T) const {}

  template <class T>
  Value wrap(const T &t) const {
    return Value(t);
  }

  unsigned int get_size(const Particle *p) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    return p->get_value(data_->num_key);
  }

  void initialize_particle(Particle *p) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    p->add_attribute(data_->num_key, 0);
  }

  Value get_value(const Particle *p, unsigned int i) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized ArryHelper traits");
    IMP_USAGE_CHECK(static_cast<unsigned int>(p->get_value(data_->num_key)) > i,
              "Out of range attribute in array");
    return p->get_value(get_key(i));
  }

  Value get_value(const Value &v) const {
    return v;
  }

  void set_value(Particle *p,
                 unsigned int i,
                 Value v) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized ArrayHelper traits");
    IMP_USAGE_CHECK(data_->keys.size() > i, "Out of range attribute in array");
    IMP_USAGE_CHECK(p->get_value(data_->num_key) > i,
              "Out of range attribute in array");
    p->set_value(data_->keys[i], v);
  }

  unsigned int push_back(Particle *p,
                         Value v) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized ArrayHelper traits");
    unsigned int osz= p->get_value(data_->num_key);
    Key k= get_key(osz);
    p->add_attribute(k, v);
    p->set_value(data_->num_key, osz+1);
    return osz;
  }

  void insert(Particle *p,
              unsigned int loc,
              Value v) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    unsigned int osz= p->get_value(data_->num_key);
    IMP_USAGE_CHECK(loc <= osz, "Attribute array must be contiguous");
    for (unsigned int i=loc; i < osz; ++i) {
      Key k= get_key(i);
      Value t= p->get_value(k);
      p->set_value(k, v);
      v=t;
    }
    Key k= get_key(osz);
    p->add_attribute(k, v);
    p->set_value(data_->num_key, osz+1);
  }


  void erase(Particle *p,
             unsigned int loc) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    unsigned int osz= p->get_value(data_->num_key);
    IMP_USAGE_CHECK(loc <= osz, "Can only erase values in array");
    for (unsigned int i=loc+1; i < osz; ++i) {
      Key k= data_->keys[i];
      Key kl= data_->keys[i-1];
      p->set_value(kl, p->get_value(k));
    }
    Key k= data_->keys[osz-1];
    p->remove_attribute(k);
    p->set_value(data_->num_key, osz-1);
  }

  void clear(Particle *p) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    unsigned int osz= p->get_value(data_->num_key);
    for (unsigned int i=0; i < osz; ++i) {
      Key kl= data_->keys[i];
      p->remove_attribute(kl);
    }
    p->set_value(data_->num_key, 0);
  }

  std::string get_prefix() const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    return data_->prefix;
  }


  bool has_required_attributes(Particle *p) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    return p->has_attribute(data_->num_key);
  }

  void add_required_attributes(Particle *p) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    p->add_attribute(data_->num_key, 0);
  }

  void on_change(Particle *, const Value &,
                 unsigned int, unsigned int) const {}
  void on_remove(Particle *, const Value &) const {}
  void on_add(Particle *, const Value &, unsigned int) const {}

  unsigned int get_index(Particle *p, const Value &v) const {
    for (unsigned int i=0; i< get_size(p); ++i) {
      if (get_value(p, i) ==v) return i;
    }
    IMP_THROW("Value not found: "  << v, ValueException);
  }

private:
  Key get_key(unsigned int i) const {
    IMP_INTERNAL_CHECK(data_, "Cannot used uninitialized HierarchyTraits");
    while (!(i < data_->keys.size())) {
      std::ostringstream oss;
      oss << data_->prefix << data_->keys.size();
      data_->keys.push_back(Key(oss.str().c_str()));
    }
    return data_->keys[i];
  }

  mutable Data* data_;
};

IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_ARRAY_ON_ATTRIBUTES_HELPER_H */
