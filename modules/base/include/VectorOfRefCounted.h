/**
 *  \file VectorOfRefCounted.h
 *  \brief A common base class for ref counted objects.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_VECTOR_OF_REF_COUNTED_H
#define IMPBASE_VECTOR_OF_REF_COUNTED_H
#include "base_config.h"
#include "RefCounted.h"
#include "Object.h"
#include "internal/ref_counting.h"
#include <IMP/compatibility/vector.h>
#include <vector>

IMPBASE_BEGIN_NAMESPACE

#if !defined(SWIG) && !defined(IMP_DOXYGEN)
struct RefCountPolicy {
  template <class O>
  static void ref(O*o) {
    IMP::base::internal::ref(o);
  }
  template <class O>
  static void unref(O*o) {
    IMP::base::internal::unref(o);
  }
};
struct NoRefCountPolicy {
  template <class O>
  static void ref(O) {
  }
  template <class O>
  static void unref(O) {
  }
};
struct ControllableRefCountPolicy {
  template <class O>
  static void ref(O o) {
    if (o.get_is_ref_counted()) {
      IMP::base::internal::ref(static_cast<base::Object*>(o));
    }
  }
  template <class O>
  static void unref(O o) {
    if (o.get_is_ref_counted()) {
      IMP::base::internal::unref(static_cast<base::Object*>(o));
    }
  }
};
#endif

//! A vector-like container for reference counted objects
/** The interface of this class is like that of vector.

    Documentation for vector can be found as part of the SGI
    stl documentation, among other places
    (http://www.sgi.com/tech/stl/Vector.html).

    The first template argument should, in general, be a pointer to the
    type of object. The second tells the container how to do ref counting
    and should probably be left with its default value.

    When used within Python, IMP::VectorOfRefCounted acts like a Python list.
 */
template <class RC, class Policy= RefCountPolicy>
class VectorOfRefCounted {
  typedef void (*RF)(RC);
  typedef vector<RC> Data;
  Data data_;
  // make sure they are converted to RC
  static void ref(RC v) {
    Policy::ref(v);
  }
  static void unref(RC v) {
    Policy::unref(v);
  }
  template <class It>
  static void ref(It b, It e) {
    std::for_each(b,e, (RF) &Policy::ref);
  }
  template <class It>
  static void unref(It b, It e) {
    std::for_each(b, e, (RF) &Policy::unref);
  }
 public:
  typedef RC const_reference;
  typedef RC value_type;
  VectorOfRefCounted(const vector<RC> &o):data_(o) {
    ref(o.begin(), o.end());
  }
  template <class It>
  VectorOfRefCounted(It b, It e): data_(b,e){
    ref(b,e);
  }
  VectorOfRefCounted(const VectorOfRefCounted<RC, Policy> &o):
    data_(o.begin(), o.end()){
    ref(o.begin(), o.end());
  }
  VectorOfRefCounted(RC rc): data_(1, rc) {
    ref(rc);
  }
  VectorOfRefCounted(unsigned int n, RC rc): data_(n, rc) {
    for (unsigned int i=0; i< n; ++i) ref(rc);
  }
  VectorOfRefCounted(unsigned int i): data_(i, RC()){}
  VectorOfRefCounted(){}
  ~VectorOfRefCounted() {
    clear();
  }
#ifndef SWIG
  const VectorOfRefCounted<RC, Policy>
  operator=(const VectorOfRefCounted<RC, Policy> &o) {
    unref(data_.begin(), data_.end());
    data_= o.data_;
    ref(data_.begin(), data_.end());
    return *this;
  }

  operator const vector<RC>&() const {
    return data_;
  }
#ifndef IMP_DOXYGEN
  typedef typename vector<RC>::size_type size_type;
  // need to inherit from T to get methods right
  template <class T>
  struct Proxy: public T {
    T &v_;
    Proxy(T& v): T(v), v_(v){}
    // return void since chaining won't work right
    template <class O>
    void operator=(O ov) {
      T &v=ov;
      using std::swap;
      swap(v_, v);
      ref(v_);
      unref(v);
    }
  };
  template <class T>
  struct Proxy<T*> {
    T* &v_;
    Proxy(T*& v): v_(v){}
    operator T*() {return v_;}
    void operator=(T* v) {
      using std::swap;
      swap(v_, v);
      ref(v_);
      unref(v);
    }
    T* operator->() {
      return v_;
    }
  };
  typedef Proxy<RC> reference;
  // swig will use __set__ so we don't have to worry about it
  Proxy<RC> operator[](unsigned int i) {
    IMP_USAGE_CHECK(i < size(), "Index out of range in []: "
                    << i << ">=" << size());
    return Proxy<RC>(data_[i]);
  }
#else
  typedef RC& reference;
  // pretend it is just a normal reference
  /** Change a value in the vector (and refcount appropriately). */
  RC& operator[](unsigned int i);
#endif

#endif
  RC operator[](unsigned int i) const {
    IMP_USAGE_CHECK(i < size(), "Index out of range in []: "
              << i << ">=" << size());
    return data_[i];
  }
  RC get(unsigned int i) const {
    return operator[](i);
  }
  void set(unsigned int i, RC p) {
    IMP_USAGE_CHECK(i < size(), "Index out of range in set "
              << i << ">=" << size());
    using std::swap;
    swap(data_[i], p);
    ref(data_[i]);
    unref(p);
  }
  RC back() const {
    IMP_USAGE_CHECK(!empty(), "Can't call back on empty container");
    return data_.back();
  }
  RC front() const {
    IMP_USAGE_CHECK(!empty(), "Can't call front on empty container");
    return data_.front();
  }
  void reserve(unsigned int i){ data_.reserve(i);}
  size_type size() const {return data_.size();}
  void resize(unsigned int i) {data_.resize(i);}
  //! for compatibility, the value is ignored
  void resize(unsigned int i, const_reference) {data_.resize(i);}
  // god swig is dumb
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
  typedef typename Data::iterator iterator;
  typedef typename Data::const_iterator const_iterator;
#else
  class iterator;
  class const_iterator;
#endif
#ifndef SWIG
  const_iterator begin() const {return data_.begin();}
  const_iterator end() const {return data_.end();}
#endif
  iterator begin() {return data_.begin();}
  iterator end() {return data_.end();}
  template <class It>
  void insert(iterator loc, It b, It e) {
    data_.insert(data_.begin()+(loc-data_.begin()), b, e);
    ref(b,e);
  }
  void insert(iterator loc, RC p) {
    data_.insert(loc, p);
    ref(p);
  }
  void push_back(RC p) {
    data_.push_back(p);
    ref(p);
  }
  void pop_back() {
    unref(data_.back());
    data_.pop_back();
  }
  bool empty() const {return data_.empty();}
#ifndef IMP_DOXYGEN
  void swap_with(VectorOfRefCounted<RC, Policy> &a) {
    std::swap(a.data_, data_);
  }
  void swap_with(vector<RC> &a) {
    std::swap(a, data_);
    ref(data_.begin(), data_.end());
    unref(a.begin(), a.end());
  }
#endif
  void clear(){
    unref(data_.begin(), data_.end());
    data_.clear();
  }
  void show(std::ostream &out=std::cout) const {
    out << data_;
  }
  void erase(iterator it) {
    unref(*it);
    data_.erase(it);
  }
  void erase(iterator b, iterator e) {
    unref(b, e);
    data_.erase(b, e);
  }
  template <class F>
  void remove_if(const F &f) {
    vector<RC> bye;
    for (iterator it= begin(); it != end(); ++it) {
      if (f(*it)) bye.push_back(*it);
    }
    if (!bye.empty()) {
      data_.resize(std::remove_if(begin(), end(), f)-begin());
      unref(bye.begin(), bye.end());
    }
  }
  //! f should take two Particle*
  template <class Less>
  void sort(const Less &f) {
    std::sort(data_.begin(), data_.end(), f);
  }
  void remove(RC r) {
    for (unsigned int i=0; i< data_.size(); ++i) {
      if (data_[i]==r) {
        unref(r);
        data_.erase(data_.begin()+i);
      }
    }
  }
};

#if !defined(SWIG) && !defined(IMP_DOXYGEN)
template <class RC, class Policy>
inline std::ostream &operator<<(std::ostream &out,
                         const VectorOfRefCounted<RC, Policy> &v) {
  v.show(out);
  return out;
}

template <class RC, class Policy>
inline void swap(VectorOfRefCounted<RC, Policy> &a,
          VectorOfRefCounted<RC, Policy> &b) {
  a.swap_with(b);
}

template <class RC, class Policy>
inline void swap(VectorOfRefCounted<RC, Policy> &a,
          vector<RC> &b) {
  a.swap_with(b);
}
template <class RC, class Policy>
inline void swap(vector<RC> &b,
          VectorOfRefCounted<RC, Policy> &a) {
  a.swap_with(b);
}
#endif

#if !defined(SWIG) && !defined(IMP_DOXYGEN)
namespace internal {

  template <class T, class P, class F>
  inline void remove_if( VectorOfRefCounted<T,P> &t, const F &f) {
    t.remove_if(f);
  }

  template <class T, class F>
  inline void remove_if(vector<T> &t, const F &f) {
    t.erase(std::remove_if(t.begin(), t.end(), f), t.end());
  }
}
#endif

IMPBASE_END_NAMESPACE

#endif  /* IMPBASE_VECTOR_OF_REF_COUNTED_H */