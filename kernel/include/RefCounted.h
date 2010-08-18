/**
 *  \file RefCounted.h
 *  \brief A common base class for ref counted objects.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_REF_COUNTED_H
#define IMP_REF_COUNTED_H

#include "kernel_config.h"
#include "exception.h"
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

#include <vector>

#ifndef IMP_DOXYGEN
#ifndef SWIG

namespace IMP {
  class Object;
  namespace internal {
    template <class R, class E>
    class RefStuff;
  }
}
//IMP_END_INTERNAL_NAMESPACE
#endif
#endif

IMP_BEGIN_NAMESPACE

//! Common base class for ref counted objects.
/** This base class implements reference counting when used in
    conjunction with IMP::Pointer or IMP::WeakPointer objects.
    Objects which inherit from IMP::RefCounted should be passed
    using pointers and stored using IMP::Pointer and
    IMP::WeakPointer objects. Users must be careful to avoid
    cycles of reference counted pointers, otherwise memory will
    never be reclaimed.

    \par Introduction to reference counting:
    Reference counting is a technique for managing memory and
    automatically freeing memory (destroying objects) when it
    is no longer needed. In reference counting, each object has a reference
    count, which tracks how many different places are using the
    object. When this count goes to 0, the object is freed.\n\n
    Python internally refence counts everything. C++, on the other hand,
    requires extra steps be taken to ensure that objects
    are reference counted properly.\n\n
    In \imp, reference counting is done through the IMP::Pointer
    and IMP::RefCounted classes. The former should be used instead of
    a raw C++ pointer when storing a pointer to any object
    inheriting from IMP::RefCounted.\n\n
    Any time one is using reference counting, one needs to be aware
    of cycles, since if, for example, object A contains an IMP::Pointer to
    object B and object B contains an IMP::Pointer to object A,
    their reference counts will never go to 0 even if both A
    and B are no longer used. To avoid this, use an
    IMP::WeakPointer in one of A or B.

    IMP::RefCounted provides no public methods or constructors.
    It makes objects that inherit from it non-copyable.

    \see IMP_REF_COUNTED_DESTRUCTOR()
 */
class IMPEXPORT RefCounted
{
#ifndef IMP_DOXYGEN
  typedef RefCounted This;
  static unsigned int live_objects_;
  RefCounted(const RefCounted &){}
  RefCounted& operator=(const RefCounted &){return *this;}

#ifndef _MSC_VER
  template <class R, class E>
    friend class internal::RefStuff;
#else
 public:
#endif // _MSC_VER
  mutable int count_;
protected:
  RefCounted() {
     ++live_objects_;
     count_=0;
  }
  ~RefCounted();

 public:
  unsigned int get_ref_count() const {
    return count_;
  }

  static unsigned int get_number_of_live_objects() {
    // for debugging purposes only
    return live_objects_;
  }
#endif // IMP_DOXYGEN

};

IMP_END_NAMESPACE

#endif  /* IMP_REF_COUNTED_H */
