/**
 *  \file DataObject.h
 *  \brief A modifier which variables within a ball.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_DATA_OBJECT_H
#define IMPCORE_DATA_OBJECT_H

#include "core_config.h"
#include <IMP/Object.h>

IMPCORE_BEGIN_NAMESPACE

/** This is a helper class for creating an IMP object storing some data. This
    can be used to, eg, add the data to a particle as part of a cache.

    This class is C++ only.
 */
template <class Data>
class DataObject: public IMP::base::Object
{
  Data data_;
public:
  DataObject(const Data &d, std::string name="DataObject %1%"):Object(name),
    data_(d){
  }
  const Data &get_data() const {return data_;}
  Data &access_data() {return data_;}
  IMP_OBJECT(DataObject);
};
#if !defined(IMP_DOXYGEN) && !defined(SWIG)
template <class Data>
inline void DataObject<Data>::do_show(std::ostream &out) const {
  IMP_UNUSED(out);
}
#endif

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_DATA_OBJECT_H */
