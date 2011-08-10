/**
 *  \file IMP/rmf/hdf5_types.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPRMF_HDF_5_TYPES_H
#define IMPRMF_HDF_5_TYPES_H

#include "rmf_config.h"
#include "NodeID.h"
#include "hdf5_handle.h"
#include "internal/utility.h"
#include <hdf5.h>
#include <algorithm>
#include <vector>
#include <exception>

#include <IMP/exception.h>
#include <IMP/RefCounted.h>
#include <limits>
#include <boost/utility.hpp>


IMPRMF_BEGIN_NAMESPACE

/** \name Traits classes

 The traits class for mapping between C++ types and HDF5 types. It defines
 - Type: the C++ type of the data to store
 - Types: the C++ type for more than one value of the data
 - static hid_t get_hdf5_type()
 - static void write_value_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                   double v)
 - static double read_value_dataset(hid_t d, hid_t is,
                                  hid_t sp)
 - static std::vector<double> read_values_attribute(hid_t a, unsigned int size)
 - static void write_values_attribute(hid_t a, const std::vector<double> &v)
 - static const double& get_null_value()
 - static std::string get_name()
 - static unsigned int get_index()

 Each type must be associated with a unique index. For the moment,
 the integers are
 - Int: 0
 - Float: 1
 - Index: 2
 - String: 3
 - NodeID: 4
 - DataSet: 5
 - Char: 6
@{
*/
/** Store floating point numbers as doubles. */
struct IMPRMFEXPORT FloatTraits {
  typedef double Type;
  typedef std::vector<double> Types;
  static hid_t get_hdf5_disk_type() {
    return H5T_IEEE_F64LE;
  }
  static hid_t get_hdf5_memory_type() {
    return H5T_NATIVE_DOUBLE;
  }
  static void write_value_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                  double v);
  static double read_value_dataset(hid_t d, hid_t is,
                                   hid_t sp);
  static void write_values_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                   const std::vector<double>& v);
  static std::vector<double> read_values_dataset(hid_t d, hid_t is,
                                                 hid_t sp,
                                                 unsigned int sz);
  static std::vector<double> read_values_attribute(hid_t a, unsigned int size);
  static void write_values_attribute(hid_t a, const std::vector<double> &v);
  static const double& get_null_value() {
    static const double ret= std::numeric_limits<double>::max();
    return ret;
  }
  static hid_t get_hdf5_fill_type() {
    return H5T_NATIVE_DOUBLE;
  }
  static const double& get_fill_value() {
    return get_null_value();
  }
  static bool get_is_null_value(const double& f) {
    return (f >= std::numeric_limits<double>::max());
  }
  static std::string get_name();
  static unsigned int get_index() {
    return 1;
  }
};
/** Store integers.*/
struct IMPRMFEXPORT IntTraits {
  typedef int Type;
  typedef std::vector<int> Types;
  static hid_t get_hdf5_disk_type() {
    return H5T_STD_I64LE;
  }
  static hid_t get_hdf5_memory_type() {
    return H5T_NATIVE_INT;
  }
  static void write_value_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                  int v);
  static int read_value_dataset(hid_t d, hid_t is,
                                hid_t sp);
  static void write_values_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                   const std::vector<int>& v);
  static std::vector<int> read_values_dataset(hid_t d, hid_t is,
                                              hid_t sp,
                                              unsigned int sz);
  static std::vector<int> read_values_attribute(hid_t a, unsigned int size);
  static void write_values_attribute(hid_t a, const std::vector<int> &v);
  static hid_t get_hdf5_fill_type() {
    return H5T_NATIVE_INT;
  }
  static const int& get_fill_value() {
    return get_null_value();
  }
  static const int& get_null_value() {
    static const int ret= std::numeric_limits<int>::max();
    return ret;
  }
  static bool get_is_null_value(int i) {
    return i== std::numeric_limits<int>::max();
  }
  static std::string get_name();
  static unsigned int get_index() {
    return 0;
  }
};


/** A non-negative index.*/
struct IMPRMFEXPORT IndexTraits: public IntTraits {
  static const int& get_null_value()  {
    static const int ret=-1;
    return ret;
  }
  static const int& get_fill_value() {
    return get_null_value();
  }
  static bool get_is_null_value(int i) {
    return i==-1;
  }
  static std::string get_name();
  static unsigned int get_index() {
    return 2;
  }
};
/** A string */
struct IMPRMFEXPORT StringTraits {
 private:
  static hid_t create_string_type() {
    hid_t tid1 = H5Tcopy (H5T_C_S1);
    IMP_HDF5_CALL(H5Tset_size (tid1,H5T_VARIABLE));
    return tid1;
  }
 public:
  typedef std::string Type;
  typedef std::vector<std::string> Types;
  static hid_t get_hdf5_disk_type() {
    static HDF5Handle ret(create_string_type(),
                          H5Tclose);
    IMP_RMF_INTERNAL_CHECK(H5Tequal(ret, ret),
                           "The type does not equal itself");
    IMP_RMF_INTERNAL_CHECK(H5Tequal(ret,
                                    HDF5Handle(create_string_type(),
                                               H5Tclose)),
                           "The type does not equal a new copy");
    return ret;
  }
  static hid_t get_hdf5_memory_type() {
    return get_hdf5_disk_type();
  }
  static void write_value_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                  std::string v);
  static std::string read_value_dataset(hid_t d, hid_t is,
                                        hid_t sp);
  static void write_values_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                  const std::vector<std::string>& v);
  static std::vector<std::string> read_values_dataset(hid_t d, hid_t is,
                                                      hid_t sp,
                                                      unsigned int sz);
  static std::vector<std::string>
    read_values_attribute(hid_t a, unsigned int size);
  static void write_values_attribute(hid_t a,
                                     const std::vector<std::string> &v);
  static std::string  get_null_value() {
    return std::string();
  }
  static hid_t get_hdf5_fill_type() {
    return get_hdf5_memory_type();
  }
  static char*&  get_fill_value() {
    static char* val=NULL;
    return val;
  }
  static bool get_is_null_value(std::string s) {
    return s.empty();
  }
  static std::string get_name();
  static unsigned int get_index() {
    return 3;
  }
};

/** Store the Node for other nodes in the hierarchy. */
struct IMPRMFEXPORT NodeIDTraits {
  typedef NodeID Type;
  typedef std::vector<NodeID> Types;
  static hid_t get_hdf5_disk_type() {
    return IndexTraits::get_hdf5_disk_type();
  }
  static hid_t get_hdf5_memory_type() {
    return IndexTraits::get_hdf5_memory_type();
  }
  static void write_value_dataset(hid_t d, hid_t is,
                                  hid_t s,
                                  NodeID v);
  static NodeID read_value_dataset(hid_t d, hid_t is,
                                   hid_t sp);
  static void write_values_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                   const std::vector<NodeID>& v);
  static std::vector<NodeID> read_values_dataset(hid_t d, hid_t is,
                                                 hid_t sp,
                                                 unsigned int sz);
  static std::vector<NodeID>
    read_values_attribute(hid_t a, unsigned int size);
  static void write_values_attribute(hid_t a,
                                     const std::vector<NodeID> &v);
  static const NodeID& get_null_value() {
    static NodeID n;
    return n;
  }
  static hid_t get_hdf5_fill_type() {
    return IntTraits::get_hdf5_fill_type();
  }
  static const int& get_fill_value() {
    return IntTraits::get_fill_value();
  }
  static bool get_is_null_value(NodeID s) {
    return s== NodeID();
  }
  static std::string get_name();
  static unsigned int get_index() {
    return 4;
  }
};

/** An HDF5 data set. Currently, the type of data stored in the data
    set must be known implicitly. The path to the HDF5 data
    set relative to the node containing the data is passed.*/
struct IMPRMFEXPORT DataSetTraits: public StringTraits {
  static std::string get_name();
  static unsigned int get_index() {
    return 5;
  }
};

/** Store a single string. Currently this is not supported
    for data sets.*/
struct IMPRMFEXPORT CharTraits {
  typedef char Type;
  typedef std::string Types;
  static hid_t get_hdf5_disk_type() {
    return H5T_STD_I8LE;
  }
  /*static void write_value_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                  int v);
  static int read_value_dataset(hid_t d, hid_t is,
                                hid_t sp);
  static void write_values_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                   const std::vector<int>& v);
  static std::string read_values_dataset(hid_t d, hid_t is,
                                         hid_t sp,
                                         unsigned int sz);*/
  static std::string read_values_attribute(hid_t a, unsigned int size);
  static void write_values_attribute(hid_t a, std::string v);
  static hid_t get_hdf5_fill_type() {
    return H5T_NATIVE_CHAR;
  }
  static char get_fill_value() {
    return get_null_value();
  }
  static char get_null_value() {
    return '\0';
  }
  static bool get_is_null_value(char i) {
    return i== '\0';
  }
  static std::string get_name();
  static unsigned int get_index() {
    return 6;
  }
};

/** Store an array of integers.*/
template <class BaseTraits>
struct ArrayTraits {
 private:
 static hid_t get_hdf5_memory_type() {
    static HDF5Handle
      ints_type( H5Tvlen_create (BaseTraits::get_hdf5_memory_type()),
                 H5Tclose);
    return ints_type;
  }
 public:
  typedef std::vector<typename BaseTraits::Type> Type;
  typedef std::vector< Type > Types;
  static hid_t get_hdf5_disk_type() {
    static HDF5Handle
      ints_type( H5Tvlen_create (BaseTraits::get_hdf5_disk_type()),
                 H5Tclose);
    return ints_type;
  }
  static void write_value_dataset(hid_t d, hid_t is,
                                  hid_t s,
                                  const Type& v) {
    hvl_t data;
    data.len=v.size();
    data.p= const_cast<typename BaseTraits::Type*>(&v[0]);
    IMP_HDF5_CALL(H5Dwrite(d,
                           get_hdf5_memory_type(), is, s,
                           H5P_DEFAULT, &data));
  }
  static Type read_value_dataset(hid_t d, hid_t is,
                                 hid_t sp) {
    hvl_t data;
    H5Dread (d, get_hdf5_memory_type(), is, sp, H5P_DEFAULT, &data);
    Type ret(data.len);
    std::copy(static_cast<typename BaseTraits::Type*>(data.p),
              static_cast<typename BaseTraits::Type*>(data.p)+data.len,
              ret.begin());
    //HDF5Handle space(H5Dget_space (d), H5Sclose);
    //IMP_HDF5_CALL(H5Dvlen_reclaim(get_hdf5_type(), space,
    // H5P_DEFAULT, &data));
    free(data.p);
    return ret;
  }
  static void write_values_dataset(hid_t d, hid_t is,
                                   hid_t s,
                                   const Types& v) {
    IMP_NOT_IMPLEMENTED;
  }
  static Types read_values_dataset(hid_t d, hid_t is,
                                   hid_t sp,
                                   unsigned int sz) {
    IMP_NOT_IMPLEMENTED;
  }
  static Types read_values_attribute(hid_t a, unsigned int size) {
    IMP_NOT_IMPLEMENTED;
  }
  static void write_values_attribute(hid_t a, const Types &v) {
    IMP_NOT_IMPLEMENTED;
  }
  static hid_t get_hdf5_fill_type() {
    return get_hdf5_memory_type();
  }
  static hvl_t& get_fill_value() {
    static hvl_t val={0,0};
    return val;
  }
  static Type get_null_value() {
    return Type();
  }
  static bool get_is_null_value(const Type &i) {
    return i.empty();
  }
  static std::string get_name() {
    return BaseTraits::get_name()+"s";
  }
  static unsigned int get_index() {
    return 7+ BaseTraits::get_index();
  }
};

typedef ArrayTraits<IntTraits> IntsTraits;
typedef ArrayTraits<NodeIDTraits> NodeIDsTraits;

/** @} */



IMPRMF_END_NAMESPACE

#endif /* IMPRMF_HDF_5_TYPES_H */
