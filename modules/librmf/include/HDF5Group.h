/**
 *  \file RMF/HDF5Group.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPLIBRMF_HDF_5GROUP_H
#define IMPLIBRMF_HDF_5GROUP_H

#include "RMF_config.h"
#include "HDF5DataSetD.h"


namespace RMF {

class HDF5File;

  /** Wrap an HDF5 Group. See
      \external{http://www.hdfgroup.org/HDF5/doc/UG/UG_frame09Groups.html,
      the HDF5 manual} for more information.
  */
  class RMFEXPORT HDF5Group {
    boost::intrusive_ptr<HDF5SharedHandle> h_;
    unsigned int get_number_of_links() const {
      H5G_info_t info;
      IMP_HDF5_CALL(H5Gget_info(h_->get_hid(), &info));
      unsigned int n= info.nlinks;
      return n;
    }
#ifndef SWIG
  protected:
    HDF5Group(HDF5SharedHandle *h);
#endif
  public:
    std::string get_name() const {
      char buf[10000];
      IMP_HDF5_CALL(H5Iget_name(h_->get_hid(), buf, 10000));
      return std::string(buf);
    }

    //! Get an object for the file containing the group
    HDF5File get_file() const;

    IMP_RMF_SHOWABLE(HDF5Group, "HDF5Group " << get_name());

    // create from an existing group
    HDF5Group(HDF5Group parent, std::string name);
    HDF5Group add_child(std::string name);
    template <class TypeTraits, unsigned int D>
      HDF5DataSetD<TypeTraits, D> add_child_data_set(std::string name) {
      return HDF5DataSetD<TypeTraits, D>(h_.get(), name);
    }
    template <class TypeTraits, unsigned int D>
      HDF5DataSetD<TypeTraits, D> get_child_data_set(std::string name) const {
      return HDF5DataSetD<TypeTraits, D>(h_.get(), name, true);
    }
#define IMP_HDF5_DATA_SET_METHODS_D(lcname, UCName, PassValue, ReturnValue, \
                                    PassValues, ReturnValues, D)        \
    HDF5DataSetD<UCName##Traits, D>                                     \
      add_child_##lcname##_data_set_##D##d(std::string name){           \
      return add_child_data_set<UCName##Traits, D>(name);               \
    }                                                                   \
    HDF5DataSetD<UCName##Traits, D>                                     \
      get_child_##lcname##_data_set_##D##d(std::string name)            \
      const {                                                           \
      return get_child_data_set<UCName##Traits, D>(name);               \
    }

#define IMP_HDF5_DATA_SET_METHODS(lcname, UCName, PassValue, ReturnValue, \
                                  PassValues, ReturnValues)             \
    IMP_HDF5_DATA_SET_METHODS_D(lcname, UCName, PassValue, ReturnValue, \
                                PassValues, ReturnValues, 1);           \
    IMP_HDF5_DATA_SET_METHODS_D(lcname, UCName, PassValue, ReturnValue, \
                                PassValues, ReturnValues, 2);           \
    IMP_HDF5_DATA_SET_METHODS_D(lcname, UCName, PassValue, ReturnValue, \
                                PassValues, ReturnValues, 3)

    /** \name Untemplated methods
        When using Python, you must call the non-templated methods listed
        below.
        @{
    */
    IMP_RMF_FOREACH_TYPE(IMP_HDF5_DATA_SET_METHODS);
    /** @} */

    unsigned int get_number_of_children() const;
    std::string get_child_name(unsigned int i) const;
    bool get_has_child(std::string name) const;
    hid_t get_handle() const;
    bool get_child_is_group(unsigned int i) const;
    bool get_child_is_data_set(unsigned int i) const;

    /** \name Template attribute methods
        When manipulating attriutes from C++ you can use these
        templated methods.
        @{
    */
    template <class TypeTraits>
      void set_attribute(std::string name,
                         typename TypeTraits::Types value) {
      if (value.empty()) {
        if (H5Aexists(h_->get_hid(), name.c_str())) {
          IMP_HDF5_CALL(H5Adelete(h_->get_hid(), name.c_str()));
        }
      } else {
        bool missing=!H5Aexists(h_->get_hid(), name.c_str());
        if (!missing) {
          hsize_t dim, maxdim;
          {
            IMP_HDF5_HANDLE(a,H5Aopen(h_->get_hid(), name.c_str(), H5P_DEFAULT),
                         &H5Aclose);
            IMP_HDF5_HANDLE(s,H5Aget_space(a), &H5Sclose);
            IMP_HDF5_CALL(H5Sget_simple_extent_dims(s, &dim, &maxdim));
          }
          if (value.size() != dim) {
            IMP_HDF5_CALL(H5Adelete(h_->get_hid(), name.c_str()));
            missing=true;
          }
        }
        if (missing) {
          IMP_HDF5_HANDLE(s, H5Screate(H5S_SIMPLE), &H5Sclose);
          hsize_t dim=std::max(value.size(), size_t(1));
          hsize_t max=H5S_UNLIMITED;
          IMP_HDF5_CALL(H5Sset_extent_simple(s, 1, &dim, &max));
          IMP_HDF5_HANDLE(a, H5Acreate2(h_->get_hid(), name.c_str(),
                                  TypeTraits::get_hdf5_disk_type(),
                                  s, H5P_DEFAULT, H5P_DEFAULT),
                       &H5Aclose);
        }
        IMP_HDF5_HANDLE( a, H5Aopen(h_->get_hid(), name.c_str(), H5P_DEFAULT),
                         &H5Aclose);
        TypeTraits::write_values_attribute(a, value);
      }
    }
    template <class TypeTraits>
      typename TypeTraits::Types
      get_attribute(std::string name) const {
      if (!H5Aexists(h_->get_hid(), name.c_str())) {
        return typename TypeTraits::Types();
      } else {
        IMP_HDF5_HANDLE(a, H5Aopen(h_->get_hid(), name.c_str(), H5P_DEFAULT),
                     &H5Aclose);
        IMP_HDF5_HANDLE(s, H5Aget_space(a), &H5Sclose);
        hsize_t dim, maxdim;
        IMP_HDF5_CALL(H5Sget_simple_extent_dims(s, &dim, &maxdim));
        typename TypeTraits::Types ret
          = TypeTraits::read_values_attribute(a, dim);
        return ret;
      }
    }
    /** @} */
    template <class CT, class CF>
      CT copy_to(const CF &cf) const {
      return CT(cf.begin(), cf.end());
    }
    bool get_has_attribute(std::string nm) const;
    /** \name Nontemplated attributes
        When using python, call the non-template versions of the
        attribute manipulation methods.
        @{
    */
#define IMP_HDF5_ATTRIBUTE(lcname, UCName, PassValue, ReturnValue,      \
                           PassValues, ReturnValues)                    \
    void set_##lcname##_attribute(std::string nm,                       \
                                  PassValues value) {                   \
      set_attribute< UCName##Traits>(nm, value);                        \
    }                                                                   \
    ReturnValues                                                        \
      get_##lcname##_attribute(std::string nm) const {                  \
      return get_attribute< UCName##Traits>(nm);                        \
    }                                                                   \

    IMP_RMF_FOREACH_SIMPLE_TYPE(IMP_HDF5_ATTRIBUTE);
    IMP_HDF5_ATTRIBUTE(char, Char, char, char, std::string, std::string);
    /** @} */
  };

} /* namespace RMF */

#endif /* IMPLIBRMF_HDF_5GROUP_H */
