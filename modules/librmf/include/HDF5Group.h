/**
 *  \file RMF/HDF5Group.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPLIBRMF_HDF_5GROUP_H
#define IMPLIBRMF_HDF_5GROUP_H

#include "RMF_config.h"
#include "HDF5ConstGroup.h"
#include "HDF5MutableAttributes.h"


namespace RMF {

  typedef HDF5MutableAttributes<HDF5ConstGroup> HDF5GroupAttributes;
#ifndef IMP_DOXYGEN
  typedef vector<HDF5GroupAttributes> HDF5GroupAttributesList;
#endif


  /** Wrap an HDF5 Group. See
      \external{http://www.hdfgroup.org/HDF5/doc/UG/UG_frame09Groups.html,
      the HDF5 manual} for more information.
  */
  class RMFEXPORT HDF5Group: public HDF5MutableAttributes<HDF5ConstGroup> {
    typedef HDF5MutableAttributes<HDF5ConstGroup> P;
    friend class HDF5File;
    unsigned int get_number_of_links() const {
      H5G_info_t info;
      IMP_HDF5_CALL(H5Gget_info(get_handle(), &info));
      unsigned int n= info.nlinks;
      return n;
    }
#ifndef SWIG
  protected:
    HDF5Group(HDF5SharedHandle *h);
#endif
  public:
    HDF5Group(){}
#if !defined(IMP_DOXYGEN) && !defined(SWIG)
    static HDF5Group get_from_const_group(HDF5ConstGroup g) {
      return HDF5Group(g.get_shared_handle());
    }
#endif

    IMP_RMF_SHOWABLE(HDF5Group, "HDF5Group " << get_name());

    // create from an existing group
    HDF5Group(HDF5Group parent, std::string name);
    HDF5Group add_child_group(std::string name);
    template <class TypeTraits, unsigned int D>
      HDF5DataSetD<TypeTraits, D>
      add_child_data_set(std::string name,
                         Compression comp= NO_COMPRESSION,
                         HDF5DataSetIndexD<D> chunksize=HDF5DataSetIndexD<D>())
    {
      return HDF5DataSetD<TypeTraits, D>(get_shared_handle(), name,
                                         comp, chunksize);
    }
    template <class TypeTraits, unsigned int D>
      HDF5DataSetD<TypeTraits, D> get_child_data_set(std::string name) const {
      return HDF5DataSetD<TypeTraits, D>(get_shared_handle(), name, true);
    }
#define IMP_HDF5_DATA_SET_METHODS_D(lcname, UCName, PassValue, ReturnValue, \
                                    PassValues, ReturnValues, D)        \
    HDF5DataSetD<UCName##Traits, D>                                     \
      get_child_##lcname##_data_set_##D##d(std::string name)            \
      const {                                                           \
      return get_child_data_set<UCName##Traits, D>(name);               \
    }                                                                   \
    HDF5DataSetD<UCName##Traits, D>                                     \
      add_child_##lcname##_data_set_##D##d(std::string name){           \
      return add_child_data_set<UCName##Traits, D>(name);               \
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

    HDF5Group get_child_group(unsigned int i) const;
  };

} /* namespace RMF */

#endif /* IMPLIBRMF_HDF_5GROUP_H */
