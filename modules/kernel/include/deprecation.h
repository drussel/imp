/**
 *  \file deprecation.h
 *  \brief Control display of deprecation information.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_DEPRECATION_H
#define IMPKERNEL_DEPRECATION_H

#include "kernel_config.h"
#include "internal/deprecation.h"

IMP_BEGIN_NAMESPACE

//! Toggle printing of warnings on using deprecated classes
/** If set to true (the default) a warning is printed every
    time a class marked as deprecated is used.
    \sa IMP_DEPRECATED
 */
IMPEXPORT void set_print_deprecation_messages(bool tf);

IMP_END_NAMESPACE

/** \brief Mark the class as deprecated. It will print out a message.

  From time to time, \imp is updated in ways that break backward
  compatibility.  In certain cases we will leave the old functionality
  in so as not to break existing code which uses \imp. Such code is
  said to be "deprecated". See \salilab{imp/doc/doxygen/deprecated.html,
  the deprecated class list} for a list of such classes.

  Deprecated classes are marked in a variety of ways:
  - They are listed in the "Deprecated List" in the "Related Pages" tab.
  - They are noted as deprecated in their documentation.
  - They print a warning when an instance is constructed or the function
    is called.

  The warnings can be turned off using the
  IMP::core::set_print_deprecation_messages function.
  \param[in] old_classname The class which is deprecated.
  \param[in] replacement_classname The class which replaces it.

  Further, \imp can be built without deprecated code by defining
  \c IMP_NO_DEPRECATED or the \c deprecated=False \c scons argument.

  You should also use the \deprecated command in the doxygen documentation.
 */
#define IMP_DEPRECATED(old_classname, replacement_classname)           \
  if (::IMP::internal::get_print_deprecation_message(#old_classname)) { \
    IMP_LOG(WARNING, "WARNING: Class " << #old_classname                \
            << " is deprecated "                                        \
            << "and should not be used.\nUse "                          \
            << #replacement_classname << " instead." << std::endl);     \
    ::IMP::internal::set_printed_deprecation_message(#old_classname,    \
                                                           true);       \
  }

#ifdef __GNUC__
#define IMP_DEPRECATED_WARN __attribute__ ((deprecated))
#else
#define IMP_DEPRECATED_WARN
#endif

#endif /* IMPKERNEL_DEPRECATION_H */
