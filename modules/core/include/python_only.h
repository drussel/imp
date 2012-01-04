/**
 *  \file core/python_only.h    \brief functionality only availble in python.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_PYTHON_ONLY_H
#define IMPCORE_PYTHON_ONLY_H

#include "core_macros.h"

IMPCORE_BEGIN_NAMESPACE

/** \name Python Only
    The following functions are only availale in Python as the
    equivalent C++ functionality is provided via template
    functions or in other ways that don't directly map to
    python.
    @{
*/
#ifdef IMP_DOXYGEN
/** Align any combination of XYZ objects and algebra::Vector3D
    objects to one another.

    \note each of the two lists must all have the same type.
    That is, list "a" cannot be a
    mix of algebra::Vector3D objects and XYZ objects.
\code
IMP::algebra::get_transformation_aligning_first_to_second(a,b);
\endcode
 */
algebra::Transformation3D
get_transformation_aligning_first_to_second(XYZsOrVector3ds a,
                                            XYZsOrVector3ds b);
#endif
/** @} */

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PYTHON_ONLY_H */
