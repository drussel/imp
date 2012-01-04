/**
 *  \file atom/python_only.h    \brief functionality only availble in python.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_PYTHON_ONLY_H
#define IMPATOM_PYTHON_ONLY_H

#include "atom_config.h"

IMPATOM_BEGIN_NAMESPACE

/** \name Python Only
    The following functions are only availale in Python as the
    equivalent C++ functionality is provided via template
    functions or in other ways that don't directly map to
    python.
    @{
*/
#ifdef IMP_DOXYGEN
/** Print out the molecular hierarchy. Equivalent to
\code
IMP::core::show(h);
\endcode
*/
void show_molecular_hiearchy(Hierarchy h);
#endif
/** @} */

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_PYTHON_ONLY_H */
