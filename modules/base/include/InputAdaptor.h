/**
 *  \file IMP/base/InputAdaptor.h    \brief Basic types used by IMP.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_INPUT_ADAPTOR_H
#define IMPBASE_INPUT_ADAPTOR_H

#include "base_config.h"

IMPBASE_BEGIN_NAMESPACE
/** An input adaptor is a convenience class that allows functions to accept a
    wider variety of inputs than they would otherwise. The InputAdaptor class
    itself should not be used directly, it is just provided for purposes of
    documentation.
*/
class InputAdaptor {
protected:
  InputAdaptor(){}
};
IMPBASE_END_NAMESPACE

#endif  /* IMPBASE_INPUT_ADAPTOR_H */
