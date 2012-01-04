/**
 *  \file UnaryFunction.cpp  \brief Single variable function.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/UnaryFunction.h>

IMP_BEGIN_NAMESPACE

UnaryFunction::UnaryFunction(): Object("UnaryFunction%1%")
{
  /* Implemented here rather than in the header so that UnaryFunction
     symbols are present in the kernel DSO */
}

IMP_END_NAMESPACE
