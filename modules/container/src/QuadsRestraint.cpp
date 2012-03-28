/**
 *  \file QuadsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/QuadsRestraint.h"


IMPCONTAINER_BEGIN_NAMESPACE

QuadsRestraint
::QuadsRestraint(QuadScore *ss,
                      QuadContainerInput pc,
                      std::string name): P(ss, pc, name) {

}

void QuadsRestraint::do_show(std::ostream& out) const
{
  P::do_show(out);
}

IMPCONTAINER_END_NAMESPACE
