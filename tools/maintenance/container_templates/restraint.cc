/**
 *  \file CLASSNAMEsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/CLASSNAMEsRestraint.h"


IMPCONTAINER_BEGIN_NAMESPACE

CLASSNAMEsRestraint
::CLASSNAMEsRestraint(CLASSNAMEScore *ss,
                      CLASSNAMEContainer *pc,
                      std::string name): P(ss, pc, name) {

}

void CLASSNAMEsRestraint::do_show(std::ostream& out) const
{
  P::do_show(out);
}

IMPCONTAINER_END_NAMESPACE