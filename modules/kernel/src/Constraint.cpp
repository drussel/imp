/**
 *  \file Constraint.cpp \brief Shared score state.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/Constraint.h"
#include "IMP/internal/utility.h"

IMP_BEGIN_NAMESPACE

Constraint::Constraint(std::string name) :
  ScoreState(name)
{
}

Constraint::Constraint(Model *m, std::string name) :
  ScoreState(m, name)
{
}

IMP_END_NAMESPACE
