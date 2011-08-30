/**
 *  \file MINORMAXCLASSNAMEScore.h    \brief Define CLASSNAMEScore.
 *
 *  This file is generated by a script (tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_MINORMAXHEADERNAME_HEADERNAME_SCORE_H
#define IMPCONTAINER_MINORMAXHEADERNAME_HEADERNAME_SCORE_H

#include "container_config.h"
#include <IMP/CLASSNAMEScore.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Evaluate the min or max n FUNCTIONNAME scores of the passed set
/** Each of the set of CLASSNAMEScores is evaluated and the sum of the
    minimum n is returned.
*/
class IMPCONTAINEREXPORT MINORMAXCLASSNAMEScore : public CLASSNAMEScore
{
  CLASSNAMEScores scores_;
  unsigned int n_;
public:
  MINORMAXCLASSNAMEScore(const CLASSNAMEScoresTemp &scores,
                       unsigned int n=1,
                       std::string name="CLASSNAMEScore %1%");
  IMP_HEADERNAME_SCORE(MINORMAXCLASSNAMEScore);

  Restraints create_current_decomposition(ARGUMENTTYPE vt) const;
};

IMP_OBJECTS(MINORMAXCLASSNAMEScore,MINORMAXCLASSNAMEScores);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MINORMAXHEADERNAME_HEADERNAME_SCORE_H */
