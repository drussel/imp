/**
 *  \file MinMaxGroupnameScore.h    \brief Define GroupnameScore.
 *
 *  This file is generated by a script (tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_MINMAX_GROUPNAME_SCORE_H
#define IMPCONTAINER_MINMAX_GROUPNAME_SCORE_H

#include "container_config.h"
#include <IMP/GroupnameScore.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Evaluate the minmax n groupname scores of the passed set of GroupnameScores
/** Each of the set of GroupnameScores is evaluated and the sum of the
    minimum n is returned.
*/
class IMPCONTAINEREXPORT MinMaxGroupnameScore : public GroupnameScore
{
  GroupnameScores scores_;
  unsigned int n_;
public:
  MinMaxGroupnameScore(const GroupnameScoresTemp &scores,
                       unsigned int n=1,
                       std::string name="GroupnameScore %1%");
  IMP_GROUPNAME_SCORE(MinMaxGroupnameScore);
};

IMP_OBJECTS(MinMaxGroupnameScore,MinMaxGroupnameScores);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MINMAX_GROUPNAME_SCORE_H */
