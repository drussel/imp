/**
 *  \file SimpleDiscreteRestraint.h
 *  \brief Simple restraint for testing
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPDOMINO_SIMPLE_DISCRETE_RESTRAINT_H
#define IMPDOMINO_SIMPLE_DISCRETE_RESTRAINT_H

#include "domino_config.h"

#include <IMP/Model.h>
#include <IMP/Restraint.h>

#include <string>
#include <climits>

IMPDOMINO_BEGIN_NAMESPACE

/**
   \ingroup restraint
 */
class IMPDOMINOEXPORT SimpleDiscreteRestraint : public Restraint
{
public:
  //! Constructor.
  /** \exception ErrorException the restraint file is of an invalid format.
   */
  SimpleDiscreteRestraint(Model& model_, std::string restraint_filename,
                          Particle *p1, Particle *p2);
  IMP_RESTRAINT(SimpleDiscreteRestraint);

  IMP_LIST(private, Particle, particle, Particle*, Particles);
protected:
  void load_restraints(std::string restraint_filename);
  Model *model;
  std::pair<int, int> key;
  Particle *p1, *p2;
  std::map<std::pair<int, int>, std::map<std::pair<int, int>,
           float> > states2values; // should be static and in a different class
};

IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_SIMPLE_DISCRETE_RESTRAINT_H */
