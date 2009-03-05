/**
 *  \file SingletonScore.h    \brief A Score on a single particle.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMP_SINGLETON_SCORE_H
#define IMP_SINGLETON_SCORE_H

#include "config.h"
#include "base_types.h"
#include "RefCountedObject.h"
#include "Pointer.h"
#include "DerivativeAccumulator.h"

IMP_BEGIN_NAMESPACE

class Particle;

//! Abstract score function for a single particle.
/** SingletonScores should take a UnaryFunction as their first
    argument if such is needed.
*/
class IMPEXPORT SingletonScore : public RefCountedObject
{
public:
  SingletonScore();
  //! Compute the score for the particle and the derivative if needed.
  virtual Float evaluate(Particle *a,
                         DerivativeAccumulator *da) const = 0;
  //! Print information about the SingletonScore to a stream.
  /** Should end in a newline.
   */
  virtual void show(std::ostream &out=std::cout) const = 0;
  IMP_REF_COUNTED_DESTRUCTOR(SingletonScore)
};

IMP_OUTPUT_OPERATOR(SingletonScore);

IMP_END_NAMESPACE

#endif  /* IMP_SINGLETON_SCORE_H */
