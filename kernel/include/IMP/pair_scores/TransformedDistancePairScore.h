/**
 *  \file TransformedDistancePairScore.h
 *  \brief A Score on the distance between a pair of particles.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef __IMP_TRANSFORMED_DISTANCE_PAIR_SCORE_H
#define __IMP_TRANSFORMED_DISTANCE_PAIR_SCORE_H

#include "../PairScore.h"
#include "../UnaryFunction.h"
#include "../Pointer.h"
#include "../Vector3D.h"

namespace IMP
{

/** \brief  Apply a function to the distance between two particles
    after transforming the first

    Apply a transform to the second particle and then apply the unary
    function to the distance between the transformed particle and the
    second. This can be used to implement symmetry restraints.

    The second particle, x, is transformed as R*(x-center)+ translation+center

    \ingroup pairscore
 */
class IMPDLLEXPORT TransformedDistancePairScore : public PairScore
{
  Pointer<UnaryFunction> f_;
  Vector3D tc_, c_;
  Vector3D r_[3], ri_[3];
public:
  TransformedDistancePairScore(UnaryFunction *f);
  virtual ~TransformedDistancePairScore(){}
  virtual Float evaluate(Particle *a, Particle *b,
                         DerivativeAccumulator *da) const;
  virtual void show(std::ostream &out=std::cout) const;

  void set_rotation(float r00, float r01, float r02,
                    float r10, float r11, float r12,
                    float r20, float r21, float r22);
  void set_translation(float t0, float t1, float t2);
  void set_center(float t0, float t1, float t2);
};

} // namespace IMP

#endif  /* __IMP_TRANSFORMED_DISTANCE_PAIR_SCORE_H */
