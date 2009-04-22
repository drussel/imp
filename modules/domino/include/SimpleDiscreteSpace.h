/**
 *  \file SimpleDiscreteSpace.h   \brief for debugging
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPDOMINO_SIMPLE_DISCRETE_SPACE_H
#define IMPDOMINO_SIMPLE_DISCRETE_SPACE_H

#include "config.h"
#include "DiscreteSampler.h"
#include "DiscreteSet.h"
#include <IMP/Particle.h>
#include <map>
#include <sstream>


IMPDOMINO_BEGIN_NAMESPACE

class IMPDOMINOEXPORT SimpleDiscreteSpace : public DiscreteSet
{
public:
  static FloatKey get_optimization_key();

  SimpleDiscreteSpace(int number_of_states): m_(new Model()){
    atts_.push_back(get_optimization_key());
    Particle * p;
    for (int j = 0;j < number_of_states;j++) {
      p = new Particle(m_);
      p->add_attribute(get_optimization_key(),j,true);
      states_.push_back(p);
    }
  }

  ~SimpleDiscreteSpace();

protected:
  Pointer<Model> m_;
};

IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_SIMPLE_DISCRETE_SPACE_H */
