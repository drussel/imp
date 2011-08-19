/**
 *  \file Scale.h
 *  \brief A decorator for scale parameters particles
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */


#ifndef IMPISD_SCALE_H
#define IMPISD_SCALE_H

#include "isd_config.h"

#include <IMP/PairContainer.h>
#include <IMP/SingletonContainer.h>
#include <IMP/Decorator.h>
#include <IMP/isd/Nuisance.h>

IMPISD_BEGIN_NAMESPACE

//! Add scale parameter to particle
/** The value of the scale parameter may express data
    or theory uncertainty. It is positive. It can be initialized with or without    specifying bounds. Default bounds: 0 and infinity. Setting it to values
    outside of bounds results in setting it to the bound value with a warning.
 */
class IMPISDEXPORT Scale: public Decorator
{
public:
  static Scale setup_particle(Particle *p, double scale=1.0, 
          double lower=0.0, 
          double upper=-1.0); //std::numeric_limits<double>::infinity()); 

  static bool particle_is_instance(Particle *p) {
    return p->has_attribute(get_scale_key());
  }

  Float get_scale() const {
    return get_particle()->get_value(get_scale_key());
  }

  Float get_nuisance() const { return get_scale(); }

  Float get_upper() const {
    double upper = get_particle()->get_value(get_upper_key());
    if (upper < -0.5) {
        return std::numeric_limits<double>::infinity(); 
    } else {
        return upper;
    }
  }

  Float get_lower() const {
    return get_particle()->get_value(get_lower_key());
  }

  void set_upper(Float d);
  
  void set_lower(Float d); 

  void set_scale(Float d); 

  void set_nuisance(Float d) {set_scale(d);}

  void add_to_scale_derivative(Float d, DerivativeAccumulator &accum) {
    get_particle()->add_to_derivative(get_scale_key(), d, accum);
  }

  void add_to_nuisance_derivative(Float d, DerivativeAccumulator &accum)
  {
      add_to_scale_derivative(d, accum);
  }

  Float get_scale_derivative() const
  {
    return get_particle()->get_derivative(get_scale_key());
  }

  Float get_nuisance_derivative() const {return get_scale_derivative(); }

  IMP_DECORATOR(Scale, Decorator);

  static FloatKey get_scale_key();

  static FloatKey get_nuisance_key() { return get_scale_key(); }

  static FloatKey get_upper_key();
  
  static FloatKey get_lower_key();

};

typedef Decorators<Scale, Particles> Scales;

IMPISD_END_NAMESPACE

#endif  /* IMPISD_SCALE_H */
