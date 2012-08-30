/**
 *  \file IMP/CLASSNAMEDerivativeModifier.h
 *  \brief A Modifier on PLURALVARIABLETYPE
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_HEADERNAME_DERIVATIVE_MODIFIER_H
#define IMPKERNEL_HEADERNAME_DERIVATIVE_MODIFIER_H

#include "kernel_config.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "internal/container_helpers.h"


IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of PLURALVARIABLETYPE
/** The primary function of such a class is to change
    the derivatives of the passed particles.

    Implementors should see and
    IMP_HEADERNAME_DERIVATIVE_MODIFIER() and
    IMP::CLASSNAMEModifier.
 */
class IMPEXPORT CLASSNAMEDerivativeModifier : public base::Object
{
public:
  typedef VARIABLETYPE Argument;
  typedef INDEXTYPE IndexArgument;
  CLASSNAMEDerivativeModifier(std::string name="CLASSNAMEModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(ARGUMENTTYPE,
                     DerivativeAccumulator &da) const=0;

 /** Apply the function to a single value*/
  virtual void apply_index(Model *m, PASSINDEXTYPE v,
                           DerivativeAccumulator &da) const {
    apply(internal::get_particle(m, v), da);
  }

  /** Apply the function to a collection of PLURALVARIABLETYPE */
  virtual void apply_indexes(Model *m, const PLURALINDEXTYPE &o,
                             DerivativeAccumulator &da) const {
    for (unsigned int i=0; i < o.size(); ++i) {
      apply_index(m, o[i], da);
    }
  }

  /** Get the set of particles read when applied to the arguments.*/
  virtual ParticlesTemp
    get_input_particles(Particle* p) const =0;
  /** Get the set of particles modifier when applied to the arguments.*/
  virtual ParticlesTemp
    get_output_particles(Particle *p) const =0;
  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(Particle *p) const =0;
  /** Get the set of output containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_output_containers(Particle *p) const =0;
};


IMP_OBJECTS(CLASSNAMEDerivativeModifier,CLASSNAMEDerivativeModifiers);


IMP_END_NAMESPACE

#endif  /* IMPKERNEL_HEADERNAME_DERIVATIVE_MODIFIER_H */
