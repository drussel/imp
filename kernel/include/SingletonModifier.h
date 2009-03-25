/**
 *  \file SingletonModifier.h    \brief A Modifier on Particles
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMP_SINGLETON_MODIFIER_H
#define IMP_SINGLETON_MODIFIER_H

#include "config.h"
#include "internal/kernel_version_info.h"
#include "internal/container_helpers.h"
#include "SingletonContainer.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of Particles
/** The primary function of such a class is to change
    the passed particles.
 */
class IMPEXPORT SingletonModifier : public RefCountedObject
{
public:
  SingletonModifier();

  /** Apply the function to a single value*/
  virtual void apply(Particle *a,
                     DerivativeAccumulator *da) const=0;

  /** Apply the function to a single value*/
  virtual void apply(Particle *a) const=0;

  /** Print out information about the function, ending in a newline.*/
  virtual void show(std::ostream &out = std::cout) const=0;

  /** return information about the authors */
  virtual VersionInfo get_version_info() const = 0;

  ~SingletonModifier(){}
};

IMP_OUTPUT_OPERATOR(SingletonModifier)

//! Apply the SingletonModifier to each element of the sequence
/** \relates SingletonModifier */
template <class It>
void apply(const SingletonModifier& f, It b, It e) {
  for (It c=b; c != e; ++c) {
    internal::ContainerTraits<Particle>::apply(&f, *c);
  }
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier& f,
                               Particles &ps) {
  apply(f, ps.begin(), ps.end());
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier& f,
                                SingletonContainer *ps) {
  apply(f, ps->particles_begin(), ps->particles_end());
}


//! Apply the SingletonModifier to each element of the sequence
/** \relates SingletonModifier */
template <class It>
void apply(const SingletonModifier& f, DerivativeAccumulator *da, It b, It e) {
  for (It c=b; c != e; ++c) {
    internal::ContainerTraits<Particle>::apply(&f, *c, da);
  }
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier& f,
                            DerivativeAccumulator *da,
                            Particles &ps) {
  apply(f, da, ps.begin(), ps.end());
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier& f,
                            DerivativeAccumulator *da,
                            SingletonContainer *ps) {
  apply(f, da, ps->particles_begin(), ps->particles_end());
}


//! Apply the SingletonModifier to each element of the sequence
/** \relates SingletonModifier */
template <class It>
void apply(const SingletonModifier* f, It b, It e) {
  for (It c=b; c != e; ++c) {
    internal::ContainerTraits<Particle>::apply(f, *c);
  }
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier* f,
                            Particles &ps) {
  apply(f, ps.begin(), ps.end());
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier* f,
                            SingletonContainer *ps) {
  apply(f, ps->particles_begin(), ps->particles_end());
}


//! Apply the SingletonModifier to each element of the sequence
/** \relates SingletonModifier */
template <class It>
void apply(const SingletonModifier* f, DerivativeAccumulator *da, It b, It e) {
  for (It c=b; c != e; ++c) {
    internal::ContainerTraits<Particle>::apply(f, *c, da);
  }
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier* f,
                            DerivativeAccumulator *da,
                            Particles &ps) {
  apply(f, da, ps.begin(), ps.end());
}

//! Apply a SingletonModifier to each in the Particles
/** \relates SingletonModifier */
IMPEXPORT inline void apply(const SingletonModifier* f,
                            DerivativeAccumulator *da,
                            SingletonContainer *ps) {
  apply(f, da, ps->particles_begin(), ps->particles_end());
}

//! A collection
typedef std::vector<SingletonModifier*> SingletonModifiers;

IMP_END_NAMESPACE

#endif  /* IMP_SINGLETON_MODIFIER_H */
