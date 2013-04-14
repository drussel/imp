/**
 *  \file internal/utility.h
 *  \brief Various useful utilities
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_INTERNAL_STATIC_H
#define IMPKERNEL_INTERNAL_STATIC_H

#include <IMP/kernel/kernel_config.h>
#include <IMP/base/map.h>
#include <IMP/base/Pointer.h>
#include <IMP/kernel/Undecorator.h>




IMPKERNEL_BEGIN_INTERNAL_NAMESPACE

extern IMPKERNELEXPORT
base::map<std::string, base::Pointer<Undecorator> > undecorators;


// lazy
#if !defined(IMP_DOXYGEN) && !defined(SWIG)
template <class Decorator>
class DecoratorUndecorator: public Undecorator {
  DecoratorUndecorator(Model *m, std::string name): Undecorator(m, name) {}
  virtual void teardown(ParticleIndex pi) const IMP_OVERRIDE {
    Decorator::teardown(Undecorator::get_model(), pi);
  }
};

/** Create an undecorator for a given Decorator that calls the
    decorator types teardown_particle() method.*/
template <class Decorator>
Undecorator* create_undecorator(Model *m, std::string name) {
  if (undecorators.find(name) == undecorators.end()) {
    undecorators[name] = new DecoratorUndecorator<Decorator>(m, name);
  }
  return undecorators.find(name)->second;
}


template <class Decorator, class Traits>
class TraitsDecoratorUndecorator: public Undecorator {
  Traits tr_;
  TraitsDecoratorUndecorator(Model *m, Traits tr, std::string name):
      Undecorator(m, name), tr_(tr) {}
  virtual void teardown(ParticleIndex pi) const IMP_OVERRIDE {
    Decorator::teardown(Undecorator::get_model(), pi, tr_);
  }
};

/** Create an undecorator for a given Decorator that calls the
    decorator types teardown_particle() method.*/
template <class Decorator, class Traits>
Undecorator* create_traits_undecorator(Model *m, Traits tr,
                                       std::string name) {
  return new TraitsDecoratorUndecorator<Decorator, Traits>(m, tr, name);
}

#endif

IMPKERNEL_END_INTERNAL_NAMESPACE

#endif  /* IMPKERNEL_INTERNAL_STATIC_H */
