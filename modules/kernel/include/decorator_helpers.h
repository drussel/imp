/**
 *  \file IMP/kernel/decorator_helpers.h
 *  \brief The base class for decorators.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_DECORATOR_HELPERS_H
#define IMPKERNEL_DECORATOR_HELPERS_H

#include <IMP/kernel/kernel_config.h>
#include "Decorator.h"

IMPKERNEL_BEGIN_NAMESPACE

/** Implementation class. */
template <class Implementation>
class ImplementDecorator: public Implementation {
  void add_undecorator(Model* m, ParticleIndex pi) {
    m->add_undecorator(pi, internal::create_undecorator<Implementation>(m,
                                             Implementation::get_name()));
  }
 public:
  typedef ImplementDecorator<Implementation> This;
  static bool get_was_setup(Model *m, ParticleIndex pi) {
    return Implementation::do_get_is_setup(m, pi);
  }
  static This teardown(Model *m, ParticleIndex pi) {
    IMP_USAGE_CHECK(get_was_setup(m, pi),
                    "Particle is not an instance in teardown");
    Implementation::do_teardown(m, pi);
  }
  void show(std::ostream &out) {
    Implementation::do_show(out);
  }
#ifndef IMP_DOXYGEN
  // backwards compat
  static bool particle_is_instance(Model *m, ParticleIndex pi) {
    return get_was_setup(m, pi);
  }
  static void teardown_particle(Model *m, ParticleIndex pi) {
    teardown(m, pi);
  }

  static bool particle_is_instance(Particle *p) {
    return get_was_setup(p->get_model(), p->get_index());
  }
  static void teardown_particle(Particle *p) {
    teardown(p->get_model(), p->get_index());
  }

  static This decorate_particle(Particle *p) {
    if (!get_was_setup(p->get_model(), p->get_index())) {
      setup(p->get_model(), p->get_index());
    }
    return This(p);
  }
#endif
};


/** Use this class to help implement decorators in C++. To use it, write
    a implementation class that inherits from the parent and
    implements the (protected) static methods
    - bool do_get_is_setup(Model *m, ParticleIndex pi)
    - void do_setup(Model *m, ParticleIndex pi)
    - void do_teardown(Model *m, ParticleIndex pi)
    - std::string get_name()
    and the non-static method
    - void do_show(std::ostream)
    See ImplementDecorator1 if you want to take an argument to setup.
*/
template <class Implementation>
class ImplementDecorator0: public ImplementDecorator<Implementation> {
 public:
  IMP_COMPARISONS(ImplementDecorator0);
  IMP_COMPARISONS(Particle*);
  typedef ImplementDecorator0<Implementation> This;
  static This setup(Model *m, ParticleIndex pi) {
    Implementation::do_setup(m, pi);
    add_undecorator(m, pi);
    return This(m, pi);
  }
  ImplementDecorator0(Model *m, ParticleIndex pi) {
    IMP_USAGE_CHECK(get_was_setup(m, pi),
                    "Particle is not an instance in teardown");
    Decorator::initialize(m, pi);
  }
#ifndef IMP_DOXYGEN
  // backwards compat
  static This setup_particle(Model *m, ParticleIndex pi) {
    return setup(m, pi);
  }
  static This setup_particle(Particle *p) {
    return setup(p->get_model(), p->get_index());
  }

  ImplementDecorator0(Particle *p) {
    IMP_USAGE_CHECK(get_was_setup(m, pi),
                    "Particle is not an instance in teardown");
    Decorator::initialize(p->get_model(), p->get_index());
  }
#endif
};

/** Like ImplementDecorator0 but requres are argument to the setup functions.*/
template <class Implementation, class Argument>
class ImplementDecorator1: public ImplementDecorator<Implementation> {
 public:
  IMP_COMPARISONS(ImplementDecorator0);
  IMP_COMPARISONS(Particle*);
  typedef ImplementDecorator1<Implementation> This;
  static This setup(Model *m, ParticleIndex pi, const Argument &t) {
    Implementation::do_setup(m, pi, t);
    add_undecorator(m, pi);
    return This(m, pi);
  }
  ImplementDecorator1(Model *m, ParticleIndex pi) {
    IMP_USAGE_CHECK(get_was_setup(m, pi),
                    "Particle is not an instance in teardown");
    Decorator::initialize(m, pi);
  }
#ifndef IMP_DOXYGEN
  // backwards compat
  static This setup_particle(Model *m, ParticleIndex pi, const Argument &t) {
    return setup(m, pi, t);
  }
  static This setup_particle(Particle *p, const Argument &t) {
    return setup(p->get_model(), p->get_index(), t);
  }

  ImplementDecorator1(Particle *p) {
    IMP_USAGE_CHECK(get_was_setup(m, pi),
                    "Particle is not an instance in teardown");
    Decorator::initialize(p->get_model(), p->get_index());
  }
#endif
};

/** Like ImplementDecorator0 but requres are a traits argument.*/
template <class Implementation, class Traits>
class ImplementDecoratorTraits: public Implementation {
void add_undecorator(Model* m, ParticleIndex pi) {
    m->add_undecorator(pi,
                       internal::create_traits_undecorator<Implementation>(m),
                       Implementation::get_name());
  }
 public:
  IMP_COMPARISONS(ImplementDecorator0);
  IMP_COMPARISONS(Particle*);
  typedef ImplementDecoratorTraits<Implementation> This;
  static This setup(Model *m, ParticleIndex pi,
                    const Traits &t = Implementation::get_default_traits()) {
    Implementation::do_setup(m, pi, t);
    add_undecorator(m, pi, t);
    return This(m, pi);
  }

  ImplementDecoratorTraits(Model *m, ParticleIndex pi,
                           const Traits &t
                           = Implementation::get_default_traits()) {
    IMP_USAGE_CHECK(get_was_setup(m, pi, t),
                    "Particle is not an instance in teardown");
    ImplementDecorator<Implementation>::set_traits(t);
    ImplementDecorator<Implementation>::initialize(m, pi);
  }

  static bool get_was_setup(Model *m, ParticleIndex pi,
                            const Traits &t
                            = Implementation::get_default_traits()) {
    return Implementation::do_get_is_setup(m, pi, t);
  }

  static This teardown(Model *m, ParticleIndex pi, const Traits &t
                        = Implementation::get_default_traits()) {
    IMP_USAGE_CHECK(get_was_setup(m, pi, t),
                    "Particle is not an instance in teardown");
    Implementation::do_teardown(m, pi, t);
  }

#ifndef IMP_DOXYGEN
  // backwards compat
  static bool particle_is_instance(Model *m, ParticleIndex pi,
                                   const Traits &t
                                   = Implementation::get_default_traits()) {
    return get_was_setup(m, pi, t);
  }

  static void teardown_particle(Model *m, ParticleIndex pi,
                                const Traits &t
                                = Implementation::get_default_traits()) {
    teardown(m, pi, t);
  }

  static bool particle_is_instance(Particle *p, const Traits &t
                                   = Implementation::get_default_traits()) {
    return get_was_setup(p->get_model(), p->get_index(), t);
  }

  static void teardown_particle(Particle *p, const Traits &t
                                = Implementation::get_default_traits()) {
    teardown(p->get_model(), p->get_index(), t);
  }

  static This decorate_particle(Particle *p, const Traits &t
                                = Implementation::get_default_traits()) {
    if (!get_was_setup(p->get_model(), p->get_index(), t)) {
      setup(p->get_model(), p->get_index(), t);
    }
    return This(p, t);
  }

  static This setup_particle(Model *m, ParticleIndex pi, const Traits &t
                             = Implementation::get_default_traits()) {
    return setup(m, pi, t);
  }

  static This setup_particle(Particle *p, const Traits &t
                             = Implementation::get_default_traits()) {
    return setup(p->get_model(), p->get_index(), t);
  }

  ImplementDecorator1(Particle *p, const Traits &t
                      = Implementation::get_default_traits()) {
    IMP_USAGE_CHECK(get_was_setup(m, pi, t),
                    "Particle is not an instance in teardown");
    ImplementDecorator<Implementation>::set_traits(t);
    ImplementDecorator<Implementation>::initialize(p->get_model(),
                                                   p->get_index());
  }
#endif
};


IMPKERNEL_END_NAMESPACE


#endif  /* IMPKERNEL_DECORATOR_HELPERS_H */
