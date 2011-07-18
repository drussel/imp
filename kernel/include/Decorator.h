/**
 *  \file Decorator.h    \brief The base class for decorators.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_DECORATOR_H
#define IMP_DECORATOR_H

#include "Object.h"
#include "Pointer.h"
#include "utility.h"
#include "Model.h"
#include "Constraint.h"
#include "internal/utility.h"

IMP_BEGIN_NAMESPACE
class Model;

/**
Representation of the structure in \imp is via a collection of
Particle objects. However, since particles are general purpose, they
provide a basic set of tools for managing the data (e.g.
IMP::Particle::add_attribute(), IMP::Particle::get_value()
etc). Decorators wrap (or \quote{decorate}) particles to provide a much
richer interface. For example, most particles have Cartesian
coordinates. The class IMP::core::XYZ decorates such a particle to
provide functions to get and set the Cartesian coordinates as well as
compute distances between particles.
\code
d0= IMP.core.XYZ(p0)
d1= IMP.core.XYZ(p1)
print IMP.core.distance(d0,d1)
print d0.get_coordinates()
\endcode

\par Decorator basics

Dealing with decorators and particles has two main parts
-# setting up the particle to be used with that decorator
-# decorating the particle.

To set up a particle to be used with the IMP::core::XYZ decorator we do
\code
d0= IMP.core.XYZ.setup_particle(p, IMP.algebra.Vector3D(0,2,3))
\endcode
The method calls also decorates the particle and returns the decorator
which can now be used to manipulate the particle. For example we can
access the coordinates \c (0,2,3) by doing
\code
print d0.get_coordinates()
\endcode
We now say the particle is an XYZ particle. If that particle is
encountered later when we do not have the existing decorator available,
we can decorate it again (since it is already set up) by doing
\code
d1= IMP.core.XYZ(p)
\endcode

If you do not know if \c p has been set up for the XYZ decorator, you can
ask with
\code
if IMP.core.XYZ.particle_is_instance(p):
\endcode

More abstractly, decorators can be used to
- maintain invariants: e.g. an IMP::atom::Bond particle always connects
  two other particles, both of which are IMP::atom::Bonded particles.
- add functionality: e.g. you can get the coordinates as an
  IMP::algebra::VectorD<3>
- provide uniform names for attributes: so you do not use \quote{x} some places
and \quote{X} other places

To see a list of all available decorators and to see what functions
all decorators have, look at the list of classes which inherit from
IMP::Decorator, below.

See the IMP::example::ExampleDecorator %example for how to implement a
simple decorator.

\note Decorator objects are ordered based on the address of the wrapped
particle. Like pointers, they are logical values so can be in \c if
statements.

\implementation{Decorator, IMP_DECORATOR, IMP::example::ExampleDecorator}
\n\n For efficiency reasons attribute keys should always be created
lazily (at the time of the first use), and not be created as static
variables. The reason for this is that initialized attribute keys result
in space being allocated in decorators, even before they are used.\n\n
Implementors should consult IMP::example::ExampleDecorator,
IMP_DECORATOR(), IMP_DECORATOR_WITH_TRAITS(), IMP_DECORATOR_GET(),
IMP_DECORATOR_ARRAY_DECL().

\advanceddoc Lists of decorators are reference counted even though the
individual decorators are not. For more efficiency
you can use the non-reference counted version, IMP::core::XYZsTemp
instead. This should only
be done when it is known to be safe. If you can't figure out
that it is, don't do it.

A decorator can be cast to a IMP::Particle* in C++. You have to
use the Decorator::get_particle() function in Python.

\see DecoratorWithTraits
*/
class Decorator
{
private:
  Model *model_;
  ParticleIndex pi_;
protected:
  Decorator(Model *m, ParticleIndex pi): model_(m),
                                         pi_(pi) {}
  Decorator(Particle *p): model_(p->get_model()),
                          pi_(p->get_index()){}
  Decorator() :model_(NULL), pi_(-1)
  {}
  int compare(Particle *o) const {
    if (o < get_particle()) return -1;
    else if (o > get_particle()) return 1;
    else return 0;
  }
public:
 ParticleIndex get_particle_index() const {
    return pi_;
  }
#ifdef _MSC_VER
  // needed to get Particle in VectorOfRefCounted
  typedef Particle* ParticleP;
#endif
#ifndef IMP_DOXYGEN
  bool __eq__(Particle* o) const {
    return operator==(o);
  }
  bool __ne__(Particle* o) const {
    return operator!=(o);
  }
  bool __lt__(Particle* o) const {
    return operator<(o);
  }
  bool __gt__(Particle* o) const {
    return operator>(o);
  }
  bool __ge__(Particle* o) const {
    return operator>=(o);
  }
  bool __le__(Particle* o) const {
    return operator<=(o);
  }
#ifndef SWIG
  bool operator==(Particle* o) const {
    return (compare(o) == 0);
  }
  bool operator!=(Particle* o) const {
    return (compare(o) != 0);
  }
  bool operator<(Particle* o) const {
    return (compare(o) <0);
  }
  bool operator>(Particle* o) const {
    return (compare(o) > 0);
  }
  bool operator>=(Particle* o) const {
    return !(compare(o) < 0);
  }
  bool operator<=(Particle* o) const {
    return !(compare(o) > 0);
  }
#endif
#endif

  /** \name Methods provided by the Decorator class
      The following methods are provided by the Decorator class.
      @{
  */

  /** Returns the particle decorated by this decorator.*/
  Particle *get_particle() const {
    if (!model_) return NULL;
    else return model_->get_particle(pi_);
  }

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  operator Particle*() const {
    return get_particle();
  }
  Particle* operator->() const {
    return get_particle();
  }
#endif

  /** \brief Returns the Model containing the particle. */
  Model *get_model() const {
    return model_;
  }
  // here just to make the docs symmetric
private:
  IMP_ONLY_DOXYGEN(int blah_);
  //! @}
public:
  IMP_HASHABLE_INLINE(Decorator, return boost::hash_value(get_particle()););
#ifdef IMP_DOXYGEN

  /** \name Methods that all decorators must have
      All decorators must have the following methods. Decorators
      which are parameterized (for example IMP::core::XYZR)
      take an (optional) extra parameter after the Particle in
      setup_particle(), and particle_is_instance().
      \note these are
      not actually methods of the Decorator class itself.
      @{
  */
  /** Add the needed attributes to the particle and initialize them
      with values taken from initial_values.

      It is an error to call this twice on the same particle for
      the same type of decorator.
  */
  static Decorator setup_particle(Particle *p, extra_arguments);

  /** \brief Return true if the particle can be cast to the decorator.

  That is, if particle_is_instance() returns \c true, then it is
  legal to construct an instance of the decorator with that particle.
  If not, setup_particle() must be called first.
  \code
  IMP::Particle *p = new IMP::Particle(m);
  // it is false
  std::cout << IMP::core::XYZ::particle_is_instance(p) << std::endl;
  // As a result this is an error
  IMP::core::XYZ d(p);
  // now set it up
  IMP::core::XYZ(p);
  // now it is true
  std::cout << IMP::core::XYZ::particle_is_instance(p) << std::endl;
  // and now this code is OK
  IMP::core::XYZ d(p);
  \endcode
  */
  static bool particle_is_instance(Particle *p);

  /** Create an instance of the Decorator from the particle that has
      already been set up. The particle must have been set up already
      (eg particle_is_instance(p) must be true), but this is not
      necessarily checked.
  */
  Decorator(Particle *p);
  /** The default constructor must be defined and create a NULL decorator,
      analogous to a \c NULL pointer in C++ or a \c None object in Python.
  */
  Decorator();
  //! @}
#endif
#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  typedef boost::false_type DecoratorHasTraits;
#endif
};


#ifndef IMP_DOXYGEN

/** A class to add ref counting to a decorator */
template <class D>
class RefCountingDecorator: public D {
public:
  RefCountingDecorator(){}
  RefCountingDecorator(const D &d): D(d){internal::ref(D::get_particle());}
  ~RefCountingDecorator(){ if (*this) internal::unref(D::get_particle());}
#ifndef SWIG
  void operator=(const D &d) {
    if (*this) {
      internal::unref(D::get_particle());
    }
    D::operator=(d);
    if (*this) {
      internal::ref(D::get_particle());
    }
  }
  const D&get_decorator() const {
    return static_cast<const D&>(*this);
  }
  D&get_decorator() {
    return static_cast<D&>(*this);
  }
#endif
};


#define IMP_CONSTRAINT_DECORATOR_DECL(Name)                             \
  private:                                                              \
  static ObjectKey get_constraint_key();                                \
  static void set_constraint(SingletonModifier* before,                 \
                             SingletonModifier *after, Particle *p);    \
public:                                                                 \
 Constraint *get_constraint() const {                                   \
   return dynamic_cast<Constraint*>(get_particle()                      \
                                    ->get_value(get_constraint_key())); \
 }                                                                      \
 IMP_REQUIRE_SEMICOLON_CLASS(constraint)


#define IMP_CONSTRAINT_DECORATOR_DEF(Name)                              \
  ObjectKey Name::get_constraint_key() {                                \
    static ObjectKey ret(#Name " score state");                         \
    return ret;                                                         \
  }                                                                     \
  void Name::set_constraint(SingletonModifier* before,                  \
                            SingletonModifier *after,                   \
                            Particle *p) {                              \
    if (!after && !before ) {                                           \
      if (p->has_attribute(get_constraint_key())) {                     \
        p->get_model()->remove_score_state(dynamic_cast<ScoreState*>    \
                                 (p->get_value(get_constraint_key()))); \
        p->remove_attribute(get_constraint_key());                      \
      }                                                                 \
    } else {                                                            \
      Constraint *ss= new SingletonConstraint(before,                   \
                                              after, p,                 \
                      std::string(#Name "updater for ")+p->get_name()); \
      p->add_attribute(get_constraint_key(), ss);                       \
      p->get_model()->add_score_state(ss);                              \
    }                                                                   \
  }                                                                     \
  IMP_REQUIRE_SEMICOLON_NAMESPACE

#endif

#ifndef SWIG
/** Register a function that can be used to check that the particle
    is valid with respect to the decorator. The function should take
    a Particle* as an argument and return a bool. It should throw
    an exception if something is wrong.

    This macro should only be used in a .cpp file.
*/
#define IMP_CHECK_DECORATOR(Name, function) \
  IMP::internal::ParticleCheck \
  Name##pc(Name::particle_is_instance, function);
#endif

IMP_END_NAMESPACE

#if !defined(SWIG) && !defined IMP_DOXYGEN
IMP_BEGIN_INTERNAL_NAMESPACE
inline void unref(Decorator d) {
  return unref(static_cast<Particle*>(d));
}
inline void release(Decorator d) {
  return release(static_cast<Particle*>(d));
}
inline void ref(Decorator d) {
  return ref(static_cast<Particle*>(d));
}

IMP_END_INTERNAL_NAMESPACE

#endif

#include "Decorators.h"

#endif  /* IMP_DECORATOR_H */
