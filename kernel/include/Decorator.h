/**
 *  \file Decorator.h    \brief The base class for decorators.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_DECORATOR_H
#define IMP_DECORATOR_H

#include "Object.h"
#include "Pointer.h"
#include "utility.h"
#include "Particle.h"
#include "Constraint.h"
#include "Model.h"
#include "internal/IndexingIterator.h"

IMP_BEGIN_NAMESPACE

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
IMP_DECORATOR(), IMP_DECORATOR_TRAITS(), IMP_DECORATOR_GET(),
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
  Particle *particle_;
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
  friend bool operator==(Decorator, Particle*);
#endif
protected:
  Decorator(Particle *p): particle_(p) {}
  Decorator() :particle_(NULL)
  {}
public:
#ifdef _MSC_VER
  // needed to get Particle in VectorOfRefCounted
  typedef Particle* ParticleP;
#endif

  IMP_NO_DOXYGEN(typedef Decorator This);

  IMP_COMPARISONS_1(particle_);

  /** \name Methods provided by the Decorator class
      The following methods are provided by the Decorator class.
      @{
  */

  /** Returns the particle decorated by this decorator.*/
  Particle *get_particle() const {
    IMP_USAGE_CHECK(particle_,
                    "You must give the decorator a particle to decorate.");
    IMP_CHECK_OBJECT(particle_);
    return particle_;
  }

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  operator Particle*() const {
    return particle_;
  }
  Particle* operator->() const {
    return particle_;
  }
#endif

  /** \brief Returns the Model containing the particle. */
  Model *get_model() const {
    IMP_CHECK_OBJECT(particle_->get_model());
    return particle_->get_model();
  }
  // here just to make the docs symmetric
private:
  IMP_ONLY_DOXYGEN(int blah_;);
  //! @}
public:
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
  IMP_NO_DOXYGEN(bool is_null() const {return !particle_;});
  IMP_NO_DOXYGEN(typedef void (Decorator::*bool_type)() const);
  IMP_NO_DOXYGEN(void safe_bool_function() const {});
};


/** Certain decorators require \quote{traits} to customize their behavior.
These traits typically are used to allow one decorator class to provide
functionality that can be applied in a variety of contexts. Examples
include IMP::core::Hierarchy and IMP::core::XYZR.

Two DecoratorsWithTraits are equal only if the particle and the traits
are equal.
*/
template <class Base, class Traits>
class DecoratorWithTraits: public Base {
  typedef DecoratorWithTraits<Base,Traits> This;
  Traits traits_;
  int compare(const DecoratorWithTraits<Base, Traits> &o) const {
    if (traits_== o.traits_) {
      return Base::compare(o);
    } else {
      if (traits_ < o.traits_) return -1;
      else return 1;
    }
  }
protected:
  DecoratorWithTraits(){}
  DecoratorWithTraits(Particle *p, Traits tr): Base(p), traits_(tr){}
public:
  typedef Traits DecoratorTraits;
  const Traits& get_decorator_traits() const {return traits_;}
  IMP_COMPARISONS;
};


#ifndef IMP_DOXYGEN
inline bool operator==(Decorator d, Particle *p) {
  return d.particle_==p;
}
inline bool operator==(Particle *p, Decorator d) {
  return d==p;
}


#if !defined(SWIG)
#define IMP_DECORATORS_METHODS(test, on_add_decorator, on_add_particle, \
                               swap)                                    \
  struct Accessor {                                                     \
  typedef Accessor This;                                                \
  typedef WrappedDecorator result_type;                                 \
  typedef unsigned int argument_type;                                   \
  result_type operator()(argument_type i) const {                       \
    return o_->operator[](i);                                           \
  }                                                                     \
  Accessor(ThisDecorators *pc): o_(pc){}                                \
  Accessor(): o_(NULL){}                                                \
  IMP_COMPARISONS_1(o_);                                                \
private:                                                                \
/* This should be ref counted, but swig memory management
   is broken */                                                         \
ThisDecorators* o_;                                                     \
};                                                                      \
void check(Particle *p) {                                               \
  IMP_USAGE_CHECK(test,                                                 \
                  "Particle \"" << (p)->get_name()                      \
                  << "\" missing required attributes");                 \
}                                                                       \
template <class It>                                                     \
void check(It b, It e) {                                                \
  for (It c= b; c!= e; ++c) {                                           \
    check(*c);                                                          \
  }                                                                     \
}                                                                       \
public:                                                                 \
typedef const WrappedDecorator const_reference;                         \
typedef WrappedDecorator value_type;                                    \
typedef Proxy reference;                                                \
const ParticlesTemp &get_particles() const {return *this;}              \
void push_back(WrappedDecorator d) {                                    \
  on_add_decorator;                                                     \
  typename ParentDecorators::value_type pd=d;                           \
  ParentDecorators::push_back(pd);                                      \
}                                                                       \
void push_back(Particle *p) {                                           \
  check(p);                                                             \
  on_add_particle;                                                      \
  typename ParentDecorators::value_type pd=WrappedDecorator(p);         \
  ParentDecorators::push_back(pd);                                      \
}                                                                       \
void set(unsigned int i, WrappedDecorator d) {                          \
  typename ParentDecorators::value_type pd=d;                           \
  ParentDecorators::operator[](i)= pd;                                  \
}                                                                       \
WrappedDecorator back() const {                                         \
  IMP_USAGE_CHECK(!ParentDecorators::empty(),                           \
                  "Can't call back on empty Decorators");               \
  return WrappedDecorator(ParentDecorators::back());                    \
}                                                                       \
WrappedDecorator front() const {                                        \
  IMP_USAGE_CHECK(!ParentDecorators::empty(),                           \
                  "Can't call front on empty Decorators");              \
  return WrappedDecorator(ParentDecorators::front());                   \
}                                                                       \
typedef internal::IndexingIterator<Accessor> iterator;                  \
typedef internal::IndexingIterator<Accessor> const_iterator;            \
iterator begin() const {                                                \
  return iterator(Accessor(const_cast<ThisDecorators*>(this)), 0);      \
}                                                                       \
iterator end() const {                                                  \
  return iterator(Accessor(const_cast<ThisDecorators*>(this)),          \
                  ParentDecorators::size());                            \
}                                                                       \
template <class It>                                                     \
void insert(iterator loc, It b, It e) {                                 \
  check(b,e);                                                           \
  for (It c=b; c!= e; ++c) {                                            \
    on_add_particle;                                                    \
  }                                                                     \
  ParentDecorators::insert(ParentDecorators::begin()+(loc-begin()),     \
                           b, e);                                       \
}                                                                       \
void swap_with(ThisDecorators &o) {                                     \
  swap;                                                                 \
  ParentDecorators::swap_with(o);                                       \
}                                                                       \

#else
#define IMP_DECORATORS_METHODS(test, on_add_decorator, on_add_particle, \
                               swap)                                    \
  public:                                                               \
  const ParticlesTemp &get_particles() const;                           \
  void push_back(WrappedDecorator d);                                   \
  void push_back(Particle *p);                                          \
  WrappedDecorator back() const;                                        \
  WrappedDecorator front() const;

#endif


template <class WrappedDecorator, class ParentDecorators>
class Decorators: public ParentDecorators {
  typedef Decorators<WrappedDecorator, ParentDecorators> ThisDecorators;
  struct Proxy: public WrappedDecorator {
    typedef typename ParentDecorators::reference Ref;
    Ref d_;
    Proxy(Ref t):
      WrappedDecorator(t), d_(t){
    }
    Proxy(Ref p, bool): WrappedDecorator(), d_(p){}
    void operator=(WrappedDecorator v) {
      WrappedDecorator::operator=(v);
      d_=v;
    }
#ifdef _MSC_VER
    // for VC, it can't otherwise figure out the conversion chain
    operator Particle*() {
      if (WrappedDecorator()==*this) return NULL;
      else return WrappedDecorator::get_particle();
    }
#endif
  };
  Proxy get_proxy(unsigned int i) {
    if (ParentDecorators::operator[](i)) {
      return Proxy(ParentDecorators::operator[](i));
    } else {
      return Proxy(ParentDecorators::operator[](i), false);
    }
  }

  IMP_DECORATORS_METHODS(WrappedDecorator::particle_is_instance(p),,,);
  public:
  explicit Decorators(const Particles &ps) {
    check(ps.begin(), ps.end());
    ParentDecorators::reserve(ps.size());
    for (unsigned int i=0; i< ps.size(); ++i) {
      push_back(ps[i]);
    }
  }
  explicit Decorators(const ParticlesTemp &ds) {
    check(ds.begin(), ds.end());
    ParentDecorators::reserve(ds.size());
    for (unsigned int i=0; i< ds.size(); ++i) {
      push_back(ds[i]);
    }
  }
  explicit Decorators(unsigned int i): ParentDecorators(i, WrappedDecorator()){}
  explicit Decorators(WrappedDecorator d): ParentDecorators(1, d){}
  explicit Decorators(unsigned int n,
                      WrappedDecorator d): ParentDecorators(n, d){}
  Decorators(){}
#ifndef SWIG
  Proxy
  operator[](unsigned int i) {
    return get_proxy(i);
  }
#endif

#ifndef SWIG
  WrappedDecorator operator[](unsigned int i) const {
    return WrappedDecorator(ParentDecorators::operator[](i));
  }
#endif
};

IMP_SWAP_2(Decorators);

template <class WrappedDecorator, class ParentDecorators, class Traits>
class DecoratorsWithTraits: public ParentDecorators {
  typedef DecoratorsWithTraits<WrappedDecorator, ParentDecorators,
                               Traits> ThisDecorators;

  struct Proxy: public WrappedDecorator {
    typedef typename ParentDecorators::reference Ref;
    Ref d_;
    Proxy(Ref t, Traits tr):
      WrappedDecorator(t, tr), d_(t){
    }
    Proxy(Ref p, bool): WrappedDecorator(), d_(p){}
    void operator=(WrappedDecorator v) {
      // traits should match, but not checked
      WrappedDecorator::operator=(v);
      d_=v;
    }
#ifdef _MSC_VER
    // for VC, it can't otherwise figure out the conversion chain
    operator Particle*() {
      if (WrappedDecorator()==*this) return NULL;
      else return WrappedDecorator::get_particle();
    }
#endif
  };
  Proxy get_proxy(unsigned int i, Traits t) {
    if (ParentDecorators::operator[](i)) {
      return Proxy(ParentDecorators::operator[](i), t);
    } else {
      return Proxy(ParentDecorators::operator[](i), false);
    }
  }
  Traits tr_;
  bool has_traits_;
  IMP_DECORATORS_METHODS(WrappedDecorator::particle_is_instance(p, tr_),{
      if (!has_traits_) {
        tr_= d.get_traits();
        has_traits_=true;
      } else {
        IMP_USAGE_CHECK(tr_ == d.get_traits(),
                        "Traits don't match");
      }
    },{
      IMP_USAGE_CHECK(has_traits_, "Need to add a decorator first to get "
                      << "traits class.");
    }, {
      std::swap(tr_, o.tr_);
      std::swap(has_traits_, o.has_traits_);
    });
  public:
  explicit DecoratorsWithTraits(Traits tr): tr_(tr), has_traits_(true){}
  explicit DecoratorsWithTraits(WrappedDecorator d): ParentDecorators(1,d),
                                              tr_(d.get_traits()),
                                              has_traits_(true){}
  explicit DecoratorsWithTraits(unsigned int n, WrappedDecorator d):
    ParentDecorators(n, d),
    tr_(d.get_traits()),
    has_traits_(true) {}
  DecoratorsWithTraits(const Particles &ps,
                       Traits tr): tr_(tr), has_traits_(true) {
    ParentDecorators::resize(ps.size());
    for (unsigned int i=0; i< ps.size(); ++i) {
      ParentDecorators::operator[](i)=WrappedDecorator(ps[i], tr);
    }
  }
  DecoratorsWithTraits(unsigned int i,
                       Traits tr): ParentDecorators(i), tr_(tr),
                                   has_traits_(true){}
  DecoratorsWithTraits(): has_traits_(false){}

#ifndef SWIG
  Proxy
  operator[](unsigned int i) {
    IMP_USAGE_CHECK(has_traits_, "Can only use operator[] on a decorator "
                    << "container "
                    << "which is non-empty. This is a bug, but hard to fix.");
    return get_proxy(i, tr_);
  }
  WrappedDecorator operator[](unsigned int i) const {
    return WrappedDecorator(ParentDecorators::operator[](i), tr_);
  }
#endif
};


IMP_SWAP_3(DecoratorsWithTraits);


/** A class to add ref counting to a decorator */
template <class D>
class RefCountingDecorator: public D {
public:
  RefCountingDecorator(){}
  RefCountingDecorator(const D &d): D(d){internal::ref(D::get_particle());}
  ~RefCountingDecorator(){ internal::unref(D::get_particle());}
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
 }


#define IMP_CONSTRAINT_DECORATOR_DEF(Name)                              \
  ObjectKey Name::get_constraint_key() {                                \
    static ObjectKey ret(#Name " score state");                         \
    return ret;                                                         \
  }                                                                     \
  void Name::set_constraint(SingletonModifier* before,                  \
                            SingletonModifier *after,                   \
                            Particle *p) {                              \
    Constraint *ss= new SingletonConstraint(before,                     \
                                            after, p,                   \
                      std::string(#Name "updater for ")+p->get_name()); \
    p->add_attribute(get_constraint_key(), ss);                         \
    p->get_model()->add_score_state(ss);                                \
  }                                                                     \

#endif

IMP_END_NAMESPACE

#endif  /* IMP_DECORATOR_H */
