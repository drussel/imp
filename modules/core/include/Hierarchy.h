/**
 *  \file core/Hierarchy.h     \brief Decorator for helping deal with
 *                                        a hierarchy.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_HIERARCHY_H
#define IMPCORE_HIERARCHY_H

#include "config.h"
#include "utility.h"
#include "internal/hierarchy_helpers.h"
#include "internal/ArrayOnAttributesHelper.h"

#include <IMP/SingletonModifier.h>
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/Decorator.h>
#include <IMP/internal/PrefixStream.h>

#include <limits>
#include <vector>
#include <deque>

IMPCORE_BEGIN_NAMESPACE

/** \defgroup hierarchy Hierarchies of particles
    These functions and classes aid in manipulating particles representing
    molecules at multiple levels.
 */
#ifndef SWIG
class Hierarchy;
#endif

//! Define the type for a type of hierarchy
/** The hierarchy class is identified by the passed string so two
    hierarchies created with the same initialization string will be
    the same.

    This example shows how to make and use a custom hierarchy:
    \verbinclude custom_hierarchy.py
    \see Hierarchy
*/
class IMPCOREEXPORT HierarchyTraits
#ifndef SWIG
: public internal::ArrayOnAttributesHelper<ParticleKey, Particle*>
#endif
{
  friend class Hierarchy;
  typedef internal::ArrayOnAttributesHelper<ParticleKey, Particle*> P;

  ParticleKey parent_key_;
  IntKey parent_index_key_;

  template <class HD>
  void on_add(Particle * p, HD d, unsigned int i) {
    d.get_particle()->add_attribute(parent_key_, p);
    d.get_particle()->add_attribute(parent_index_key_, i);
  }
  void on_change(Particle *, Particle* p, unsigned int oi,
                        unsigned int ni) {
    p->set_value(parent_index_key_, ni);
  }
  template <class HD>
  void on_remove(Particle *, HD d) {
    d.get_particle()->remove_attribute(parent_index_key_);
    d.get_particle()->remove_attribute(parent_key_);
  }
  template <class HD>
  Particle *get_value(HD d) const {
    return d.get_particle();
  }
  template <class HD>
  unsigned int get_index(Particle *, HD d) {
    return d.get_parent_index();
  }
  // otherwise it is masked
  using P::get_value;

  template <class T>
  void audit_value(T t) const {
    IMP_USAGE_CHECK(t.get_traits().get_name() == get_name(),
              "Mixing hierarchies of type " << get_name()
              << " and type " << t.get_traits().get_name(),
              ValueException);
  }

  const Hierarchy wrap(Particle* p) const;

public:
  HierarchyTraits(){}
  //! Create a HierarchyTraits with the given name
  HierarchyTraits(std::string name);
  //! Get the name used to identify this traits.
  std::string get_name() const {
    return get_prefix();
  }

  bool operator==(const HierarchyTraits &o) const {
    return get_name() == o.get_name();
  }
};


class Hierarchy;
typedef DecoratorsWithTraits<Hierarchy, Particles,
                             HierarchyTraits> GenericHierarchies;


//! A decorator for helping deal with a hierarchy.
/**
    See HierarchyTraits for an example of how to define a custom hierarchy
    and Hierarchy for a hierarchy for molecules.
    \ingroup hierarchy
    \see HierarchyTraits
 */
class IMPCOREEXPORT Hierarchy: public Decorator
{
  typedef Decorator P;

  IMP_DECORATOR_ARRAY_DECL(public, Hierarchy, Child, child, children,
                           traits_, Hierarchy, GenericHierarchies)
public:
  IMP_DECORATOR_TRAITS(Hierarchy, Decorator,
                       HierarchyTraits, traits,
                       get_default_traits());


  //! Add the needed attributes to a particle
  static Hierarchy setup_particle(Particle *p,
                          HierarchyTraits traits
                          =Hierarchy::get_default_traits()) {
    add_required_attributes_for_child(p, traits);
    return Hierarchy(p, traits);
  }

  //! Add the needed attributes to a particle and add the particles as children
  /** The particles can be, but don't have to be Hierarchy particles
      already.
  */
  static Hierarchy setup_particle(Particle *p,
                          const Particles &children,
                          HierarchyTraits traits
                          =Hierarchy::get_default_traits()) {
    add_required_attributes_for_child(p, traits);
    Hierarchy h(p, traits);
    for (unsigned int i=0; i< children.size(); ++i) {
      if (!Hierarchy::particle_is_instance(children[i], traits)) {
        add_required_attributes_for_child(children[i], traits);
      }
      Hierarchy c(children[i], traits);
      h.add_child(c);
    }
    return h;
  }

  /** Check if the particle has the needed attributes for a
   cast to succeed */
  static bool particle_is_instance(Particle *p,
                             HierarchyTraits traits
                             =Hierarchy::get_default_traits()){
    return has_required_attributes_for_child(p, traits);
  }
#if 0
  /** Return the particles of the children
   */
  DecoratorsWithTraits<Hierarchy, Particles,
    HierarchyTraits> get_children() const {
    DecoratorsWithTraits<Hierarchy, Particles,
      HierarchyTraits> ps(get_number_of_children(), get_traits());
    for (unsigned int i=0; i< get_number_of_children(); ++i) {
      ps.set(i, get_child(i));
    }
    return ps;
  }
#endif

  /** \return the parent particle, or Hierarchy()
      if it has no parent.
   */
  Hierarchy get_parent() const {
    IMP_DECORATOR_GET(traits_.parent_key_, Particle*,
                      return This(VALUE, traits_),
                      return This());
  }

  //! Get the index of this particle in the list of children
  /** \return index in the list of children of the parent, or -1 if
      it does not have a parent.
   */
  int get_parent_index() const {
    IMP_DECORATOR_GET(traits_.parent_index_key_,
                      Int, return VALUE, return -1);
  }

  /** Return true if the parent is not empty */
  bool has_parent() const {
    return get_particle()->has_attribute(traits_.parent_key_);
  }

  //! Get the index of a specific child in this particle.
  /** This takes linear time.
      \note This is mostly useful for debugging as you can always call
      get_parent_index() on the child.
      \return the index, or -1 if there is no such child.
   */
  int get_child_index(Hierarchy c) const;

  IMP_NO_DOXYGEN(void validate_node() const;)

  IMP_NO_DOXYGEN(void validate() const;)

  //! Get the default hierarchy traits
  static const HierarchyTraits& get_default_traits();
};


IMP_OUTPUT_OPERATOR(Hierarchy);



//! A visitor for traversal of a hierarchy
/** This works from both C++ and Python
    \ingroup hierarchy
    \ingroup decorators
    \see Hierarchy
 */
class IMPCOREEXPORT HierarchyVisitor
{
public:
  HierarchyVisitor() {}
  //! Return true if the traversal should visit this node's children
  /** The const is needed to make memory management simple. Just declare
      internal data mutable.
   */
  virtual bool visit(Hierarchy p) = 0;
  virtual ~HierarchyVisitor() {}
};



//! A which applies a singleton modifier to each Particle in a hierarchy
/** This works from both C++ and Python
    \ingroup hierarchy
    \ingroup decorators
    \see SingletonModifier
    \see Hierarchy
 */
class IMPCOREEXPORT ModifierVisitor: public HierarchyVisitor
{
  Pointer<SingletonModifier> sm_;
public:
  ModifierVisitor(SingletonModifier *sm): sm_(sm) {}
  virtual bool visit(Hierarchy p) {
    sm_->apply(p.get_particle());
    return true;
  }
  virtual ~ModifierVisitor() {}
};



#if !defined (SWIG) && !defined(IMP_DOXYGEN)
namespace internal {
template <class F, class Out>
struct Gather: public HierarchyVisitor
{
  //! initialize with the function and the container
  Gather(F f, Out out): f_(f), out_(out) {}
  bool visit(Hierarchy p) {
    if (f_(p)) {
      *out_=p;
      ++out_;
    }
    return true;
  }
  //! Return the container
  Out get_out() const {
    return out_;
  }
private:
  F f_;
  Out out_;
};

}
#endif

inline const Hierarchy HierarchyTraits::wrap(Particle* p) const {
  return Hierarchy(p, *this);
}








//! Apply the visitor to each particle,  breadth first.
/** \param[in] d The Hierarchy for the tree in question
    \param[in] v The visitor to be applied. This is passed by reference.
    \ingroup hierarchy
    \relatesalso Hierarchy
 */
IMPCOREEXPORT
void breadth_first_traversal(Hierarchy d,  HierarchyVisitor &v);

//! Depth first traversal of the hierarchy
/** See breadth_first_traversal and HierarchyVisitor for more information
    \ingroup hierarchy
    \relatesalso Hierarchy
 */
IMPCOREEXPORT
void depth_first_traversal(Hierarchy d,  HierarchyVisitor &v);


//! Apply functor F to each particle, traversing the hierarchy breadth first.
/** This method allows data to be associated with each visited node.
    The data of the parent is passed to each invocation of the child.

    \param[in] d The Hierarchy for the tree in question
    \param[in] f The functor to be applied
    F must define a type Data which is returned by each call.
    The result of the parent call is passed as the second argument
    to the operator() of the child. e.g.
    struct DepthVisitor {
      typedef int result_type;
      result_type operator()(Particle *p, int i) const
      {
        if (p==NULL) return 0;
        else return i+1;
      }
    };
    \param[in] i The data to be used for d (since it has no relevant parent)

    \return A copy of the functor passed in. Use this if you care about
           the functor state.

    \ingroup hierarchy
    \relatesalso Hierarchy
 */
template <class HD, class F>
F breadth_first_traversal_with_data(HD d, F f, typename F::result_type i)
{
  typedef std::pair<typename F::result_type, HD> DP;
  std::deque<DP > stack;
  stack.push_back(DP(i, d));
  //d.show(std::cerr);
  do {
    DP cur= stack.front();
    stack.pop_front();
    typename F::result_type r= f(cur.second.get_particle(), cur.first);
    //std::cerr << "Visiting particle " << cur.get_particle() << std::endl;
    for (int i=cur.second.get_number_of_children()-1; i>=0; --i) {
      stack.push_back(std::make_pair(r, cur.second.get_child(i)));
    }
  } while (!stack.empty());
  return f;
}


//! Apply functor F to each particle, traversing the hierarchy depth first.
/** See breadth_first_traversal for documentation.
    \ingroup hierarchy
    \relatesalso Hierarchy
 */
template <class HD, class F>
F depth_first_traversal_with_data(HD d,  F f, typename F::result_type i)
{
  typedef std::pair<typename F::result_type, HD> DP;
  std::vector<DP> stack;
  stack.push_back(DP(i, d));
  //d.show(std::cerr);
  do {
    DP cur= stack.back();
    stack.pop_back();
    typename F::result_type r=f(cur.second, cur.first);
    //std::cerr << "Visiting particle " << cur.get_particle() << std::endl;
    for (int i=cur.second.get_number_of_children()-1; i>=0; --i) {
      stack.push_back(DP(r, cur.second.get_child(i)));
    }
  } while (!stack.empty());
  return f;
}




//! A simple visitor which pretty-prints the hierarchy
/** The template argument NP is the decorator to use to print each node.
    \ingroup hierarchy
    \see Hierarchy
 */
template <class PD>
struct HierarchyPrinter
{
private:
  struct RefCountedStream: public RefCounted,
                           public IMP::internal::PrefixStream{
    RefCountedStream(std::ostream *out): IMP::internal::PrefixStream(out){}
  };
  mutable Pointer<RefCountedStream> out_;
public:
  HierarchyPrinter(std::ostream &out,
                   unsigned int max_depth):
    out_(new RefCountedStream(&out)),
    md_(max_depth)
  {}

  typedef unsigned int result_type;
  int operator()(Hierarchy hd, unsigned int depth) const {
    if (depth > md_) return depth+1;
    std::string prefix;
    for (unsigned int i=0; i< depth; ++i) {
      prefix+=" ";
    }
    out_->set_prefix(prefix);
    if (hd == Hierarchy() || hd.get_number_of_children()==0) {
      *out_ << "-";
    } else {
      *out_ << "+";
    }
    *out_ << "Particle " << hd.get_particle()->get_name() << std::endl;
    out_->set_prefix(prefix+" ");
    PD nd= PD::decorate_particle(hd.get_particle());
    if (nd != PD()) {
      nd.show(*out_);
    } else {
      *out_ << "*******";
    }
    *out_ << std::endl;
    return depth+1;
  }
  unsigned int md_;
};


//! Print the hierarchy using a given decorator as to display each node
/** The last argument limits how deep will be printed out.
    \ingroup hierarchy
    \relatesalso Hierarchy
 */
template <class ND>
std::ostream &show(Hierarchy h, std::ostream &out=std::cout,
                   unsigned int max_depth
                   = std::numeric_limits<unsigned int>::max())
{
  depth_first_traversal_with_data(h, HierarchyPrinter<ND>(out, max_depth), 0);
  return out;
}


//! A simple functor to count the number of particles in a hierarchy.
/** This is a good example of a simple HierarchyVisitor.
    \ingroup hierarchy
    \see Hierarchy
 */
struct HierarchyCounter: public HierarchyVisitor
{
  HierarchyCounter(): ct_(0) {}

  //! Increment the counter
  bool visit(Hierarchy) {
    ++ct_;
    return true;
  }
  //! Return how many nodes have been visited
  unsigned int get_count() const {
    return ct_;
  }
private:

  unsigned int ct_;
};

//! Gather all the particles in the hierarchy which meet some criteria
/** \ingroup hierarchy
    \relatesalso Hierarchy
 */
template <class Out, class F>
Out gather(Hierarchy h, F f, Out out)
{
  internal::Gather<F,Out> gather(f,out);
  depth_first_traversal(h, gather);
  return gather.get_out();
}

//! Gather all the particles in the hierarchy which match on an attribute
/** \ingroup hierarchy
    \relatesalso Hierarchy
 */
template <class Out, class K, class V>
Out gather_by_attribute(Hierarchy h, K k, V v, Out out)
{
  internal::Gather<internal::MatchAttribute<K, V>,Out>
    gather(internal::MatchAttribute<K,V>(k,v),
           out);
  depth_first_traversal(h, gather);
  return gather.get_out();
}




//! Gather all the particles in the hierarchy which match on two attributes
/** \ingroup hierarchy
    \relatesalso Hierarchy
 */
template <class Out, class K0, class V0, class K1, class V1>
Out gather_by_attributes(Hierarchy h, K0 k0,
                                   V0 v0, K1 k1, V1 v1, Out out)
{
  internal::Gather<internal::MatchAttributes<K0, V0, K1, V1>,Out>
    gather(internal::MatchAttributes<K0,V0, K1, V1>(k0,v0, k1, v1),
           out);
  depth_first_traversal(h, gather);
  return gather.get_out();
}


//! Find the first node which matches some criteria
/** \ingroup hierarchy
    \relatesalso Hierarchy
 */
template <class HD, class F>
HD breadth_first_find(HD h, F f)
{
  if (f(h.get_particle())) return h;
  std::vector<HD> stack;
  stack.push_back(h);
  //d.show(std::cerr);
  do {
    HD cur= stack.back();
    stack.pop_back();

    for (int i=cur.get_number_of_children()-1; i>=0; --i) {
      HD hd= cur.get_child(i);
      if (f(hd.get_particle())) {
        return hd;
      } else {
        stack.push_back(hd);
      }
    }
  } while (!stack.empty());
  return HD();
}



//! Get all the leaves of the bit of hierarchy
/**     \relatesalso Hierarchy
 */
IMPCOREEXPORT GenericHierarchies
get_leaves(Hierarchy mhd);

//! Get all the particles in the subtree
/**     \relatesalso Hierarchy
 */
IMPCOREEXPORT GenericHierarchies
get_all_descendants(Hierarchy mhd);

//! Return the root of the hierarchy
/** \relatesalso Hierarchy */
inline Hierarchy root(Hierarchy h) {
  while (h.has_parent()) {
    h= h.get_parent();
  }
  return h;
}

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_HIERARCHY_H */
