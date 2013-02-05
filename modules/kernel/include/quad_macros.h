/**
 *  \file IMP/kernel/quad_macros.h
 *  \brief Macros for various classes.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_QUAD_MACROS_H
#define IMPKERNEL_QUAD_MACROS_H

#include "internal/TupleRestraint.h"
#include "internal/functors.h"
#include "container_macros.h"
#include "input_output_macros.h"
#include <algorithm>


//! Declare the functions needed for a QuadScore
/** In addition to the methods done by IMP_INTERACTON, it declares
    - IMP::QuadScore::evaluate(IMP::Particle*,
    IMP::DerivativeAccumulator*)
    - IMP::QuadScore::get_input_particles()
    - IMP::QuadScore::get_output_particles()

    See IMP_SIMPLE_QUAD_SCORE() for a way of providing an
    implementation of that method.
*/
#define IMP_QUAD_SCORE(Name)                                      \
  IMP_IMPLEMENT(double evaluate(const ParticleQuad& p,\
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(double evaluate_index(Model *m,                  \
                                const ParticleIndexQuad& p,           \
                                     DerivativeAccumulator *da) const, { \
       return evaluate(IMP::kernel::internal::get_particle(m,p), da); \
                                    });                                 \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,          \
                          const ParticleIndexQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                                                     double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                       });                                              \
  IMP_BACKWARDS_MACRO_INPUTS;                                           \
  IMP_OBJECT(Name)

//! Declare the functions needed for a QuadScore
/** In addition to the methods declared and defined by IMP_QUAD_SCORE,
    the macro provides an implementation of
    - IMP::QuadScore::get_input_particles()
    - IMP::QuadScore::get_input_containers()
    which assume that only the passed particle serves as input to the
    score.
*/
#define IMP_SIMPLE_QUAD_SCORE(Name)                               \
  IMP_IMPLEMENT(double evaluate(const ParticleQuad& p,    \
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(ModelObjectsTemp                                 \
                       do_get_inputs(Model *m,                          \
                                     const ParticleIndexes &pis) const, { \
                         ModelObjectsTemp ret;                          \
                         ret+=IMP::kernel::get_particles(m, pis);       \
                         return ret;                                    \
                       });                                              \
  IMP_IMPLEMENT_INLINE(Restraints do_create_current_decomposition          \
                       (Model *m,                                       \
                        const ParticleIndexQuad& vt) const, {           \
                         return  IMP::kernel::internal            \
                           ::create_score_current_decomposition(this, m, vt); \
                       });                                        \
  IMP_OBJECT(Name)



//! Declare the functions needed for a complex QuadScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::QuadScore::evaluate_index()
    - IMP::QuadScore::do_get_inputs()
    - IMP::QuadScore::evaluate_if_good_index()
*/
#define IMP_COMPOSITE_QUAD_SCORE(Name)                            \
  IMP_IMPLEMENT_INLINE(double evaluate(const ParticleQuad& p,     \
                                       DerivativeAccumulator *da) const, { \
              return evaluate_index(IMP::kernel::internal::get_model(p), \
                               IMP::kernel::internal::get_index(p), da); \
                       });                                              \
  IMP_IMPLEMENT(double evaluate_index(Model *m, const ParticleIndexQuad& p,\
                                      DerivativeAccumulator *da) const); \
  IMP_IMPLEMENT(double evaluate_if_good_index(Model *m,                 \
                          const ParticleIndexQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                                              double max) const);       \
  IMP_IMPLEMENT_INLINE(double evaluate_indexes(Model *m,                \
                                        const ParticleIndexQuads &p,       \
                                        DerivativeAccumulator *da,      \
                                        unsigned int lower_bound,       \
                                               unsigned int upper_bound) const,\
  {                                                                     \
    double ret=0;                                                       \
    for (unsigned int i=lower_bound; i < upper_bound; ++i) {            \
      ret+= evaluate_index(m, p[i], da);                                \
    }                                                                   \
    return ret;                                                         \
  });                                                                   \
  IMP_IMPLEMENT_INLINE(double                                           \
                       evaluate_if_good_indexes(Model *m,               \
                         const ParticleIndexQuads &p,                      \
                         DerivativeAccumulator *da,                     \
                         double max,                                    \
                         unsigned int lower_bound,                      \
                         unsigned int upper_bound) const, {             \
    double ret=0;                                                       \
    for (unsigned int i=lower_bound; i < upper_bound; ++i) {            \
      ret+= evaluate_if_good_index(m, p[i], da, max-ret);               \
      if (ret>max) return std::numeric_limits<double>::max();           \
    }                                                                   \
    return ret;                                                         \
                       });                                              \
  IMP_IMPLEMENT(ModelObjectsTemp                                        \
  do_get_inputs(Model *m,                                               \
                const ParticleIndexes &pis) const );        \
   IMP_OBJECT(Name)

//! Declare the functions needed for a complex QuadScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::QuadScore::evaluate()
    - IMP::QuadScore::get_input_particles()
    - IMP::QuadScore::get_output_particles()
    - IMP::QuadScore::evaluate_if_good
*/
#define IMP_INDEX_QUAD_SCORE(Name)                                \
  IMP_IMPLEMENT_INLINE(double evaluate(const ParticleQuad& p,\
                                        DerivativeAccumulator *da) const, { \
    return evaluate_index(IMP::kernel::internal::get_model(p),    \
                  IMP::kernel::internal::get_index(p),      \
                  da);                                                  \
                        });                                             \
  IMP_IMPLEMENT(double evaluate_index(Model *m, const ParticleIndexQuad& p,\
                                      DerivativeAccumulator *da) \
                const IMP_FINAL);                         \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,         \
                          const ParticleIndexQuad& p,                      \
                          DerivativeAccumulator *da,                    \
                                                      double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                       });                                              \
  IMP_IMPLEMENT_INLINE(double                                           \
  evaluate_indexes(Model *m,                                            \
                   const ParticleIndexQuads &p,                            \
                   DerivativeAccumulator *da,                           \
                   unsigned int lower_bound,                            \
                   unsigned int upper_bound) const IMP_FINAL, \
  {                                                                     \
    double ret=0;                                                       \
    for (unsigned int i=lower_bound; i < upper_bound; ++i) {            \
      ret+= evaluate_index(m, p[i], da);                                \
    }                                                                   \
    return ret;                                                         \
  });                                                                   \
  IMP_IMPLEMENT_INLINE(double                                           \
                       evaluate_if_good_indexes(Model *m,               \
                         const ParticleIndexQuads &p,                      \
                         DerivativeAccumulator *da,                     \
                         double max,                                    \
                         unsigned int lower_bound,                      \
                         unsigned int upper_bound) const, {             \
    double ret=0;                                                       \
    for (unsigned int i=lower_bound; i < upper_bound; ++i) {            \
      ret+= evaluate_if_good_index(m, p[i], da, max-ret);               \
      if (ret>max) return std::numeric_limits<double>::max();           \
    }                                                                   \
    return ret;                                                         \
                       });                                              \
  IMP_IMPLEMENT(ModelObjectsTemp                                        \
  do_get_inputs(Model *m,                                               \
                const ParticleIndexes &pis) const);        \
  IMP_OBJECT(Name)



//! Declare the functions needed for a QuadPredicate
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::QuadPredicate::get_value()
    - IMP::QuadPredicate::get_input_particles()
    - IMP::QuadPredicate::get_output_particles()
*/
#define IMP_QUAD_PREDICATE(Name)                                   \
  IMP_IMPLEMENT(int get_value(const ParticleQuad& a) const);   \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                              ParticleQuadsTemp &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           const ParticleIndexQuad& vt)\
                       const, {                                         \
        return Name::get_value(IMP::kernel::internal::get_particle(m, vt)); \
                       });                                              \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                     const ParticleIndexQuads &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_BACKWARDS_MACRO_INPUTS;                                           \
  IMP_OBJECT(Name)


//! Declare the functions needed for a QuadPredicate
/** In addition to the methods done by IMP_OBJECT, it defines
    - IMP::QuadPredicate::get_value_index() based on the return_value
    parameter
    - IMP::QuadPredicate::do_get_inputs() based on the return_inputs
    parameter
*/
#define IMP_INDEX_QUAD_PREDICATE(Name, return_value, return_inputs) \
  IMP_IMPLEMENT_INLINE(int get_value(const ParticleQuad& a) const, {    \
    return get_value_index(IMP::kernel::internal::get_model(a),          \
                     IMP::kernel::internal::get_index(a));            \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                                      ParticleQuadsTemp &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
                       })                                               \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           const ParticleIndexQuad& pi)\
                       const, {                                         \
                         return_value;                                  \
                       })                                               \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                const ParticleIndexQuads &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_IMPLEMENT_INLINE_NO_SWIG(void remove_if_equal(Model *m,           \
                                            ParticleIndexQuads& ps,        \
                                            int value) const, {         \
      ps.erase(std::remove_if(ps.begin(), ps.end(),                     \
                IMP::kernel::internal::PredicateEquals<Name, true>(this, \
                                                              m, value)), \
               ps.end());                                               \
                       });                                              \
  IMP_IMPLEMENT_INLINE_NO_SWIG(void remove_if_not_equal(Model *m,       \
                                            ParticleIndexQuads& ps,        \
                                            int value) const, {         \
      ps.erase(std::remove_if(ps.begin(), ps.end(),                     \
              IMP::kernel::internal::PredicateEquals<Name, false>(this, \
                                                                 m, value)), \
               ps.end());                                               \
                       });                                              \
  IMP_IMPLEMENT_INLINE(ModelObjectsTemp                                 \
  do_get_inputs(Model *m,                                               \
                const ParticleIndexes &pi) const, {                     \
    return_inputs;                                                      \
                       });                                              \
  IMP_OBJECT_INLINE(Name,IMP_UNUSED(out),)


//! Declare the functions needed for a QuadModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::QuadModifier::apply(IMP::Particle*)
    - IMP::QuadModifier::get_input_particles()
    - IMP::QuadModifier::get_output_particles()
*/
#define IMP_QUAD_MODIFIER(Name)                                   \
  IMP_IMPLEMENT(void apply(const ParticleQuad& a) const); \
  IMP_IMPLEMENT_INLINE(void apply_index(Model *m, \
                                        const ParticleIndexQuad& a) const, {\
    return Name::apply(IMP::kernel::internal::get_particle(m,a));    \
    })                                                                  \
  IMP_BACKWARDS_MACRO_INPUTS;                                                 \
  IMP_BACKWARDS_MACRO_OUTPUTS;                                                \
  IMP_OBJECT(Name)

//! Use IMP_QUAD_MODIFIER() instead
#define IMP_QUAD_DERIVATIVE_MODIFIER(Name)                        \
  IMP_QUAD_MODIFIER(Name)


//! Declare the functions needed for a QuadModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::QuadModifier::apply(IMP::Particle*)
    - IMP::QuadModifier::get_inputs()
    - IMP::QuadModifier::get_outputs()
*/
#define IMP_INDEX_QUAD_MODIFIER(Name)                 \
  IMP_IMPLEMENT_INLINE(void apply(const ParticleQuad& a) const, {  \
    apply_index(IMP::kernel::internal::get_model(a),              \
                IMP::kernel::internal::get_index(a));                 \
    });                                                                 \
  IMP_IMPLEMENT(void apply_index(Model *m,                              \
                                 const ParticleIndexQuad& a)\
                const IMP_FINAL);                          \
  IMP_IMPLEMENT_INLINE(void apply_indexes(Model *m,                     \
                                          const ParticleIndexQuads &o,     \
                                          unsigned int lower_bound,     \
                                          unsigned int upper_bound)\
                       const IMP_FINAL,                    \
  {                                                                     \
    for (unsigned int i=lower_bound; i < upper_bound; ++i) {            \
      apply_index(m, o[i]);                                             \
    }                                                                   \
  });                                                                   \
  IMP_IMPLEMENT(ModelObjectsTemp do_get_inputs(Model *m,                \
                                    const ParticleIndexes &pis) const); \
  IMP_IMPLEMENT(ModelObjectsTemp do_get_outputs(Model *m,               \
                                    const ParticleIndexes &pis) const); \
  IMP_OBJECT(Name)

//! Use IMP_INDEX_QUAD_MODIFIER instead
#define IMP_INDEX_QUAD_DERIVATIVE_MODIFIER(Name)  \
  IMP_INDEX_QUAD_MODIFIER(Name)




#ifndef IMP_DOXYGEN
#define IMP_IMPLEMENT_QUAD_CONTAINER(Name)                        \
  IMP_IMPLEMENT_INLINE(void do_apply(const QuadModifier *sm) const, {\
    apply_generic(sm);                                                  \
  });                                                                   \
  IMP_IMPLEMENT(ParticleIndexes get_all_possible_indexes() const);      \
  IMP_OBJECT(Name)
#endif





//! Declare the needed functions for a QuadContainer
/** In addition to the methods of IMP_OBJECT, it declares
    - IMP::QuadContainer::get_number_of_particle_particle_quads()
    - IMP::QuadContainer::get_particle_particle_quad()
    - IMP::QuadContainer::apply()
    - IMP::QuadContainer::evaluate()
    - IMP::Interaction::get_input_objects()

    You need to define a template method with the signature
\code
template <class Functor>
Functor for_each(Functor f);
\endcode
    that applied the functor to each thing in the container.
*/
#define IMP_QUAD_CONTAINER(Name)                                  \
  IMP_IMPLEMENT(ParticleIndexQuads get_indexes() const);                   \
  IMP_IMPLEMENT(ParticleIndexQuads get_range_indexes() const);      \
  IMP_IMPLEMENT(void do_before_evaluate());                             \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles() const);             \
  IMP_IMPLEMENT(ContainersTemp get_input_containers() const);           \
  IMP_IMPLEMENT_INLINE(ModelObjectsTemp do_get_inputs() const, {        \
    ModelObjects ret;                                                   \
    ret+=get_input_containers();                                        \
    ret+=get_input_particles();                                         \
    return ret;                                                         \
  });                                                                   \
  IMP_IMPLEMENT_QUAD_CONTAINER(Name)


//! Declare the needed functions for an active QuadContainer
/** In addition to the methods of IMP_QUAD_CONTAINER(), it declares
    - IMP::ScoreState::get_input_particles()
    - IMP::ScoreState::get_input_containers()
    - IMP::ScoreState::do_before_evaluate()
*/
#define IMP_ACTIVE_QUAD_CONTAINER(Name)                           \
  IMP_QUAD_CONTAINER(name)

/** Use IMP_CONTAINER_FOREACH() instead.
*/
#define IMP_FOREACH_QUAD(sequence, operation) do {                \
    IMP::kernel::ParticleQuadsTemp imp_all=sequence->get();            \
  for (unsigned int _2=0;                                               \
       _2 != imp_all.size();                                            \
       ++_2) {                                                          \
    IMP::kernel::ParticleQuad _1= imp_all[_2];                          \
    bool imp_foreach_break=false;                                       \
    operation                                                           \
      if (imp_foreach_break) break;                                     \
  }                                                                     \
  } while (false)



/** Use IMP_CONTAINER_FOREACH() instead.
*/
#define IMP_FOREACH_QUAD_INDEX(sequence, operation)               \
  IMP_CONTAINER_FOREACH(QuadContainer, sequence, operation)

#endif  /* IMPKERNEL_QUAD_MACROS_H */
