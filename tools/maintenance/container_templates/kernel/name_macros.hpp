/**
 *  \file IMP/LCCLASSNAME_macros.h
 *  \brief Macros for various classes.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_HEADERNAME_MACROS_H
#define IMPKERNEL_HEADERNAME_MACROS_H

#include "internal/TupleRestraint.h"
#include "internal/functors.h"
#include <algorithm>


#define IMP_HEADERNAME_SCORE_BASE(Name)                                 \
  IMP_IMPLEMENT_INLINE(double evaluate_indexes(Model *m,                \
                                 const PLURALINDEXTYPE &ps, \
                                               DerivativeAccumulator *da) \
                       const, {                                         \
                         IMP::internal::ScoreAccumulator<Name> sa(m,    \
                                                                  this, \
                                                                  da);  \
                         sa(ps);                                        \
                         return sa.get_score();                         \
                       });                                              \
  IMP_IMPLEMENT_INLINE(double                                           \
                       evaluate_if_good_indexes(Model *m,               \
                                 const PLURALINDEXTYPE &ps, \
                                                DerivativeAccumulator *da, \
                                                double max) const, {    \
                         IMP::internal::ScoreAccumulatorIfGood<Name> sa(m, \
                                                                        this, \
                                                                        max, \
                                                                        da); \
                         sa(ps);                                        \
                         return sa.get_score();                         \
                       });                                              \
  IMP_OBJECT(Name)



//! Declare the functions needed for a CLASSNAMEScore
/** In addition to the methods done by IMP_INTERACTON, it declares
    - IMP::CLASSNAMEScore::evaluate(IMP::Particle*,
    IMP::DerivativeAccumulator*)
    - IMP::CLASSNAMEScore::get_input_particles()
    - IMP::CLASSNAMEScore::get_output_particles()

    See IMP_SIMPLE_HEADERNAME_SCORE() for a way of providing an
    implementation of that method.
*/
#define IMP_HEADERNAME_SCORE(Name)                                      \
  IMP_IMPLEMENT(double evaluate(ARGUMENTTYPE p,\
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(double evaluate_index(Model *m,                  \
                                PASSINDEXTYPE p,           \
                                     DerivativeAccumulator *da) const, { \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
                                    });                                 \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,          \
                          PASSINDEXTYPE p,                       \
                          DerivativeAccumulator *da,                    \
                                                     double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*p) const) ;  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *) const) ; \
  IMP_HEADERNAME_SCORE_BASE(Name)

//! Declare the functions needed for a CLASSNAMEScore
/** In addition to the methods declared and defined by IMP_HEADERNAME_SCORE,
    the macro provides an implementation of
    - IMP::CLASSNAMEScore::get_input_particles()
    - IMP::CLASSNAMEScore::get_input_containers()
    which assume that only the passed particle serves as input to the
    score.
*/
#define IMP_SIMPLE_HEADERNAME_SCORE(Name)                               \
  IMP_IMPLEMENT(double evaluate(ARGUMENTTYPE p,    \
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(double evaluate(Model *m,                        \
                                  PASSINDEXTYPE p,  \
                                       DerivativeAccumulator *da) const, { \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
                       });                                              \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,          \
                          PASSINDEXTYPE p,                       \
                          DerivativeAccumulator *da,                    \
                                                     double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                       });                                              \
  IMP_IMPLEMENT_INLINE(ParticlesTemp get_input_particles(Particle*p) const, { \
    return ParticlesTemp(1,p);                                          \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(ContainersTemp get_input_containers(Particle *) const, \
  {                                                                     \
    return ContainersTemp();                                            \
  });                                                                   \
  IMP_IMPLEMENT_INLINE(Restraints create_current_decomposition\
  (ARGUMENTTYPE vt) const, {                                      \
 return  IMP::internal::create_score_current_decomposition(this, vt); \
                       });                                        \
  IMP_HEADERNAME_SCORE_BASE(Name)



//! Declare the functions needed for a complex CLASSNAMEScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::CLASSNAMEScore::evaluate()
    - IMP::CLASSNAMEScore::get_input_particles()
    - IMP::CLASSNAMEScore::get_output_particles()
    - IMP::CLASSNAMEScore::evaluate_if_good
*/
#define IMP_COMPOSITE_HEADERNAME_SCORE(Name)                            \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle *p) const);  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *p) const); \
  IMP_IMPLEMENT_INLINE(double evaluate(ARGUMENTTYPE p,     \
                                       DerivativeAccumulator *da) const, { \
    return evaluate_index(IMP::internal::get_model(p),                  \
                  IMP::internal::get_index(p), da);                     \
                       });                                              \
  IMP_IMPLEMENT(double evaluate_index(Model *m, PASSINDEXTYPE p,\
                                      DerivativeAccumulator *da) const); \
  IMP_IMPLEMENT(double evaluate_if_good_index(Model *m,                 \
                          PASSINDEXTYPE p,                       \
                          DerivativeAccumulator *da,                    \
                                              double max) const);       \
  IMP_HEADERNAME_SCORE_BASE(Name)

//! Declare the functions needed for a complex CLASSNAMEScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::CLASSNAMEScore::evaluate()
    - IMP::CLASSNAMEScore::get_input_particles()
    - IMP::CLASSNAMEScore::get_output_particles()
    - IMP::CLASSNAMEScore::evaluate_if_good
*/
#define IMP_INDEX_HEADERNAME_SCORE(Name)                                \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle *p) const);  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *p) const); \
  IMP_IMPLEMENT_INLINE(double evaluate(ARGUMENTTYPE p,\
                                        DerivativeAccumulator *da) const, { \
    return evaluate_index(IMP::internal::get_model(p),                  \
                  IMP::internal::get_index(p),                          \
                  da);                                                  \
                        });                                             \
  IMP_IMPLEMENT(double evaluate_index(Model *m, PASSINDEXTYPE p,\
                                      DerivativeAccumulator *da) const); \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,         \
                          PASSINDEXTYPE p,                      \
                          DerivativeAccumulator *da,                    \
                                                      double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                        });                                             \
  IMP_HEADERNAME_SCORE_BASE(Name)



//! Declare the functions needed for a CLASSNAMEPredicate
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEPredicate::get_value()
    - IMP::CLASSNAMEPredicate::get_input_particles()
    - IMP::CLASSNAMEPredicate::get_output_particles()
*/
#define IMP_HEADERNAME_PREDICATE(Name)                                   \
  IMP_IMPLEMENT(int get_value(ARGUMENTTYPE a) const);   \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                              PLURALVARIABLETYPE &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           PASSINDEXTYPE vt)\
                       const, {                                         \
        return Name::get_value(IMP::internal::get_particle(m, vt)); \
                       });                                              \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                     const PLURALINDEXTYPE &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_OBJECT(Name)


//! Declare the functions needed for a CLASSNAMEPredicate
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEPredicate::get_value_index()
    - IMP::CLASSNAMEPredicate::get_input_particles()
    - IMP::CLASSNAMEPredicate::get_output_particles()
*/
#define IMP_INDEX_HEADERNAME_PREDICATE(Name, gv)                        \
  IMP_IMPLEMENT_INLINE(int get_value(ARGUMENTTYPE a) const, { \
    return get_value_index(IMP::internal::get_model(a),                 \
                     IMP::internal::get_index(a));                      \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                                      PLURALVARIABLETYPE &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
                       })                                               \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           PASSINDEXTYPE pi)\
                       const, {                                         \
    gv;                                                                 \
                       })                                               \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                const PLURALINDEXTYPE &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_IMPLEMENT_INLINE_NO_SWIG(void remove_if_equal(Model *m,           \
                                            PLURALINDEXTYPE& ps,        \
                                            int value) const, {         \
      ps.erase(std::remove_if(ps.begin(), ps.end(),                     \
                              IMP::internal::PredicateEquals<Name, true>(this, \
                                                              m, value)), \
               ps.end());                                               \
                       });                                              \
  IMP_IMPLEMENT_INLINE_NO_SWIG(void remove_if_not_equal(Model *m,       \
                                            PLURALINDEXTYPE& ps,        \
                                            int value) const, {         \
      ps.erase(std::remove_if(ps.begin(), ps.end(),                     \
                          IMP::internal::PredicateEquals<Name, false>(this, \
                                                                 m, value)), \
               ps.end());                                               \
                       });                                              \
  IMP_IMPLEMENT_INLINE(ParticlesTemp get_input_particles(Particle*p) const, { \
   return ParticlesTemp(1, p);                                          \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(ContainersTemp get_input_containers(Particle*) const, { \
   return ContainersTemp();                                             \
    });                                                                 \
 IMP_OBJECT_INLINE(Name,IMP_UNUSED(out),)


//! Declare the functions needed for a CLASSNAMEModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEModifier::apply(IMP::Particle*)
    - IMP::CLASSNAMEModifier::get_input_particles()
    - IMP::CLASSNAMEModifier::get_output_particles()
*/
#define IMP_HEADERNAME_MODIFIER(Name)                                   \
  IMP_IMPLEMENT(void apply(ARGUMENTTYPE a) const); \
  IMP_IMPLEMENT_INLINE(void apply_index(Model *m, \
                                        PASSINDEXTYPE a) const, {\
    return Name::apply(IMP::internal::get_particle(m,a));               \
    })                                                                  \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)

//! Declare the functions needed for a CLASSNAMEModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEDerivativeModifier::apply()
    - IMP::CLASSNAMEDerivativeModifier::get_input_particles()
    - IMP::CLASSNAMEDerivativeModifier::get_output_particles()
*/
#define IMP_HEADERNAME_DERIVATIVE_MODIFIER(Name)                        \
  IMP_IMPLEMENT(void apply(ARGUMENTTYPE a,\
                           DerivativeAccumulator&da) const);            \
  IMP_IMPLEMENT_INLINE(void apply_index(Model *m,                       \
                                        PASSINDEXTYPE a,  \
                                        DerivativeAccumulator&da) const, { \
    return Name::apply(IMP::internal::get_particle(m,a), da);           \
                       });                                              \
  IMP_IMPLEMENT_INLINE(void apply_indexes(Model *m,\
                                          const PLURALINDEXTYPE &ps,    \
                                          DerivativeAccumulator&da) const, { \
                         IMP::internal::ModifierDerivativeApplier<Name>(m, \
                                                                        this,\
                                                                        da) \
                           (ps);                                        \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)



//! Declare the functions needed for a CLASSNAMEModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEModifier::apply(IMP::Particle*)
    - IMP::CLASSNAMEModifier::get_input_particles()
    - IMP::CLASSNAMEModifier::get_output_particles()
*/
#define IMP_INDEX_HEADERNAME_MODIFIER(Name)                     \
  IMP_IMPLEMENT_INLINE(void apply(ARGUMENTTYPE a) const, {  \
    apply_index(IMP::internal::get_model(a),                            \
                IMP::internal::get_index(a));                           \
    });                                                                 \
  IMP_IMPLEMENT(void apply_index(Model *m,\
                                 PASSINDEXTYPE a) const);   \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)

//! Declare the functions needed for a CLASSNAMEModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEDerivativeModifier::apply()
    - IMP::CLASSNAMEDerivativeModifier::get_input_particles()
    - IMP::CLASSNAMEDerivativeModifier::get_output_particles()
*/
#define IMP_INDEX_HEADERNAME_DERIVATIVE_MODIFIER(Name)                        \
  IMP_IMPLEMENT_INLINE(void apply(ARGUMENTTYPE a,\
                                  DerivativeAccumulator&da) const, {    \
    apply_index(IMP::internal::get_model(a),                            \
                IMP::internal::get_index(a), da);                       \
                       });                                              \
  IMP_IMPLEMENT(void apply_index(Model *m, PASSINDEXTYPE a,\
                                 DerivativeAccumulator&da) const);      \
  IMP_IMPLEMENT_INLINE(void apply_indexes(Model *m,\
                                          const PLURALINDEXTYPE &ps,    \
                                          DerivativeAccumulator&da) const, { \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply_index(m, ps[i], da);                                  \
    }                                                                   \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)




#ifndef IMP_DOXYGEN
#define IMP_IMPLEMENT_HEADERNAME_CONTAINER(Name)                        \
  void apply(const CLASSNAMEModifier *sm) const {                       \
    for_each(IMP::internal::ModifierApplier<CLASSNAMEModifier>          \
             (get_model(), sm));                                        \
  }                                                                     \
  void apply(const CLASSNAMEDerivativeModifier *sm,                     \
             DerivativeAccumulator &da) const {                         \
    for_each(IMP::internal                                              \
             ::ModifierDerivativeApplier<CLASSNAMEDerivativeModifier>   \
             (get_model(), sm, da));                                    \
  }                                                                     \
  double evaluate(const CLASSNAMEScore *s,                              \
                  DerivativeAccumulator *da) const {                    \
    return for_each(IMP::internal::ScoreAccumulator<CLASSNAMEScore>     \
                    (get_model(), s, da)).get_score();                  \
  }                                                                     \
  double evaluate_if_good(const CLASSNAMEScore *s,                      \
                          DerivativeAccumulator *da, double max) const { \
    return for_each(IMP::internal::ScoreAccumulatorIfGood<CLASSNAMEScore> \
                    (get_model(), s, max, da)).get_score();             \
  }                                                                     \
  ParticlesTemp get_all_possible_particles() const;                     \
  IMP_OBJECT(Name)
#endif





//! Declare the needed functions for a CLASSNAMEContainer
/** In addition to the methods of IMP_OBJECT, it declares
    - IMP::CLASSNAMEContainer::get_contains_particle_FUNCTIONNAME()
    - IMP::CLASSNAMEContainer::get_number_of_particle_FUNCTIONNAMEs()
    - IMP::CLASSNAMEContainer::get_particle_FUNCTIONNAME()
    - IMP::CLASSNAMEContainer::apply()
    - IMP::CLASSNAMEContainer::evaluate()
    - IMP::Interaction::get_input_objects()

    You need to define a template method with the signature
\code
template <class Functor>
Functor for_each(Functor f);
\endcode
    that applied the functor to each thing in the container.
*/
#define IMP_HEADERNAME_CONTAINER(Name)                                  \
  IMP_IMPLEMENT(bool get_is_changed() const);                           \
  IMP_IMPLEMENT(bool get_contains_FUNCTIONNAME(ARGUMENTTYPE p)   \
                const);                                                 \
  IMP_IMPLEMENT(PLURALINDEXTYPE get_indexes() const);                   \
  IMP_IMPLEMENT(PLURALINDEXTYPE get_all_possible_indexes() const);      \
  IMP_IMPLEMENT(void do_before_evaluate());                             \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles() const);             \
  IMP_IMPLEMENT(ContainersTemp get_input_containers() const);           \
  IMP_IMPLEMENT_HEADERNAME_CONTAINER(Name)


//! Declare the needed functions for an active CLASSNAMEContainer
/** In addition to the methods of IMP_HEADERNAME_CONTAINER(), it declares
    - IMP::ScoreState::get_input_particles()
    - IMP::ScoreState::get_input_containers()
    - IMP::ScoreState::do_before_evaluate()
*/
#define IMP_ACTIVE_HEADERNAME_CONTAINER(Name)                           \
  IMP_HEADERNAME_CONTAINER(name)

/** These macros avoid various inefficiencies.

    The macros take the name of the sequence and the operation to
    peform. The item in the sequence is called _1, it's index is _2.
    Use it like
    \code
    IMP_FOREACH_PARTICLE(sc, std::cout << "Item " << _2
    << " is " << _1->get_name() << std::endl);
    \endcode
*/
#define IMP_FOREACH_HEADERNAME(sequence, operation) do {                \
    IMP::PLURALVARIABLETYPE imp_all=sequence->get();   \
  for (unsigned int _2=0;                                               \
       _2 != imp_all.size();                                            \
       ++_2) {                                                          \
    IMP::VARIABLETYPE _1= imp_all[_2];               \
    bool imp_foreach_break=false;                                       \
    operation                                                           \
      if (imp_foreach_break) break;                                     \
  }                                                                     \
  } while (false)



/** These macros avoid various inefficiencies.

    The macros take the name of the sequence and the operation to
    peform. The item in the sequence is called _1, it's index is _2.
    Use it like
    \code
    IMP_FOREACH_PARTICLE(sc, std::cout << "Item " << _2
                         << " is _1->get_name() << std::endl);
    \endcode
*/
#define IMP_FOREACH_HEADERNAME_INDEX(sequence, operation)               \
  do {                                                                  \
    if (sequence->get_provides_access()) {                              \
      const PLURALINDEXTYPE &imp_foreach_access                         \
        =sequence->get_access();                                        \
      for (unsigned int _2=0; _2< imp_foreach_access.size(); ++_2) {    \
        IMP::INDEXTYPE _1= imp_foreach_access[_2];          \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) { break;}                              \
      }                                                                 \
    } else {                                                            \
      PLURALINDEXTYPE imp_foreach_indexes              \
        =sequence->get_indexes();                                       \
      for (unsigned int _2=0;                                           \
           _2 != imp_foreach_indexes.size();                            \
           ++_2) {                                                      \
        IMP::INDEXTYPE _1= imp_foreach_indexes[_2];            \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) break;                                 \
      }                                                                 \
    }                                                                   \
  } while (false)

#endif  /* IMPKERNEL_HEADERNAME_MACROS_H */
