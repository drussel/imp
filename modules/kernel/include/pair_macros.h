/**
 *  \file IMP/pair_macros.h
 *  \brief Macros for various classes.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_PAIR_MACROS_H
#define IMPKERNEL_PAIR_MACROS_H

#include "internal/TupleRestraint.h"
#include "internal/functors.h"
#include <algorithm>


#define IMP_PAIR_SCORE_BASE(Name)                                 \
  IMP_IMPLEMENT_INLINE(double evaluate_indexes(Model *m,                \
                                 const ParticleIndexPairs &ps, \
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
                                 const ParticleIndexPairs &ps, \
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



//! Declare the functions needed for a PairScore
/** In addition to the methods done by IMP_INTERACTON, it declares
    - IMP::PairScore::evaluate(IMP::Particle*,
    IMP::DerivativeAccumulator*)
    - IMP::PairScore::get_input_particles()
    - IMP::PairScore::get_output_particles()

    See IMP_SIMPLE_PAIR_SCORE() for a way of providing an
    implementation of that method.
*/
#define IMP_PAIR_SCORE(Name)                                      \
  IMP_IMPLEMENT(double evaluate(const ParticlePair& p,\
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(double evaluate_index(Model *m,                  \
                                const ParticleIndexPair& p,           \
                                     DerivativeAccumulator *da) const, { \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
                                    });                                 \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,          \
                          const ParticleIndexPair& p,                       \
                          DerivativeAccumulator *da,                    \
                                                     double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*p) const) ;  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *) const) ; \
  IMP_PAIR_SCORE_BASE(Name)

//! Declare the functions needed for a PairScore
/** In addition to the methods declared and defined by IMP_PAIR_SCORE,
    the macro provides an implementation of
    - IMP::PairScore::get_input_particles()
    - IMP::PairScore::get_input_containers()
    which assume that only the passed particle serves as input to the
    score.
*/
#define IMP_SIMPLE_PAIR_SCORE(Name)                               \
  IMP_IMPLEMENT(double evaluate(const ParticlePair& p,    \
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(double evaluate(Model *m,                        \
                                  const ParticleIndexPair& p,  \
                                       DerivativeAccumulator *da) const, { \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
                       });                                              \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,          \
                          const ParticleIndexPair& p,                       \
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
  (const ParticlePair& vt) const, {                                      \
 return  IMP::internal::create_score_current_decomposition(this, vt); \
                       });                                        \
  IMP_PAIR_SCORE_BASE(Name)



//! Declare the functions needed for a complex PairScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::PairScore::evaluate()
    - IMP::PairScore::get_input_particles()
    - IMP::PairScore::get_output_particles()
    - IMP::PairScore::evaluate_if_good
*/
#define IMP_COMPOSITE_PAIR_SCORE(Name)                            \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle *p) const);  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *p) const); \
  IMP_IMPLEMENT_INLINE(double evaluate(const ParticlePair& p,     \
                                       DerivativeAccumulator *da) const, { \
    return evaluate_index(IMP::internal::get_model(p),                  \
                  IMP::internal::get_index(p), da);                     \
                       });                                              \
  IMP_IMPLEMENT(double evaluate_index(Model *m, const ParticleIndexPair& p,\
                                      DerivativeAccumulator *da) const); \
  IMP_IMPLEMENT(double evaluate_if_good_index(Model *m,                 \
                          const ParticleIndexPair& p,                       \
                          DerivativeAccumulator *da,                    \
                                              double max) const);       \
  IMP_PAIR_SCORE_BASE(Name)

//! Declare the functions needed for a complex PairScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::PairScore::evaluate()
    - IMP::PairScore::get_input_particles()
    - IMP::PairScore::get_output_particles()
    - IMP::PairScore::evaluate_if_good
*/
#define IMP_INDEX_PAIR_SCORE(Name)                                \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle *p) const);  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *p) const); \
  IMP_IMPLEMENT_INLINE(double evaluate(const ParticlePair& p,\
                                        DerivativeAccumulator *da) const, { \
    return evaluate_index(IMP::internal::get_model(p),                  \
                  IMP::internal::get_index(p),                          \
                  da);                                                  \
                        });                                             \
  IMP_IMPLEMENT(double evaluate_index(Model *m, const ParticleIndexPair& p,\
                                      DerivativeAccumulator *da) const); \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,         \
                          const ParticleIndexPair& p,                      \
                          DerivativeAccumulator *da,                    \
                                                      double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                        });                                             \
  IMP_PAIR_SCORE_BASE(Name)



//! Declare the functions needed for a PairPredicate
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::PairPredicate::get_value()
    - IMP::PairPredicate::get_input_particles()
    - IMP::PairPredicate::get_output_particles()
*/
#define IMP_PAIR_PREDICATE(Name)                                   \
  IMP_IMPLEMENT(int get_value(const ParticlePair& a) const);   \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                              ParticlePairsTemp &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           const ParticleIndexPair& vt)\
                       const, {                                         \
        return Name::get_value(IMP::internal::get_particle(m, vt)); \
                       });                                              \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                     const ParticleIndexPairs &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_OBJECT(Name)


//! Declare the functions needed for a PairPredicate
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::PairPredicate::get_value_index()
    - IMP::PairPredicate::get_input_particles()
    - IMP::PairPredicate::get_output_particles()
*/
#define IMP_INDEX_PAIR_PREDICATE(Name, gv)                        \
  IMP_IMPLEMENT_INLINE(int get_value(const ParticlePair& a) const, { \
    return get_value_index(IMP::internal::get_model(a),                 \
                     IMP::internal::get_index(a));                      \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                                      ParticlePairsTemp &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
                       })                                               \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           const ParticleIndexPair& pi)\
                       const, {                                         \
    gv;                                                                 \
                       })                                               \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                const ParticleIndexPairs &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_IMPLEMENT_INLINE_NO_SWIG(void remove_if_equal(Model *m,           \
                                            ParticleIndexPairs& ps,        \
                                            int value) const, {         \
      ps.erase(std::remove_if(ps.begin(), ps.end(),                     \
                              IMP::internal::PredicateEquals<Name, true>(this, \
                                                              m, value)), \
               ps.end());                                               \
                       });                                              \
  IMP_IMPLEMENT_INLINE_NO_SWIG(void remove_if_not_equal(Model *m,       \
                                            ParticleIndexPairs& ps,        \
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


//! Declare the functions needed for a PairModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::PairModifier::apply(IMP::Particle*)
    - IMP::PairModifier::get_input_particles()
    - IMP::PairModifier::get_output_particles()
*/
#define IMP_PAIR_MODIFIER(Name)                                   \
  IMP_IMPLEMENT(void apply(const ParticlePair& a) const); \
  IMP_IMPLEMENT_INLINE(void apply_index(Model *m, \
                                        const ParticleIndexPair& a) const, {\
    return Name::apply(IMP::internal::get_particle(m,a));               \
    })                                                                  \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)

//! Declare the functions needed for a PairModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::PairDerivativeModifier::apply()
    - IMP::PairDerivativeModifier::get_input_particles()
    - IMP::PairDerivativeModifier::get_output_particles()
*/
#define IMP_PAIR_DERIVATIVE_MODIFIER(Name)                        \
  IMP_IMPLEMENT(void apply(const ParticlePair& a,\
                           DerivativeAccumulator&da) const);            \
  IMP_IMPLEMENT_INLINE(void apply_index(Model *m,                       \
                                        const ParticleIndexPair& a,  \
                                        DerivativeAccumulator&da) const, { \
    return Name::apply(IMP::internal::get_particle(m,a), da);           \
                       });                                              \
  IMP_IMPLEMENT_INLINE(void apply_indexes(Model *m,\
                                          const ParticleIndexPairs &ps,    \
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



//! Declare the functions needed for a PairModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::PairModifier::apply(IMP::Particle*)
    - IMP::PairModifier::get_input_particles()
    - IMP::PairModifier::get_output_particles()
*/
#define IMP_INDEX_PAIR_MODIFIER(Name)                     \
  IMP_IMPLEMENT_INLINE(void apply(const ParticlePair& a) const, {  \
    apply_index(IMP::internal::get_model(a),                            \
                IMP::internal::get_index(a));                           \
    });                                                                 \
  IMP_IMPLEMENT(void apply_index(Model *m,\
                                 const ParticleIndexPair& a) const);   \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)

//! Declare the functions needed for a PairModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::PairDerivativeModifier::apply()
    - IMP::PairDerivativeModifier::get_input_particles()
    - IMP::PairDerivativeModifier::get_output_particles()
*/
#define IMP_INDEX_PAIR_DERIVATIVE_MODIFIER(Name)                        \
  IMP_IMPLEMENT_INLINE(void apply(const ParticlePair& a,\
                                  DerivativeAccumulator&da) const, {    \
    apply_index(IMP::internal::get_model(a),                            \
                IMP::internal::get_index(a), da);                       \
                       });                                              \
  IMP_IMPLEMENT(void apply_index(Model *m, const ParticleIndexPair& a,\
                                 DerivativeAccumulator&da) const);      \
  IMP_IMPLEMENT_INLINE(void apply_indexes(Model *m,\
                                          const ParticleIndexPairs &ps,    \
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
#define IMP_IMPLEMENT_PAIR_CONTAINER(Name)                        \
  void apply(const PairModifier *sm) const {                       \
    for_each(IMP::internal::ModifierApplier<PairModifier>          \
             (get_model(), sm));                                        \
  }                                                                     \
  void apply(const PairDerivativeModifier *sm,                     \
             DerivativeAccumulator &da) const {                         \
    for_each(IMP::internal                                              \
             ::ModifierDerivativeApplier<PairDerivativeModifier>   \
             (get_model(), sm, da));                                    \
  }                                                                     \
  double evaluate(const PairScore *s,                              \
                  DerivativeAccumulator *da) const {                    \
    return for_each(IMP::internal::ScoreAccumulator<PairScore>     \
                    (get_model(), s, da)).get_score();                  \
  }                                                                     \
  double evaluate_if_good(const PairScore *s,                      \
                          DerivativeAccumulator *da, double max) const { \
    return for_each(IMP::internal::ScoreAccumulatorIfGood<PairScore> \
                    (get_model(), s, max, da)).get_score();             \
  }                                                                     \
  ParticlesTemp get_all_possible_particles() const;                     \
  IMP_OBJECT(Name)
#endif





//! Declare the needed functions for a PairContainer
/** In addition to the methods of IMP_OBJECT, it declares
    - IMP::PairContainer::get_contains_particle_particle_pair()
    - IMP::PairContainer::get_number_of_particle_particle_pairs()
    - IMP::PairContainer::get_particle_particle_pair()
    - IMP::PairContainer::apply()
    - IMP::PairContainer::evaluate()
    - IMP::Interaction::get_input_objects()

    You need to define a template method with the signature
\code
template <class Functor>
Functor for_each(Functor f);
\endcode
    that applied the functor to each thing in the container.
*/
#define IMP_PAIR_CONTAINER(Name)                                  \
  IMP_IMPLEMENT(bool get_is_changed() const);                           \
  IMP_IMPLEMENT(bool get_contains_particle_pair(const ParticlePair& p)   \
                const);                                                 \
  IMP_IMPLEMENT(ParticleIndexPairs get_indexes() const);                   \
  IMP_IMPLEMENT(ParticleIndexPairs get_all_possible_indexes() const);      \
  IMP_IMPLEMENT(void do_before_evaluate());                             \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles() const);             \
  IMP_IMPLEMENT(ContainersTemp get_input_containers() const);           \
  IMP_IMPLEMENT_PAIR_CONTAINER(Name)


//! Declare the needed functions for an active PairContainer
/** In addition to the methods of IMP_PAIR_CONTAINER(), it declares
    - IMP::ScoreState::get_input_particles()
    - IMP::ScoreState::get_input_containers()
    - IMP::ScoreState::do_before_evaluate()
*/
#define IMP_ACTIVE_PAIR_CONTAINER(Name)                           \
  IMP_PAIR_CONTAINER(name)

/** These macros avoid various inefficiencies.

    The macros take the name of the sequence and the operation to
    peform. The item in the sequence is called _1, it's index is _2.
    Use it like
    \code
    IMP_FOREACH_PARTICLE(sc, std::cout << "Item " << _2
    << " is " << _1->get_name() << std::endl);
    \endcode
*/
#define IMP_FOREACH_PAIR(sequence, operation) do {                \
    IMP::ParticlePairsTemp imp_all=sequence->get();   \
  for (unsigned int _2=0;                                               \
       _2 != imp_all.size();                                            \
       ++_2) {                                                          \
    IMP::ParticlePair _1= imp_all[_2];               \
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
#define IMP_FOREACH_PAIR_INDEX(sequence, operation)               \
  do {                                                                  \
    if (sequence->get_provides_access()) {                              \
      const ParticleIndexPairs &imp_foreach_access                         \
        =sequence->get_access();                                        \
      for (unsigned int _2=0; _2< imp_foreach_access.size(); ++_2) {    \
        IMP::ParticleIndexPair _1= imp_foreach_access[_2];          \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) { break;}                              \
      }                                                                 \
    } else {                                                            \
      ParticleIndexPairs imp_foreach_indexes              \
        =sequence->get_indexes();                                       \
      for (unsigned int _2=0;                                           \
           _2 != imp_foreach_indexes.size();                            \
           ++_2) {                                                      \
        IMP::ParticleIndexPair _1= imp_foreach_indexes[_2];            \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) break;                                 \
      }                                                                 \
    }                                                                   \
  } while (false)

#endif  /* IMPKERNEL_PAIR_MACROS_H */
