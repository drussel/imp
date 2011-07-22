/**
 *  \file LCCLASSNAME_macros.h    \brief Macros for various classes.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_HEADERNAME_MACROS_H
#define IMP_HEADERNAME_MACROS_H

#ifndef IMP_DOXYGEN

#define IMP_HEADERNAME_SCORE_BASE(Name)                                 \
  double evaluate(ARGUMENTTYPE a,                                  \
                  DerivativeAccumulator *da) const;                     \
  double evaluate(const PLURALVARIABLETYPE &ps,                         \
                  DerivativeAccumulator *da) const {                    \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      ret+=Name::evaluate(ps[i], da);                                   \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  double evaluate_if_good(const PLURALVARIABLETYPE &ps,                 \
                          DerivativeAccumulator *da,                    \
                          double max) const {                           \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      double cur=Name::evaluate(ps[i], da);                             \
      max-=cur;                                                         \
      ret+=cur;                                                         \
      if (max <0) break;                                                \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  double evaluate_if_good(ARGUMENTTYPE ps,                         \
                          DerivativeAccumulator *da,                    \
                          double ) const {                              \
    return Name::evaluate(ps, da);                                      \
  }                                                                     \
  IMP_OBJECT(Name)

#else
#define IMP_HEADERNAME_SCORE_BASE(Name)
#endif


//! Declare the functions needed for a CLASSNAMEScore
/** In addition to the methods done by IMP_INTERACTON, it declares
    - IMP::CLASSNAMEScore::evaluate(IMP::Particle*,
    IMP::DerivativeAccumulator*)
    - IMP::CLASSNAMEScore::get_input_particles()
    - IMP::CLASSNAMEScore::get_output_particles()

    See IMP_SIMPLE_HEADERNAME_SCORE() for a way of providing an
    implementation of that method.
*/
#define IMP_HEADERNAME_SCORE(Name)                              \
  ParticlesTemp get_input_particles(Particle*) const;           \
  ContainersTemp get_input_containers(Particle *) const;        \
  IMP_HEADERNAME_SCORE_BASE(Name)

//! Declare the functions needed for a CLASSNAMEScore
/** In addition to the methods declared and defined by IMP_HEADERNAME_SCORE,
    the macro provides an implementation of
    - IMP::CLASSNAMEScore::get_input_particles()
    - IMP::CLASSNAMEScore::get_input_containers()
    which assume that only the passed particle serves as input to the
    score.
*/
#define IMP_SIMPLE_HEADERNAME_SCORE(Name)                       \
  ParticlesTemp get_input_particles(Particle*p) const {         \
    return ParticlesTemp(1,p);                                  \
  }                                                             \
  ContainersTemp get_input_containers(Particle *) const {       \
    return ContainersTemp();                                    \
  }                                                             \
  IMP_HEADERNAME_SCORE_BASE(Name)



//! Declare the functions needed for a complex CLASSNAMEScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::CLASSNAMEScore::evaluate()
    - IMP::CLASSNAMEScore::get_input_particles()
    - IMP::CLASSNAMEScore::get_output_particles()
    - IMP::CLASSNAMEScore::evaluate_if_good
*/
#define IMP_COMPOSITE_HEADERNAME_SCORE(Name)                            \
  ParticlesTemp get_input_particles(Particle *p) const;                 \
  ContainersTemp get_input_containers(Particle *p) const;               \
  double evaluate(ARGUMENTTYPE p,                                  \
                  DerivativeAccumulator *da) const;                     \
  double evaluate(const PLURALVARIABLETYPE& ps,                         \
                  DerivativeAccumulator *da) const {                    \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      ret+=Name::evaluate(ps[i], da);                                   \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  double evaluate_if_good(const PLURALVARIABLETYPE &ps,                 \
                          DerivativeAccumulator *da,                    \
                          double max) const {                           \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      double cur=Name::evaluate_if_good(ps[i], da, max);                \
      max-=cur;                                                         \
      ret+=cur;                                                         \
      if (max <0) break;                                                \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  double evaluate_if_good( ARGUMENTTYPE ps,                        \
                          DerivativeAccumulator *da,                    \
                          double ) const;                               \
  IMP_OBJECT(Name)






//! Declare the functions needed for a CLASSNAMEModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEModifier::apply(IMP::Particle*)
    - IMP::CLASSNAMEModifier::get_input_particles()
    - IMP::CLASSNAMEModifier::get_output_particles()
*/
#define IMP_HEADERNAME_MODIFIER(Name)                                   \
  void apply(ARGUMENTTYPE a) const;                             \
  void apply(const PLURALVARIABLETYPE &ps) const {                      \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply(ps[i]);                                               \
    }                                                                   \
  }                                                                     \
  void apply(Model *m, PASSINDEXTYPE a) const {                   \
    return Name::apply(get_particle(m,a));                              \
  }                                                                     \
  void apply(Model *m, const PLURALINDEXTYPE &ps) const {               \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply(m, ps[i]);                                            \
    }                                                                   \
  }                                                                     \
  ParticlesTemp get_input_particles(Particle*) const;                   \
  ParticlesTemp get_output_particles(Particle*) const;                  \
  ContainersTemp get_input_containers(Particle*) const;                 \
  ContainersTemp get_output_containers(Particle*) const;                \
  IMP_OBJECT(Name)

//! Declare the functions needed for a CLASSNAMEModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::CLASSNAMEDerivativeModifier::apply(IMP::Particle*)
    - IMP::CLASSNAMEDerivativeModifier::get_input_particles()
    - IMP::CLASSNAMEDerivativeModifier::get_output_particles()
*/
#define IMP_HEADERNAME_DERIVATIVE_MODIFIER(Name)                        \
  void apply(ARGUMENTTYPE a, DerivativeAccumulator&da) const;    \
  void apply(const PLURALVARIABLETYPE &ps,                              \
             DerivativeAccumulator &da) const {                         \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply(ps[i], da);                                           \
    }                                                                   \
  }                                                                     \
  void apply(Model *m, PASSINDEXTYPE a,\
             DerivativeAccumulator&da) const {                          \
    return Name::apply(get_particle(m,a), da);                          \
  }                                                                     \
  void apply(Model *m, const PLURALINDEXTYPE &ps,                       \
             , DerivativeAccumulator&da) const {                        \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply(m, ps[i], da);                                        \
    }                                                                   \
  }                                                                     \
  ParticlesTemp get_input_particles(Particle*) const;                   \
  ParticlesTemp get_output_particles(Particle*) const;                  \
  ContainersTemp get_input_containers(Particle*) const;                 \
  ContainersTemp get_output_containers(Particle*) const;                \
  IMP_OBJECT(Name)





#ifndef IMP_DOXYGEN
#define IMP_IMPLEMENT_HEADERNAME_CONTAINER(Name)                        \
  void apply(const CLASSNAMEModifier *sm) {                             \
    template_apply(sm);                                                 \
  }                                                                     \
  void apply(const CLASSNAMEDerivativeModifier *sm,                     \
             DerivativeAccumulator &da) {                               \
    template_apply(sm, da);                                             \
  }                                                                     \
  double evaluate(const CLASSNAMEScore *s,                              \
                  DerivativeAccumulator *da) const {                    \
    return template_evaluate(s, da);                                    \
  }                                                                     \
  double evaluate_if_good(const CLASSNAMEScore *s,                      \
                          DerivativeAccumulator *da, double max) const { \
    return template_evaluate_if_good(s, da, max);                       \
  }                                                                     \
  ParticlesTemp get_contained_particles() const;                        \
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
*/
#define IMP_HEADERNAME_CONTAINER(Name)                                  \
  CLASSNAMEContainerPair get_added_and_removed_containers() const;      \
  bool get_contains_FUNCTIONNAME(ARGUMENTTYPE p) const;      \
  unsigned int get_number_of_FUNCTIONNAMEs() const;                     \
  VARIABLETYPE get_FUNCTIONNAME(unsigned int i) const;                  \
  IMP_IMPLEMENT_HEADERNAME_CONTAINER(Name)


#ifndef SWIG
//! Declare the needed functions for a CLASSNAMEFilter
/** In addition to the methods done by all the macros, it declares
    - IMP::CLASSNAMEFilter::get_contains_particle_LCCLASNAME()
    - IMP::CLASSNAMEFilter::get_input_particles()
*/
#define IMP_HEADERNAME_FILTER(Name)                                     \
public:                                                                 \
 bool get_contains(ARGUMENTTYPE p) const;                   \
 ParticlesTemp get_input_particles(Particle* t) const;                  \
 ContainersTemp get_input_containers(Particle* t) const;                \
 void filter_in_place(PLURALVARIABLETYPE &ps) const {                   \
   ps.erase(std::remove_if(ps.begin(), ps.end(),                        \
                           IMP::internal::GetContains<Name>(this)),     \
            ps.end());                                                  \
 }                                                                      \
 IMP_OBJECT(Name)
#else
#define IMP_HEADERNAME_FILTER(Name)                                     \
  bool get_contains(ARGUMENTTYPE p) const;                    \
  ParticlesTemp get_input_particles(Particle*t) const;                  \
  ContainersTemp get_input_containers(Particle*t) const;                \
  IMP_OBJECT(Name)
#endif


/** These macros avoid various inefficiencies.

    The macros take the name of the sequence and the operation to
    peform. The item in the sequence is called _1, it's index is _2.
    Use it like
    \code
    IMP_FOREACH_PARTICLE(sc, std::cout << "Item " << _2
                         << " is _1->get_name() << std::endl);
    \endcode
*/
#define IMP_FOREACH_HEADERNAME(sequence, operation)                     \
  do {                                                                  \
    if (sequence->get_provides_access()) {                              \
      const PLURALVARIABLETYPE &imp_foreach_access                      \
        =sequence->get_access();                                        \
      for (unsigned int _2=0; _2< imp_foreach_access.size(); ++_2) {    \
        IMP::VARIABLETYPE _1= imp_foreach_access[_2];                   \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) { break;}                              \
      }                                                                 \
    } else {                                                            \
      unsigned int imp_foreach_size                                     \
        = sequence->get_number_of_FUNCTIONNAMEs();                      \
      for (unsigned int _2=0;                                           \
           _2 != imp_foreach_size;                                      \
           ++_2) {                                                      \
        IMP::VARIABLETYPE _1= sequence->get_FUNCTIONNAME(_2);           \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) break;                                 \
      }                                                                 \
    }                                                                   \
  } while (false)

#endif  /* IMP_HEADERNAME_MACROS_H */
