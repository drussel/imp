/**
 *  \file quad_macros.h    \brief Macros for various classes.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_QUAD_MACROS_H
#define IMP_QUAD_MACROS_H

#ifndef IMP_DOXYGEN

#define IMP_QUAD_SCORE_BASE(Name)                                 \
  double evaluate(const ParticleQuadsTemp &ps,                         \
                  DerivativeAccumulator *da) const {                    \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      ret+=Name::evaluate(ps[i], da);                                   \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  double evaluate_if_good(const ParticleQuadsTemp &ps,                 \
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
  double evaluate(Model *m, const ParticleIndexQuads &ps,                  \
                  DerivativeAccumulator *da) const {                    \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      ret+=Name::evaluate(m, ps[i], da);                                \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  double evaluate_if_good(Model *m, const ParticleIndexQuads &ps,          \
                          DerivativeAccumulator *da,                    \
                          double max) const {                           \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      double cur=Name::evaluate(m, ps[i], da);                          \
      max-=cur;                                                         \
      ret+=cur;                                                         \
      if (max <0) break;                                                \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT(Name)


#else
#define IMP_QUAD_SCORE_BASE(Name)
#endif


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
  double evaluate(const ParticleQuad& p,                              \
                  DerivativeAccumulator *da) const;                     \
  double evaluate_if_good(const ParticleQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                          double max) const{                            \
    IMP_UNUSED(max);                                                    \
    return evaluate(p, da);                                             \
  }                                                                     \
  double evaluate(Model *m, const ParticleIndexQuad& p,                     \
                  DerivativeAccumulator *da) const {                    \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
  }                                                                     \
  double evaluate_if_good(Model *m,                                     \
                          const ParticleIndexQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                          double max) const{                            \
    IMP_UNUSED(max);                                                    \
    return evaluate(m, p, da);                                          \
  }                                                                     \
  ParticlesTemp get_input_particles(Particle*p) const ;                 \
  ContainersTemp get_input_containers(Particle *) const ;               \
  IMP_QUAD_SCORE_BASE(Name)

//! Declare the functions needed for a QuadScore
/** In addition to the methods declared and defined by IMP_QUAD_SCORE,
    the macro provides an implementation of
    - IMP::QuadScore::get_input_particles()
    - IMP::QuadScore::get_input_containers()
    which assume that only the passed particle serves as input to the
    score.
*/
#define IMP_SIMPLE_QUAD_SCORE(Name)                               \
  double evaluate(const ParticleQuad& p,                        \
                  DerivativeAccumulator *da) const;                     \
  double evaluate_if_good(const ParticleQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                          double max) const{                            \
    IMP_UNUSED(max);                                                    \
    return evaluate(p, da);                                             \
  }                                                                     \
  double evaluate(Model *m, const ParticleIndexQuad& p,                     \
                  DerivativeAccumulator *da) const {                    \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
  }                                                                     \
  double evaluate_if_good(Model *m,                                     \
                          const ParticleIndexQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                          double max) const{                            \
    IMP_UNUSED(max);                                                    \
    return evaluate(m, p, da);                                          \
  }                                                                     \
  ParticlesTemp get_input_particles(Particle*p) const {                 \
    return ParticlesTemp(1,p);                                          \
  }                                                                     \
  ContainersTemp get_input_containers(Particle *) const {               \
    return ContainersTemp();                                            \
  }                                                                     \
  IMP_QUAD_SCORE_BASE(Name)



//! Declare the functions needed for a complex QuadScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::QuadScore::evaluate()
    - IMP::QuadScore::get_input_particles()
    - IMP::QuadScore::get_output_particles()
    - IMP::QuadScore::evaluate_if_good
*/
#define IMP_COMPOSITE_QUAD_SCORE(Name)                            \
  ParticlesTemp get_input_particles(Particle *p) const;                 \
  ContainersTemp get_input_containers(Particle *p) const;               \
  double evaluate(const ParticleQuad& p,                              \
                  DerivativeAccumulator *da) const;                     \
  double evaluate_if_good(const ParticleQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                          double max) const;                            \
  double evaluate(Model *m, const ParticleIndexQuad& p,                     \
                  DerivativeAccumulator *da) const {                    \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
  }                                                                     \
  double evaluate_if_good(Model *m,                                     \
                          const ParticleIndexQuad& p,                       \
                          DerivativeAccumulator *da,                    \
                          double max) const {                           \
    return evaluate_if_good(IMP::internal::get_particle(m,p),           \
                            da, max);                                   \
  }                                                                     \
  IMP_QUAD_SCORE_BASE(Name)

//! Declare the functions needed for a complex QuadScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::QuadScore::evaluate()
    - IMP::QuadScore::get_input_particles()
    - IMP::QuadScore::get_output_particles()
    - IMP::QuadScore::evaluate_if_good
*/
#define IMP_INDEX_QUAD_SCORE(Name)                                \
  ParticlesTemp get_input_particles(Particle *p) const;                 \
  ContainersTemp get_input_containers(Particle *p) const;               \
  double evaluate(const ParticleQuad& p,                             \
                  DerivativeAccumulator *da) const {                    \
    return evaluate(IMP::internal::get_model(p),                        \
                  IMP::internal::get_index(p),                          \
                  da);                                                  \
  }                                                                     \
  double evaluate_if_good(const ParticleQuad& p,                     \
                          DerivativeAccumulator *da,                    \
                          double max) const{                            \
    return evaluate_if_good(IMP::internal::get_model(p),                \
                            IMP::internal::get_index(p),                \
                            da, max);                                   \
  }                                                                     \
  double evaluate(Model *m, const ParticleIndexQuad& p,                   \
                  DerivativeAccumulator *da) const;                     \
  double evaluate_if_good(Model *m,                                     \
                          const ParticleIndexQuad& p,                      \
                          DerivativeAccumulator *da,                    \
                          double max) const {                           \
    IMP_UNUSED(max);                                                    \
    return evaluate(m, p, da);                                          \
  }                                                                     \
  IMP_QUAD_SCORE_BASE(Name)






//! Declare the functions needed for a QuadModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::QuadModifier::apply(IMP::Particle*)
    - IMP::QuadModifier::get_input_particles()
    - IMP::QuadModifier::get_output_particles()
*/
#define IMP_QUAD_MODIFIER(Name)                                   \
  void apply(const ParticleQuad& a) const;                             \
  void apply(const ParticleQuadsTemp &ps) const {                      \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply(ps[i]);                                               \
    }                                                                   \
  }                                                                     \
  void apply(Model *m, const ParticleIndexQuad& a) const {            \
    return Name::apply(get_particle(m,a));                              \
  }                                                                     \
  void apply(Model *m, const ParticleIndexQuads &ps) const {               \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply(m, ps[i]);                                            \
    }                                                                   \
  }                                                                     \
  ParticlesTemp get_input_particles(Particle*) const;                   \
  ParticlesTemp get_output_particles(Particle*) const;                  \
  ContainersTemp get_input_containers(Particle*) const;                 \
  ContainersTemp get_output_containers(Particle*) const;                \
  IMP_OBJECT(Name)

//! Declare the functions needed for a QuadModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::QuadDerivativeModifier::apply(IMP::Particle*)
    - IMP::QuadDerivativeModifier::get_input_particles()
    - IMP::QuadDerivativeModifier::get_output_particles()
*/
#define IMP_QUAD_DERIVATIVE_MODIFIER(Name)                        \
  void apply(const ParticleQuad& a, DerivativeAccumulator&da) const;    \
  void apply(const ParticleQuadsTemp &ps,                              \
             DerivativeAccumulator &da) const {                         \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Name::apply(ps[i], da);                                           \
    }                                                                   \
  }                                                                     \
  void apply(Model *m, const ParticleIndexQuad& a,\
             DerivativeAccumulator&da) const {                          \
    return Name::apply(IMP::internal::get_particle(m,a), da);           \
  }                                                                     \
  void apply(Model *m, const ParticleIndexQuads &ps,                       \
             DerivativeAccumulator&da) const {                          \
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
#define IMP_IMPLEMENT_QUAD_CONTAINER(Name)                        \
  void apply(const QuadModifier *sm) {                             \
    template_apply(sm);                                                 \
  }                                                                     \
  void apply(const QuadDerivativeModifier *sm,                     \
             DerivativeAccumulator &da) {                               \
    template_apply(sm, da);                                             \
  }                                                                     \
  double evaluate(const QuadScore *s,                              \
                  DerivativeAccumulator *da) const {                    \
    return template_evaluate(s, da);                                    \
  }                                                                     \
  double evaluate_if_good(const QuadScore *s,                      \
                          DerivativeAccumulator *da, double max) const { \
    return template_evaluate_if_good(s, da, max);                       \
  }                                                                     \
  ParticlesTemp get_contained_particles() const;                        \
  IMP_OBJECT(Name)
#endif







//! Declare the needed functions for a QuadContainer
/** In addition to the methods of IMP_OBJECT, it declares
    - IMP::QuadContainer::get_contains_particle_particle_quad()
    - IMP::QuadContainer::get_number_of_particle_particle_quads()
    - IMP::QuadContainer::get_particle_particle_quad()
    - IMP::QuadContainer::apply()
    - IMP::QuadContainer::evaluate()
    - IMP::Interaction::get_input_objects()
*/
#define IMP_QUAD_CONTAINER(Name)                                  \
  QuadContainerPair get_added_and_removed_containers() const;      \
  bool get_contains_particle_quad(const ParticleQuad& p) const;      \
  unsigned int get_number_of_particle_quads() const;                     \
  ParticleQuad get_particle_quad(unsigned int i) const;                  \
  IMP_IMPLEMENT_QUAD_CONTAINER(Name)


#ifndef SWIG
//! Declare the needed functions for a QuadFilter
/** In addition to the methods done by all the macros, it declares
    - IMP::QuadFilter::get_contains_particle_LCCLASNAME()
    - IMP::QuadFilter::get_input_particles()
*/
#define IMP_QUAD_FILTER(Name)                                     \
public:                                                                 \
 bool get_contains(const ParticleQuad& p) const;                   \
 bool get_contains(Model *m,const ParticleIndexQuad& p) const {         \
   return get_contains(IMP::internal::get_particle(m,p));               \
 }                                                                      \
 ParticlesTemp get_input_particles(Particle* t) const;                  \
 ContainersTemp get_input_containers(Particle* t) const;                \
 void filter_in_place(ParticleQuadsTemp &ps) const {                   \
   ps.erase(std::remove_if(ps.begin(), ps.end(),                        \
                           IMP::internal::GetContains<Name>(this)),     \
            ps.end());                                                  \
 }                                                                      \
 void filter_in_place(Model *m, ParticleIndexQuads &ps) const {             \
   ps.erase(std::remove_if(ps.begin(), ps.end(),                        \
                IMP::internal::GetContainsIndex<Name>(this, m)), \
            ps.end());                                                  \
 }                                                                      \
 IMP_OBJECT(Name)
#else
#define IMP_QUAD_FILTER(Name)                                     \
  bool get_contains(const ParticleQuad& p) const;                    \
  bool get_contains(Model *m,const ParticleIndexQuad& p) const;           \
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
#define IMP_FOREACH_QUAD(sequence, operation)                     \
  do {                                                                  \
    if (sequence->get_provides_access()) {                              \
      const ParticleQuadsTemp &imp_foreach_access                      \
        =sequence->get_access();                                        \
      for (unsigned int _2=0; _2< imp_foreach_access.size(); ++_2) {    \
        IMP::ParticleQuad _1= imp_foreach_access[_2];                   \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) { break;}                              \
      }                                                                 \
    } else {                                                            \
      unsigned int imp_foreach_size                                     \
        = sequence->get_number_of_particle_quads();                      \
      for (unsigned int _2=0;                                           \
           _2 != imp_foreach_size;                                      \
           ++_2) {                                                      \
        IMP::ParticleQuad _1= sequence->get_particle_quad(_2);           \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) break;                                 \
      }                                                                 \
    }                                                                   \
  } while (false)

#endif  /* IMP_QUAD_MACROS_H */
