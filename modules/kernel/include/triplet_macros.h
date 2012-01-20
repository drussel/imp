/**
 *  \file triplet_macros.h    \brief Macros for various classes.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_TRIPLET_MACROS_H
#define IMPKERNEL_TRIPLET_MACROS_H

#define IMP_TRIPLET_SCORE_BASE(Name)                                 \
  IMP_IMPLEMENT_INLINE(double evaluate(const ParticleTripletsTemp &ps,    \
                                       DerivativeAccumulator *da) const, { \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      ret+=Name::evaluate(ps[i], da);                                   \
    }                                                                   \
    return ret;                                                         \
                       });                                              \
  IMP_IMPLEMENT_INLINE(double evaluate_indexes(Model *m,                \
                                              const ParticleIndexTriplets &ps, \
                                              DerivativeAccumulator *da)\
    const, {                                                            \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      ret+=Name::evaluate_index(m, ps[i], da);                          \
    }                                                                   \
    return ret;                                                         \
                       });                                              \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_indexes(Model *m, \
                                     const ParticleIndexTriplets &ps, \
                          DerivativeAccumulator *da,                    \
                                                      double max) const, { \
    double ret=0;                                                       \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      double cur=Name::evaluate_if_good_index(m, ps[i], da, max);       \
      max-=cur;                                                         \
      ret+=cur;                                                         \
      if (max <0) break;                                                \
    }                                                                   \
    return ret;                                                         \
                      });                                               \
  IMP_OBJECT(Name)



//! Declare the functions needed for a TripletScore
/** In addition to the methods done by IMP_INTERACTON, it declares
    - IMP::TripletScore::evaluate(IMP::Particle*,
    IMP::DerivativeAccumulator*)
    - IMP::TripletScore::get_input_particles()
    - IMP::TripletScore::get_output_particles()

    See IMP_SIMPLE_TRIPLET_SCORE() for a way of providing an
    implementation of that method.
*/
#define IMP_TRIPLET_SCORE(Name)                                      \
  IMP_IMPLEMENT(double evaluate(const ParticleTriplet& p,\
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(double evaluate_index(Model *m,                  \
                                const ParticleIndexTriplet& p,           \
                                     DerivativeAccumulator *da) const, { \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
                                    });                                 \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,          \
                          const ParticleIndexTriplet& p,                       \
                          DerivativeAccumulator *da,                    \
                                                     double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*p) const) ;  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *) const) ; \
  IMP_TRIPLET_SCORE_BASE(Name)

//! Declare the functions needed for a TripletScore
/** In addition to the methods declared and defined by IMP_TRIPLET_SCORE,
    the macro provides an implementation of
    - IMP::TripletScore::get_input_particles()
    - IMP::TripletScore::get_input_containers()
    which assume that only the passed particle serves as input to the
    score.
*/
#define IMP_SIMPLE_TRIPLET_SCORE(Name)                               \
  IMP_IMPLEMENT(double evaluate(const ParticleTriplet& p,    \
                                DerivativeAccumulator *da) const);      \
  IMP_IMPLEMENT_INLINE(double evaluate(Model *m,                        \
                                  const ParticleIndexTriplet& p,  \
                                       DerivativeAccumulator *da) const, { \
    return evaluate(IMP::internal::get_particle(m,p), da);              \
                       });                                              \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,          \
                          const ParticleIndexTriplet& p,                       \
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
  (const ParticleTriplet& vt) const, {                                      \
    return Restraints(1, create_restraint(this, vt));             \
                       });                                        \
  IMP_TRIPLET_SCORE_BASE(Name)



//! Declare the functions needed for a complex TripletScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::TripletScore::evaluate()
    - IMP::TripletScore::get_input_particles()
    - IMP::TripletScore::get_output_particles()
    - IMP::TripletScore::evaluate_if_good
*/
#define IMP_COMPOSITE_TRIPLET_SCORE(Name)                            \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle *p) const);  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *p) const); \
  IMP_IMPLEMENT_INLINE(double evaluate(const ParticleTriplet& p,     \
                                       DerivativeAccumulator *da) const, { \
    return evaluate_index(IMP::internal::get_model(p),                  \
                  IMP::internal::get_index(p), da);                     \
                       });                                              \
  IMP_IMPLEMENT(double evaluate_index(Model *m, const ParticleIndexTriplet& p,\
                                      DerivativeAccumulator *da) const); \
  IMP_IMPLEMENT(double evaluate_if_good_index(Model *m,                 \
                          const ParticleIndexTriplet& p,                       \
                          DerivativeAccumulator *da,                    \
                                              double max) const);       \
  IMP_TRIPLET_SCORE_BASE(Name)

//! Declare the functions needed for a complex TripletScore
/** In addition to the methods done by IMP_OBJECT(), it declares
    - IMP::TripletScore::evaluate()
    - IMP::TripletScore::get_input_particles()
    - IMP::TripletScore::get_output_particles()
    - IMP::TripletScore::evaluate_if_good
*/
#define IMP_INDEX_TRIPLET_SCORE(Name)                                \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle *p) const);  \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle *p) const); \
  IMP_IMPLEMENT_INLINE(double evaluate(const ParticleTriplet& p,\
                                        DerivativeAccumulator *da) const, { \
    return evaluate_index(IMP::internal::get_model(p),                  \
                  IMP::internal::get_index(p),                          \
                  da);                                                  \
                        });                                             \
  IMP_IMPLEMENT(double evaluate_index(Model *m, const ParticleIndexTriplet& p,\
                                      DerivativeAccumulator *da) const); \
  IMP_IMPLEMENT_INLINE(double evaluate_if_good_index(Model *m,         \
                          const ParticleIndexTriplet& p,                      \
                          DerivativeAccumulator *da,                    \
                                                      double max) const, { \
    IMP_UNUSED(max);                                                    \
    return evaluate_index(m, p, da);                                    \
                        });                                             \
  IMP_TRIPLET_SCORE_BASE(Name)



//! Declare the functions needed for a TripletPredicate
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::TripletPredicate::get_value()
    - IMP::TripletPredicate::get_input_particles()
    - IMP::TripletPredicate::get_output_particles()
*/
#define IMP_TRIPLET_PREDICATE(Name)                                   \
  IMP_IMPLEMENT(int get_value(const ParticleTriplet& a) const);   \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                              ParticleTripletsTemp &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           const ParticleIndexTriplet& vt)\
                       const, {                                         \
    return Name::get_value(internal::get_particle(m, vt));              \
                       });                                              \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                     const ParticleIndexTriplets &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_OBJECT(Name)


//! Declare the functions needed for a TripletPredicate
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::TripletPredicate::get_value_index()
    - IMP::TripletPredicate::get_input_particles()
    - IMP::TripletPredicate::get_output_particles()
*/
#define IMP_INDEX_TRIPLET_PREDICATE(Name, gv)                        \
  IMP_IMPLEMENT_INLINE(int get_value(const ParticleTriplet& a) const, { \
    return get_value_index(IMP::internal::get_model(a),                 \
                     IMP::internal::get_index(a));                      \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(Ints get_value(const                             \
                                      ParticleTripletsTemp &o) const, {   \
    Ints ret(o.size());                                                 \
    for (unsigned int i=0; i< o.size(); ++i) {                          \
      ret[i]+= Name::get_value(o[i]);                                   \
    }                                                                   \
    return ret;                                                         \
                       })                                               \
  IMP_IMPLEMENT_INLINE(int get_value_index(Model *m,                    \
                                           const ParticleIndexTriplet& pi)\
                       const, {                                         \
    gv;                                                                 \
                       })                                               \
  IMP_IMPLEMENT_INLINE(Ints get_value_index(Model *m,                   \
                                const ParticleIndexTriplets &o) const, { \
   Ints ret(o.size());                                                  \
   for (unsigned int i=0; i< o.size(); ++i) {                           \
     ret[i]+= Name::get_value_index(m, o[i]);                           \
   }                                                                    \
   return ret;                                                          \
                       });                                              \
  IMP_IMPLEMENT_INLINE(ParticlesTemp get_input_particles(Particle*p) const, { \
   return ParticlesTemp(1, p);                                          \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(ContainersTemp get_input_containers(Particle*) const, { \
   return ContainersTemp();                                             \
    });                                                                 \
 IMP_OBJECT_INLINE(Name,IMP_UNUSED(out),)


//! Declare the functions needed for a TripletModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::TripletModifier::apply(IMP::Particle*)
    - IMP::TripletModifier::get_input_particles()
    - IMP::TripletModifier::get_output_particles()
*/
#define IMP_TRIPLET_MODIFIER(Name)                                   \
  IMP_IMPLEMENT(void apply(const ParticleTriplet& a) const); \
  IMP_IMPLEMENT_INLINE(void apply_index(Model *m, \
                                        const ParticleIndexTriplet& a) const, {\
    return Name::apply(IMP::internal::get_particle(m,a));               \
    })                                                                  \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)

//! Declare the functions needed for a TripletModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::TripletDerivativeModifier::apply()
    - IMP::TripletDerivativeModifier::get_input_particles()
    - IMP::TripletDerivativeModifier::get_output_particles()
*/
#define IMP_TRIPLET_DERIVATIVE_MODIFIER(Name)                        \
  IMP_IMPLEMENT(void apply(const ParticleTriplet& a,\
                           DerivativeAccumulator&da) const);            \
  IMP_IMPLEMENT_INLINE(void apply_index(Model *m,                       \
                                        const ParticleIndexTriplet& a,  \
                                        DerivativeAccumulator&da) const, { \
    return Name::apply(IMP::internal::get_particle(m,a), da);           \
                       });                                              \
  IMP_IMPLEMENT_INLINE(void apply_indexes(Model *m,\
                                          const ParticleIndexTriplets &ps,    \
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



//! Declare the functions needed for a TripletModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::TripletModifier::apply(IMP::Particle*)
    - IMP::TripletModifier::get_input_particles()
    - IMP::TripletModifier::get_output_particles()
*/
#define IMP_INDEX_TRIPLET_MODIFIER(Name)                     \
  IMP_IMPLEMENT_INLINE(void apply(const ParticleTriplet& a) const, {  \
    apply_index(IMP::internal::get_model(a),                            \
                IMP::internal::get_index(a));                           \
    });                                                                 \
  IMP_IMPLEMENT(void apply_index(Model *m,\
                                 const ParticleIndexTriplet& a) const);   \
  IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle*) const);    \
  IMP_IMPLEMENT(ParticlesTemp get_output_particles(Particle*) const);   \
  IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle*) const);  \
  IMP_IMPLEMENT(ContainersTemp get_output_containers(Particle*) const); \
  IMP_OBJECT(Name)

//! Declare the functions needed for a TripletModifier
/** In addition to the methods done by IMP_OBJECT, it declares
    - IMP::TripletDerivativeModifier::apply()
    - IMP::TripletDerivativeModifier::get_input_particles()
    - IMP::TripletDerivativeModifier::get_output_particles()
*/
#define IMP_INDEX_TRIPLET_DERIVATIVE_MODIFIER(Name)                        \
  IMP_IMPLEMENT_INLINE(void apply(const ParticleTriplet& a,\
                                  DerivativeAccumulator&da) const, {    \
    apply_index(IMP::internal::get_model(a),                            \
                IMP::internal::get_index(a), da);                       \
                       });                                              \
  IMP_IMPLEMENT(void apply_index(Model *m, const ParticleIndexTriplet& a,\
                                 DerivativeAccumulator&da) const);      \
  IMP_IMPLEMENT_INLINE(void apply_indexes(Model *m,\
                                          const ParticleIndexTriplets &ps,    \
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
#define IMP_IMPLEMENT_TRIPLET_CONTAINER(Name)                        \
  void apply(const TripletModifier *sm) const {                       \
    template_apply(sm);                                                 \
  }                                                                     \
  void apply(const TripletDerivativeModifier *sm,                     \
             DerivativeAccumulator &da) const {                         \
    template_apply(sm, da);                                             \
  }                                                                     \
  double evaluate(const TripletScore *s,                              \
                  DerivativeAccumulator *da) const {                    \
    return template_evaluate(s, da);                                    \
  }                                                                     \
  double evaluate_if_good(const TripletScore *s,                      \
                          DerivativeAccumulator *da, double max) const { \
    return template_evaluate_if_good(s, da, max);                       \
  }                                                                     \
  ParticlesTemp get_contained_particles() const;                        \
  IMP_OBJECT(Name)
#endif



/** Implement the needed template functions for a container.
    \param[in] Name the name
    \param[in] LOOP do the loop. There should be a line IMP_OPERATION and
    the current item should be in a variable named item at that time.
 */
#define IMP_IMPLEMENT_TRIPLET_CONTAINER_OPERATIONS(Name, LOOP)       \
  public:                                                               \
  IMP_IMPLEMENTATION_TEMPLATE_1(class SM,                               \
  void template_apply(const SM *sm,                                     \
                      DerivativeAccumulator &da) const, {               \
    LOOP(call_apply_index(sm, item, da));                               \
                                });                                     \
  IMP_IMPLEMENTATION_TEMPLATE_1(class SM,                               \
    void template_apply(const SM *sm) const, {                          \
    LOOP(call_apply_index(sm, item));                                   \
                                });                                     \
  IMP_IMPLEMENTATION_TEMPLATE_1(class SS,                               \
  double template_evaluate(const SS *s,                                 \
                           DerivativeAccumulator *da) const, {          \
    double ret=0;                                                       \
    LOOP({                                                              \
        double cur=call_evaluate_index(s, item, da);                    \
      ret+=cur;                                                         \
      });                                                               \
    return ret;                                                         \
                                });                                     \
  IMP_IMPLEMENTATION_TEMPLATE_1(class SS,                               \
  double template_evaluate_if_good(const SS *s,                         \
                                   DerivativeAccumulator *da,           \
                                   double max) const, {                 \
    double ret=0;                                                       \
    LOOP({                                                              \
        double cur=call_evaluate_if_good_index(s, item, da, max);       \
      ret+=cur;                                                         \
      max-=cur;                                                         \
      if (max < 0) return ret;                                          \
      });                                                               \
    return ret;                                                         \
                                });


//! Declare the needed functions for a TripletContainer
/** In addition to the methods of IMP_OBJECT, it declares
    - IMP::TripletContainer::get_contains_particle_particle_triplet()
    - IMP::TripletContainer::get_number_of_particle_particle_triplets()
    - IMP::TripletContainer::get_particle_particle_triplet()
    - IMP::TripletContainer::apply()
    - IMP::TripletContainer::evaluate()
    - IMP::Interaction::get_input_objects()
*/
#define IMP_TRIPLET_CONTAINER(Name)                      \
  IMP_IMPLEMENT(bool get_contents_changed() const);                     \
  IMP_IMPLEMENT(bool get_contains_particle_triplet(const ParticleTriplet& p)\
                const);                                       \
  IMP_IMPLEMENT(ParticleIndexTriplets get_indexes() const);                   \
  IMP_IMPLEMENT(ParticleIndexTriplets get_all_possible_indexes() const);      \
  IMP_IMPLEMENTATION_TEMPLATE_1(class S,                                \
    Restraints create_decomposition_t(S *s) const, {                    \
    ParticleIndexTriplets all= get_all_possible_indexes();            \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]=  IMP::create_restraint(s,                           \
                                              IMP::internal::           \
                                              get_particle(get_model(), \
                                                           all[i]));    \
    }                                                                   \
    return ret;                                                         \
                                });                                     \
  IMP_IMPLEMENT_INLINE(Restraints                                       \
  create_decomposition(TripletScore *s) const, {                      \
    ParticleIndexTriplets all= get_all_possible_indexes();                    \
    Restraints ret(all.size());                                         \
    for (unsigned int i=0; i< all.size(); ++i) {                        \
      ret[i]= new IMP::core::TripletRestraint(s,                      \
            IMP::internal::get_particle(get_model(), all[i]));          \
    }                                                                   \
    return ret;                                                         \
                       });                                              \
  IMP_IMPLEMENT_TRIPLET_CONTAINER(Name)


#ifndef SWIG
//! Declare the needed functions for a TripletFilter
/** In addition to the methods done by all the macros, it declares
    - IMP::TripletFilter::get_contains()
    - IMP::TripletFilter::get_input_particles()
*/
#define IMP_TRIPLET_FILTER(Name)                                     \
public:                                                                 \
 IMP_IMPLEMENT(bool get_contains(const ParticleTriplet& p) const); \
 IMP_IMPLEMENT_INLINE(bool get_contains(Model *m,                       \
                                        const ParticleIndexTriplet& p)\
                      const, {                                          \
   return get_contains(IMP::internal::get_particle(m,p));               \
                      });                                               \
 IMP_IMPLEMENT(ParticlesTemp get_input_particles(Particle* t) const);   \
 IMP_IMPLEMENT(ContainersTemp get_input_containers(Particle* t) const); \
 IMP_IMPLEMENT_INLINE(void filter_in_place(Model *m,\
                                   ParticleIndexTriplets &ps) const, { \
   ps.erase(std::remove_if(ps.begin(), ps.end(),                        \
                           IMP::internal::GetContainsIndex<Name>(this,  \
                                                                 m)),   \
            ps.end());                                                  \
                      });                                               \
 IMP_IMPLEMENT_INLINE(void filter_in_place(ParticleTripletsTemp &ps) const, { \
   ps.erase(std::remove_if(ps.begin(), ps.end(),                        \
                           IMP::internal::GetContains<Name>(this)),   \
            ps.end());                                                  \
   });                                                                  \
 IMP_OBJECT(Name)
#else
#define IMP_TRIPLET_FILTER(Name)                                     \
  bool get_contains(const ParticleTriplet& p) const;                    \
  bool get_contains(Model *m,const ParticleIndexTriplet& p) const;           \
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
#define IMP_FOREACH_TRIPLET(sequence, operation) do {                \
    IMP::ParticleTripletsTemp imp_all=sequence->get();   \
  for (unsigned int _2=0;                                               \
       _2 != imp_all.size();                                            \
       ++_2) {                                                          \
    IMP::ParticleTriplet _1= imp_all[_2];               \
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
#define IMP_FOREACH_TRIPLET_INDEX(sequence, operation)               \
  do {                                                                  \
    if (sequence->get_provides_access()) {                              \
      const ParticleIndexTriplets &imp_foreach_access                         \
        =sequence->get_access();                                        \
      for (unsigned int _2=0; _2< imp_foreach_access.size(); ++_2) {    \
        IMP::ParticleIndexTriplet _1= imp_foreach_access[_2];          \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) { break;}                              \
      }                                                                 \
    } else {                                                            \
      ParticleIndexTriplets imp_foreach_indexes              \
        =sequence->get_indexes();                                       \
      for (unsigned int _2=0;                                           \
           _2 != imp_foreach_indexes.size();                            \
           ++_2) {                                                      \
        IMP::ParticleIndexTriplet _1= imp_foreach_indexes[_2];            \
        bool imp_foreach_break=false;                                   \
        operation                                                       \
          if (imp_foreach_break) break;                                 \
      }                                                                 \
    }                                                                   \
  } while (false)

#endif  /* IMPKERNEL_TRIPLET_MACROS_H */
