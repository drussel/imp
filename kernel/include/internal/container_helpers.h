/**
 *  \file container_helpers.h
 *  \brief Internal helpers for container classes.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_INTERNAL_CONTAINER_HELPERS_H
#define IMP_INTERNAL_CONTAINER_HELPERS_H

#include "../base_types.h"
#include "../Particle.h"
#include "../SingletonScore.h"
#include "../PairScore.h"
#include "../SingletonContainer.h"
#include "../PairContainer.h"
#include "../ScoreState.h"
#include "../scoped.h"
#include "utility.h"

#include <boost/tuple/tuple.hpp>


IMP_BEGIN_INTERNAL_NAMESPACE


inline bool is_valid(Particle *p) {
  return p;
}
template <unsigned int D>
bool is_valid(const ParticleTuple<D> &p) {
  for (unsigned int i=0; i< D; ++i) {
    if (!p[i]) return false;
  }
  return true;
}

inline bool is_dirty(Particle *p) {
  return p->get_is_changed();
}
template <unsigned int D>
bool is_dirty( const ParticleTuple<D> &p) {
  for (unsigned int i=0; i< D; ++i) {
    if (p[i]->get_is_changed()) return true;
  }
  return false;
}
inline Particle* prechange(Particle*p) {
  return p->get_prechange_particle();
}

template <unsigned int D>
ParticleTuple<D> prechange(const ParticleTuple<D> &p) {
  ParticleTuple<D> ret;
  for (unsigned int i=0; i< D; ++i) {
    ret[i]= p[i]->get_prechange_particle();
  }
  return ret;
}

inline bool is_inactive(const Particle *p) {
  return !p->get_is_active();
}
template <unsigned int D>
bool is_inactive(const ParticleTuple<D> &p) {
  for (unsigned int i=0; i< D; ++i) {
    if (!p[i]->get_is_active()) return true;
  }
  return false;
}

struct IsInactive {
  template <class T>
  bool operator()(const T& t) {
    return is_inactive(t);
  }
};

template <class VT>
ParticlesTemp flatten(const VT &in) {
  typedef typename VT::value_type T;
  ParticlesTemp ret(in.size()*T::get_dimension());
  for (unsigned int i=0; i< in.size(); ++i) {
    for (unsigned int j=0; j< T::get_dimension(); ++j) {
      ret[i*T::get_dimension()+j]= in[i][j];
    }
  }
  return ret;
}

inline const ParticlesTemp& flatten(const ParticlesTemp &in) {
  return in;
}

inline const ParticlesTemp& flatten(const Particles &in) {
  return in;
}



inline const Particle& streamable(Particle *p) {
  return *p;
}

template <unsigned int D>
inline const ParticleTuple<D>& streamable(const ParticleTuple<D> &p) {
  return p;
}



template <class Score, class C>
ParticlesTemp get_output_particles(Score *s,
                                   const C& p) {
  ParticlesTemp ret;
  for (unsigned int i=0; i< p.size(); ++i) {
    ParticlesTemp c= s->get_output_particles(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}
template <class Score, class C>
ParticlesTemp get_input_particles(Score *s,
                                  const C& p) {
  ParticlesTemp ret;
  for (unsigned int i=0; i< p.size(); ++i) {
    ParticlesTemp c= s->get_input_particles(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}
template <class Score, class C>
ContainersTemp get_input_containers(Score *s,
                                   const C& p) {
  ContainersTemp ret;
  for (unsigned int i=0; i< p.size(); ++i) {
    ContainersTemp c= s->get_input_containers(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}
template <class S>
ParticlesTemp get_output_particles(S *s,
                                   Particle *p) {
  return s->get_output_particles(p);
}
template <class S>
ParticlesTemp get_input_particles(S *s,
                                     Particle *p) {
  return s->get_input_particles(p);
}
template <class S>
ContainersTemp get_input_containers(S *s,
                                   Particle *p) {
  return s->get_input_containers(p);
}
template <class S>
ParticlesTemp get_output_particles(S *s,
                                   Pointer<Particle> p) {
  return s->get_output_particles(p);
}
template <class S>
ParticlesTemp get_input_particles(S *s,
                                  Pointer<Particle> p) {
  return s->get_input_particles(p);
}
template <class S>
ContainersTemp get_input_containers(S *s,
                                   Pointer<Particle> p) {
  return s->get_input_containers(p);
}


inline Model *get_model(Particle*p) {
  return p->get_model();
}

template <unsigned int D>
inline Model *get_model(const ParticleTuple<D>& p) {
  return p[0]->get_model();
}

inline std::string get_name(Particle*p) {
  return p->get_name();
}

template <unsigned int D>
inline std::string get_name(const ParticleTuple<D>& p) {
  return p.get_name();
}


#define IMP_ACTIVE_CONTAINER_DECL(Name)                         \
  private:                                                      \
  unsigned int eval_update_;                                    \
  class Ticker: public ScoreState {                             \
    Name *back_;                                                \
  public:                                                       \
  Ticker(Name *n): ScoreState(n->get_name()+" updater"),        \
                   back_(n){                                    \
    n->eval_update_=std::numeric_limits<unsigned int>::max();   \
  }                                                             \
  IMP_SCORE_STATE(Ticker);                                      \
  };                                                            \
  friend class Ticker;                                          \
  GenericScopedScoreState<Ticker> ticker_;                      \
  unsigned int get_last_update_evaluation() const {             \
  return eval_update_;                                          \
  }                                                             \
  void do_before_evaluate();                                    \
  void do_after_evaluate();                                     \
  bool get_has_model() const { return ticker_.get_is_set();}    \
  ParticlesTemp get_state_input_particles() const;              \
  ContainersTemp get_state_input_containers() const;            \
  void initialize_active_container(Model *m)

#define IMP_ACTIVE_CONTAINER_DEF(Name)                                  \
  void Name::Ticker::do_before_evaluate() {                             \
    back_->do_before_evaluate();                                        \
    back_->eval_update_= back_->get_model()->get_evaluation();          \
  }                                                                     \
  void Name::Ticker::do_after_evaluate(DerivativeAccumulator*) {        \
    back_->do_after_evaluate();                                         \
  }                                                                     \
  ContainersTemp Name::Ticker::get_input_containers() const {           \
    IMP_IF_CHECK(USAGE) {                                               \
      std::string name= back_->get_name();                              \
      {Pointer<Name> rc(back_);}                                        \
      IMP_INTERNAL_CHECK((back_)->get_is_valid(),                       \
                         "Container " << name                           \
                         << " is not ref counted properly"              \
                         << " bad things will happen.");                \
    }                                                                   \
    return back_->get_state_input_containers();                         \
  }                                                                     \
  ContainersTemp Name::Ticker::get_output_containers() const {          \
    return ContainersTemp(1, back_);                                    \
  }                                                                     \
  ParticlesTemp Name::Ticker::get_input_particles() const {             \
    return back_->get_state_input_particles();                          \
  }                                                                     \
  ParticlesTemp Name::Ticker::get_output_particles() const {            \
    return ParticlesTemp();                                             \
  }                                                                     \
  void Name::Ticker::do_show(std::ostream &out) const {                 \
    out << "back is " << *back_ << std::endl;                           \
  }                                                                     \
  void Name::initialize_active_container(Model *m) {                    \
    IMP_INTERNAL_CHECK(!get_has_model(),                                \
                       "Can only set the model once.");                 \
    IMP_LOG(TERSE, "Setting up score state for container "              \
            << get_name()<< std::endl);                                 \
    IMP_INTERNAL_CHECK(!get_is_added_or_removed_container(),            \
                       "No active updating of add/remove containers."); \
    ticker_.set(new Ticker(this), m);                                   \
  }                                                                     \
  IMP_REQUIRE_SEMICOLON_NAMESPACE






template <class M>
struct ApplyIt {
  const M *sm_;
  ApplyIt(const M *sm): sm_(sm){}
  template <class T>
  void operator()(const T& t) const {
    sm_->apply(t);
  }
};

template <class M>
ApplyIt<M> make_apply_it(const M* m) {
  return ApplyIt<M>(m);
}

template <class M>
struct ApplyItDa {
  const M *sm_;
  DerivativeAccumulator& da_;
  ApplyItDa(const M *sm, DerivativeAccumulator &da): sm_(sm), da_(da){}
  template <class T>
  void operator()(const T& t) const {
    sm_->apply(t);
  }
};

template <class M>
ApplyItDa<M> make_apply_it(const M* m, DerivativeAccumulator&da) {
  return ApplyItDa<M>(m, da);
}



template <class M>
struct ApplyItT {
  const M *sm_;
  ApplyItT(const M *sm): sm_(sm){}
  template <class T>
  void operator()(const T& t) const {
    sm_->M::apply(t);
  }
};

template <class M>
ApplyItT<M> make_apply_it_t(const M* m) {
  return ApplyItT<M>(m);
}

template <class M>
struct ApplyItDaT {
  const M *sm_;
  DerivativeAccumulator& da_;
  ApplyItDaT(const M *sm, DerivativeAccumulator &da): sm_(sm), da_(da){}
  template <class T>
  void operator()(const T& t) const {
    sm_->M::apply(t, da_);
  }
};

template <class M>
ApplyItDaT<M> make_apply_it_t(const M* m, DerivativeAccumulator&da) {
  return ApplyItDaT<M>(m, da);
}

#define IMP_EVALUATOR(Name, name, line)                                 \
  template <class S>                                                    \
  struct Evaluate##Name {                                               \
    const S*s_;                                                         \
  DerivativeAccumulator *da_;                                           \
  double score_;                                                        \
  Evaluate##Name(const S* s, DerivativeAccumulator *da): s_(s),         \
    da_(da), score_(0){}                                                \
  template <class T>                                                    \
  void operator()(const T&t) {                                          \
    line;                                                               \
  }                                                                     \
  operator double() const {                                             \
    return score_;                                                      \
  }                                                                     \
  };                                                                    \
  template <class S>                                                    \
  Evaluate##Name<S> make_evaluate_##name(const S*s,                     \
                                         DerivativeAccumulator *da) {   \
    return Evaluate##Name<S>(s, da);                                    \
  }                                                                     \

IMP_EVALUATOR(It, it, score_+= s_->evaluate(t, da_));
IMP_EVALUATOR(Change, change, score_+= s_->evaluate_change(t, da_));
IMP_EVALUATOR(PreChange, prechange, score_+= s_->evaluate_prechange(t, da_));


IMP_EVALUATOR(ItT, it_t, score_+= s_->S::evaluate(t, da_));
IMP_EVALUATOR(ChangeT, change_t, score_+= s_->S::evaluate_change(t, da_));
IMP_EVALUATOR(PreChangeT, prechange_t,
              score_+= s_->S::evaluate_prechange(t, da_));


IMP_END_INTERNAL_NAMESPACE

#endif /* IMP_INTERNAL_CONTAINER_HELPERS_H */
