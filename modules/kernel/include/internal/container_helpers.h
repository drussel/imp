/**
 *  \file container_helpers.h
 *  \brief Internal helpers for container classes.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_INTERNAL_CONTAINER_HELPERS_H
#define IMPKERNEL_INTERNAL_CONTAINER_HELPERS_H

#include "../base_types.h"
#include "../declare_Particle.h"
#include "../declare_Model.h"
#include "utility.h"
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/tuple/tuple.hpp>


IMP_BEGIN_INTERNAL_NAMESPACE

inline bool is_valid(Particle *p) {
  return p;
}
template <unsigned int D>
inline bool is_valid(const ParticleTuple<D> &p) {
  for (unsigned int i=0; i< D; ++i) {
    if (!p[i]) return false;
  }
  return true;
}


inline bool is_inactive(const Particle *p) {
  return !p->get_is_active();
}
template <unsigned int D>
inline bool is_inactive(const ParticleTuple<D> &p) {
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
inline ParticlesTemp flatten(const VT &in) {
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

inline ParticlesTemp flatten(const Particles &in) {
  return get_as<ParticlesTemp>(in);
}



inline std::string streamable(Particle *p) {
  return p->get_name();
}

template <unsigned int D>
inline std::string streamable(const ParticleTuple<D> &p) {
  std::ostringstream oss;
  oss << p;
  return oss.str();
}



template <unsigned int D, class Score>
inline ContainersTemp get_input_containers(Score *s,
                                     const ParticleTuple<D>& p) {
  ContainersTemp ret;
  for (unsigned int i=0; i< D; ++i) {
    ContainersTemp c= s->get_input_containers(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}

template <unsigned int D, class Score>
inline ContainersTemp get_output_containers(Score *s,
                                     const ParticleTuple<D>& p) {
  ContainersTemp ret;
  for (unsigned int i=0; i< D; ++i) {
    ContainersTemp c= s->get_output_containers(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}

template <class Score>
inline ContainersTemp get_input_containers(Score *s,
                                     Particle * const p) {
  return s->get_input_containers(p);
}

template <class Score>
inline ContainersTemp get_output_containers(Score *s,
                                     Particle* const p) {
  return s->get_output_containers(p);
}


template <class Score, class C>
inline ParticlesTemp get_output_particles(Score *s,
                                   const C& p) {
  ParticlesTemp ret;
  for (unsigned int i=0; i< p.size(); ++i) {
    ParticlesTemp c= s->get_output_particles(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}
template <class Score, class C>
inline ParticlesTemp get_input_particles(Score *s,
                                  const C& p) {
  ParticlesTemp ret;
  for (unsigned int i=0; i< p.size(); ++i) {
    ParticlesTemp c= s->get_input_particles(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}
template <class Score, class C>
inline ContainersTemp get_input_containers(Score *s,
                                   const C& p) {
  ContainersTemp ret;
  for (unsigned int i=0; i< p.size(); ++i) {
    ContainersTemp c= s->get_input_containers(p[i]);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}
template <class S>
inline ParticlesTemp get_output_particles(S *s,
                                   Particle *p) {
  return s->get_output_particles(p);
}
template <class S>
inline ParticlesTemp get_input_particles(S *s,
                                     Particle *p) {
  return s->get_input_particles(p);
}

template <class S>
inline ParticlesTemp get_output_particles(S *s,
                                   Pointer<Particle> p) {
  return s->get_output_particles(p);
}
template <class S>
inline ParticlesTemp get_input_particles(S *s,
                                  Pointer<Particle> p) {
  return s->get_input_particles(p);
}
template <class S>
inline ContainersTemp get_input_containers(S *s,
                                   Pointer<Particle> p) {
  return s->get_input_containers(p);
}


inline
Particle* get_particle(Model *m, ParticleIndex pi) {
  return m->get_particle(pi);
}
template <unsigned int D>
ParticleTuple<D> get_particle(Model *m, const ParticleIndexTuple<D> &in) {
  ParticleTuple<D> ret;
  for (unsigned int i=0; i< D; ++i) {
    ret[i]= get_particle(m, in[i]);
    IMP_CHECK_OBJECT(ret[i]);
  }
  return ret;
}

inline ParticlesTemp
get_particle(Model *m, const ParticleIndexes &ps) {
  ParticlesTemp ret(ps.size());
  for (unsigned int i=0; i< ps.size(); ++i) {
    ret[i]= get_particle(m, ps[i]);
  }
  return ret;
}

template <unsigned int D>
inline
vector<ParticleTuple<D> > get_particle(Model *m,
        const vector<ParticleIndexTuple<D> > &ps) {
  vector<ParticleTuple<D> > ret(ps.size());
  for (unsigned int i=0; i< ps.size(); ++i) {
    ret[i]= get_particle(m, ps[i]);
  }
  return ret;
}

inline
ParticleIndex get_index(Particle*p) {
  IMP_CHECK_OBJECT(p);
  return p->get_index();
}
template <unsigned int D>
ParticleIndexTuple<D> get_index(const ParticleTuple<D> &in) {
  ParticleIndexTuple<D> ret;
  for (unsigned int i=0; i< D; ++i) {
    ret[i]= get_index(in[i]);
  }
  return ret;
}

inline
ParticleIndexes get_index(const ParticlesTemp& p) {
  ParticleIndexes ret(p.size(), base::get_invalid_index<ParticleIndexTag>());
  for (unsigned int i=0; i< ret.size(); ++i) {
    ret[i]= get_index(p[i]);
  }
  return ret;
}
template <unsigned int D>
vector<ParticleIndexTuple<D> >
get_index(const vector<ParticleTuple<D> > &in) {
  vector<ParticleIndexTuple<D> > ret(in.size());
  for (unsigned int i=0; i< ret.size(); ++i) {
    ParticleIndexTuple<D> c;
    for (unsigned int j=0; j< D; ++j) {
      c[j]= get_index(in[i][j]);
    }
    ret[i]=c;
  }
  return ret;
}


inline Model *get_model(Particle*p) {
  return p->get_model();
}

inline Model *get_model(const ParticlesTemp&p) {
  IMP_USAGE_CHECK(p.size() >0, "Empty particles list");
  return get_model(p[0]);
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


template <class Filter>
class GetContains {
  const Filter* back_;
public:
  GetContains(const Filter *n): back_(n){}
  template <class T>
  bool operator()(const T &p) const {
    return back_->get_contains(get_model(p),
                               IMP::internal::get_index(p));
  }
};

template <class Filter>
class GetContainsIndex {
  const Filter* back_;
  Model *m_;
public:
  GetContainsIndex(const Filter *n,
              Model *m): back_(n), m_(m){}
  template <class T>
  bool operator()(const T &p) const {
    return back_->get_contains(m_, p);
  }
};




IMP_END_INTERNAL_NAMESPACE

#endif /* IMPKERNEL_INTERNAL_CONTAINER_HELPERS_H */
