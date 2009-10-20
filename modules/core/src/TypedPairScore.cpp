/**
 *  \file TypedPairScore.cpp
 *  \brief Delegate to another PairScore depending on particle types.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#include <IMP/core/TypedPairScore.h>

IMPCORE_BEGIN_NAMESPACE

Float TypedPairScore::evaluate(Particle *a, Particle *b,
                               DerivativeAccumulator *da) const
{
  if (!a->has_attribute(typekey_)) {
    set_particle_type(a);
  }
  if (!b->has_attribute(typekey_)) {
    set_particle_type(b);
  }
  Int atype = a->get_value(typekey_);
  Int btype = b->get_value(typekey_);

  ScoreMap::const_iterator psit =
      score_map_.find(std::pair<Int,Int>(std::min(atype, btype),
                                         std::max(atype, btype)));
  if (psit == score_map_.end()) {
    if (!allow_invalid_types_) {
      std::ostringstream oss;
      oss << "Attempt to evaluate TypedPairScore on "
          "particles with invalid types (" << atype << ", " << btype << ")";
      throw ValueException(oss.str().c_str());
    } else {
      return 0.;
    }
  } else {
    PairScore *ps = psit->second.get();
    return ps->evaluate(a, b, da);
  }
}

TypedPairScore::TypedPairScore(IntKey typekey, bool allow_invalid_types)
      : typekey_(typekey), score_map_(),
    allow_invalid_types_(allow_invalid_types) {}


ParticlesList TypedPairScore::get_interacting_particles(Particle *a,
                                                        Particle *b) const {
  return ParticlesList(1, get_input_particles(a,b));
}

ParticlesTemp TypedPairScore::get_input_particles(Particle *a,
                                                  Particle *b) const {
  ParticlesTemp ret(2);
  ret[0]=a;
  ret[1]=b;
  return ret;
}


void TypedPairScore::show(std::ostream &out) const
{
  out << "TypedPairScore with type key " << typekey_;
}

IMPCORE_END_NAMESPACE
