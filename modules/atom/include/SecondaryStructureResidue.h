/**
 *  \file IMP/atom/SecondaryStructureResidue.h
 *  \brief A decorator for SSE Residues.
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_SECONDARY_STRUCTURE_RESIDUE_H
#define IMPATOM_SECONDARY_STRUCTURE_RESIDUE_H

#include "atom_config.h"
#include "atom_macros.h"
#include "Hierarchy.h"

#include <IMP/base_types.h>
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/Decorator.h>

IMPATOM_BEGIN_NAMESPACE

//! A decorator for a residue with probability of secondary structure
/**
   Contains probabilities for each sse type (helix, strand, coil)
 */
class IMPATOMEXPORT SecondaryStructureResidue: public Decorator {
public:
  IMP_DECORATOR(SecondaryStructureResidue, Decorator);

  //! Set up SecondaryStructureResidue when you know all three probabilities
  static SecondaryStructureResidue setup_particle(Particle *res_p,
                                                  Float prob_helix,
                                                  Float prob_strand,
                                                  Float prob_coil){
    res_p->add_attribute(get_prob_helix_key(),prob_helix);
    res_p->add_attribute(get_prob_strand_key(),prob_strand);
    res_p->add_attribute(get_prob_coil_key(),prob_coil);
    if (!Hierarchy::particle_is_instance(res_p)) {
      Hierarchy::setup_particle(res_p);
    }
    SecondaryStructureResidue ssr(res_p);
    ssr.set_prob_helix(prob_helix);
    ssr.set_prob_strand(prob_strand);
    ssr.set_prob_coil(prob_coil);
    return ssr;
  }

  //! Set up SecondaryStructureResidue with default probabilities
  static SecondaryStructureResidue setup_particle(Particle *res_p){
    Float prob_helix=1.0/3.0,prob_strand=1.0/3.0,prob_coil=1.0/3.0;
    SecondaryStructureResidue ssr = setup_particle(res_p,
                                                   prob_helix,
                                                   prob_strand,
                                                   prob_coil);
    return ssr;
  }

  //! Return true if the particle is a secondary structure residue
  static bool particle_is_instance(Particle *p) {
    if (p->has_attribute(get_prob_helix_key())
        && (p->has_attribute(get_prob_strand_key()))
        && (p->has_attribute(get_prob_coil_key()))) return true;
    return false;
  }

  Particle* get_particle() const {
    return Decorator::get_particle();
  }

  //! Return all probabilities in one vector
  Floats get_all_probabilities(){
    Floats res;
    res.push_back(get_prob_helix());
    res.push_back(get_prob_strand());
    res.push_back(get_prob_coil());
    return res;
  }

  IMP_DECORATOR_GET_SET_OPT(prob_helix, get_prob_helix_key(),
                            Float, Float, 0.333);
  IMP_DECORATOR_GET_SET_OPT(prob_strand, get_prob_strand_key(),
                            Float, Float, 0.333);
  IMP_DECORATOR_GET_SET_OPT(prob_coil, get_prob_coil_key(),
                            Float, Float, 0.333);

  static FloatKey get_prob_helix_key();
  static FloatKey get_prob_strand_key();
  static FloatKey get_prob_coil_key();
};
IMP_DECORATORS(SecondaryStructureResidue, SecondaryStructureResidues,
               ParticlesTemp);

//! Makes a SecondaryStructureResidue with probabilities from a set
/** \param[in] ssr_ps The SSR-decorated particles to be combined
    \param[in] mdl The IMP Model
    \param[in] winner_takes_all_per_res Whether to set prob=1.0 for top
               scoring secondary structure type
 */
IMPATOMEXPORT
SecondaryStructureResidue get_coarse_ssr(const Particles &ssr_ps,
                                         Model *mdl,
                                         bool winner_takes_all_per_res=false);

//! Gets pearson correlation between SSE probs (higher is better match)
IMPATOMEXPORT
Float get_match_score(Particle * ssr1, Particle * ssr2);

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_SECONDARY_STRUCTURE_RESIDUE_H */
