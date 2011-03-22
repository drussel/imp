/**
 *  \file Molecule.h     \brief A decorator for Molecules.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_MOLECULE_H
#define IMPATOM_MOLECULE_H

#include "atom_config.h"
#include "atom_macros.h"
#include "Hierarchy.h"

#include <IMP/base_types.h>
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/Decorator.h>

IMPATOM_BEGIN_NAMESPACE


//! A decorator for a molecule.
/** */
class IMPATOMEXPORT Molecule: public Hierarchy
{
  static IntKey key();
public:
  IMP_DECORATOR(Molecule, Hierarchy);
  //! Add the required attributes to the particle and create a Molecule
  static Molecule setup_particle(Particle *p) {
    if (!Hierarchy::particle_is_instance(p)) {
      Hierarchy::setup_particle(p);
    }
    p->add_attribute(key(),1);
    return Molecule(p);
  }

  //! Copy data from the other Molecule to the particle p
  static Molecule setup_particle(Particle *p, Molecule) {
    return setup_particle(p);
  }

  static bool particle_is_instance(Particle *p) {
    return p->has_attribute(key());
  }
};

IMP_OUTPUT_OPERATOR(Molecule);

IMP_DECORATORS(Molecule,Molecules, Hierarchies);

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_MOLECULE_H */
