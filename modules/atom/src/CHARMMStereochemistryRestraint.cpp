/**
 * \file CHARMMStereochemistryRestraint.cpp
 * \brief Class to maintain CHARMM stereochemistry.
 *
 * Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/atom/CHARMMStereochemistryRestraint.h>
#include <IMP/core/Harmonic.h>

IMPATOM_BEGIN_NAMESPACE

CHARMMStereochemistryRestraint::CHARMMStereochemistryRestraint(Hierarchy h,
                                                   CHARMMTopology *topology)
{
  bonds_ = topology->add_bonds(h);
  angles_ = topology->get_parameters()->create_angles(bonds_);
  dihedrals_ = topology->get_parameters()->create_dihedrals(bonds_);
  impropers_ = topology->add_impropers(h);

  bond_score_ = new BondSingletonScore(new core::Harmonic(0., 1.));
  angle_score_ = new AngleSingletonScore(new core::Harmonic(0., 1.));
  dihedral_score_ = new DihedralSingletonScore();
  improper_score_ = new ImproperSingletonScore(new core::Harmonic(0., 1.));
}

double CHARMMStereochemistryRestraint::unprotected_evaluate(
                                         DerivativeAccumulator *accum) const
{
  double score = 0.;

  for (Particles::const_iterator b = bonds_.begin(); b != bonds_.end(); ++b) {
    score += bond_score_->evaluate(*b, accum);
  }
  for (Particles::const_iterator a = angles_.begin(); a != angles_.end(); ++a) {
    score += angle_score_->evaluate(*a, accum);
  }
  for (Particles::const_iterator d = dihedrals_.begin();
       d != dihedrals_.end(); ++d) {
    score += dihedral_score_->evaluate(*d, accum);
  }
  for (Particles::const_iterator i = impropers_.begin();
       i != impropers_.end(); ++i) {
    score += improper_score_->evaluate(*i, accum);
  }

  return score;
}

ParticlesTemp CHARMMStereochemistryRestraint::get_input_particles() const
{
  ParticlesTemp ps;
  for (Particles::const_iterator b = bonds_.begin(); b != bonds_.end(); ++b) {
    ps.push_back(*b);
    ParticlesTemp bps = bond_score_->get_input_particles(*b);
    ps.insert(ps.end(), bps.begin(), bps.end());
  }
  for (Particles::const_iterator a = angles_.begin();
       a != angles_.end(); ++a) {
    ps.push_back(*a);
    ParticlesTemp bps = angle_score_->get_input_particles(*a);
    ps.insert(ps.end(), bps.begin(), bps.end());
  }
  for (Particles::const_iterator d = dihedrals_.begin();
       d != dihedrals_.end(); ++d) {
    ps.push_back(*d);
    ParticlesTemp bps = dihedral_score_->get_input_particles(*d);
    ps.insert(ps.end(), bps.begin(), bps.end());
  }
  for (Particles::const_iterator i = impropers_.begin();
       i != impropers_.end(); ++i) {
    ps.push_back(*i);
    ParticlesTemp bps = improper_score_->get_input_particles(*i);
    ps.insert(ps.end(), bps.begin(), bps.end());
  }
  return ps;
}

ContainersTemp CHARMMStereochemistryRestraint::get_input_containers() const
{
  return ContainersTemp();
}

void CHARMMStereochemistryRestraint::do_show(std::ostream& out) const
{
  out << "CHARMMStereochemistryRestraint" << std::endl;
}

StereochemistryPairFilter *CHARMMStereochemistryRestraint::get_pair_filter()
{
  IMP_NEW(StereochemistryPairFilter, pf, ());
  pf->set_bonds(bonds_);
  pf->set_angles(angles_);
  pf->set_dihedrals(dihedrals_);
  return pf.release();
}

IMPATOM_END_NAMESPACE
