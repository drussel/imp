/**
 *  \file DihedralRestraint.h  \brief Dihedral restraint between four particles.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_DIHEDRAL_RESTRAINT_H
#define IMPCORE_DIHEDRAL_RESTRAINT_H

#include "config.h"

#include <IMP/UnaryFunction.h>
#include <IMP/Restraint.h>

IMPCORE_BEGIN_NAMESPACE

//! Dihedral restraint between four particles
class IMPCOREEXPORT DihedralRestraint : public Restraint
{
public:
  //! Create the dihedral restraint.
  /** \param[in] score_func Scoring function for the restraint.
      \param[in] p1 Pointer to first particle in dihedral restraint.
      \param[in] p2 Pointer to second particle in dihedral restraint.
      \param[in] p3 Pointer to third particle in dihedral restraint.
      \param[in] p4 Pointer to fourth particle in dihedral restraint.
   */
  DihedralRestraint(UnaryFunction* score_func,
                    Particle* p1, Particle* p2, Particle* p3, Particle *p4);

  IMP_RESTRAINT(DihedralRestraint, get_module_version_info())

private:
  IMP::internal::OwnerPointer<UnaryFunction> score_func_;
  IMP::internal::OwnerPointer<Particle> p_[4];
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_DIHEDRAL_RESTRAINT_H */
