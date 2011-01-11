/**
 *  \file DihedralRestraint.cpp \brief Dihedral restraint between four
 *                                     particles.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/DihedralRestraint.h>
#include <IMP/core/XYZ.h>
#include <IMP/core/internal/dihedral_helpers.h>
#include <IMP/algebra/Vector3D.h>

#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/log.h>

#include <boost/tuple/tuple.hpp>

IMPCORE_BEGIN_NAMESPACE

DihedralRestraint::DihedralRestraint(UnaryFunction* score_func,
                                     Particle* p1, Particle* p2, Particle* p3,
                                     Particle* p4)
{
  p_[0]=p1;
  p_[1]=p2;
  p_[2]=p3;
  p_[3]=p4;

  score_func_ = score_func;
}

//! Calculate the score for this dihedral restraint.
/** \param[in] accum If not NULL, use this object to accumulate partial first
                     derivatives.
    \return Current score.
 */
double
DihedralRestraint::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_CHECK_OBJECT(score_func_);
  XYZ d0 = XYZ::decorate_particle(p_[0]);
  XYZ d1 = XYZ::decorate_particle(p_[1]);
  XYZ d2 = XYZ::decorate_particle(p_[2]);
  XYZ d3 = XYZ::decorate_particle(p_[3]);

  Float score;

  if (accum) {
    algebra::VectorD<3> derv0, derv1, derv2, derv3;
    double angle = internal::dihedral(d0, d1, d2, d3, &derv0, &derv1,
                                      &derv2, &derv3);

    Float deriv;
    boost::tie(score, deriv) = score_func_->evaluate_with_derivative(angle);

    d0.add_to_derivatives(derv0 * deriv, *accum);
    d1.add_to_derivatives(derv1 * deriv, *accum);
    d2.add_to_derivatives(derv2 * deriv, *accum);
    d3.add_to_derivatives(derv3 * deriv, *accum);
  } else {
    double angle = internal::dihedral(d0, d1, d2, d3, NULL, NULL, NULL, NULL);
    score = score_func_->evaluate(angle);
  }
  return score;
}

ParticlesTemp DihedralRestraint::get_input_particles() const {
  ParticlesTemp ret(p_, p_+4);
  return ret;
}

ContainersTemp DihedralRestraint::get_input_containers() const {
  return ContainersTemp();
}


//! Show the current restraint.
/** \param[in] out Stream to send restraint description to.
 */
void DihedralRestraint::do_show(std::ostream& out) const
{
  out << "dihedral restraint:" << std::endl;

  get_version_info().show(out);
  out << "  particles: " << p_[0]->get_name();
  out << ", " << p_[1]->get_name();
  out << ", " << p_[2]->get_name();
  out << " and " << p_[3]->get_name();
  out << "  ";
  score_func_->show(out);
  out << std::endl;
}

IMPCORE_END_NAMESPACE
