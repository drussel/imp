/**
 *  \file isd/LogicalORRestraint.cpp
 *  \brief Restrain a list of particle pairs with a lognormal+ISPA.
 *  NOTE: for now, the derivatives are written to all variables.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/isd/LogicalORRestraint.h>
#include <math.h>

IMPISD_BEGIN_NAMESPACE

LogicalORRestraint::LogicalORRestraint(ISDRestraint *r0, ISDRestraint *r1) 
                                     : r0_(r0), r1_(r1) {}

                                          
/* Apply the restraint to two other restraints
 */
double
LogicalORRestraint::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  
  double p0 = r0_->get_probability();
  double p1 = r1_->get_probability();
  double score = -log(p0 + p1 - p0*p1);

  if (accum)
  {
    //IMP_NEW(DerivativeAccumulator, a0, (*accum,(p0 * (1 - p1))/score));
    //IMP_NEW(DerivativeAccumulator, a1, (*accum,(p1 * (1 - p0))/score));
    DerivativeAccumulator a0(*accum,(p0 * (1 - p1))/score);
    DerivativeAccumulator a1(*accum,(p1 * (1 - p0))/score);
    r0_->unprotected_evaluate(&a0);
    r1_->unprotected_evaluate(&a1);
  }  
  return score;
}


/* Return all particles whose attributes are read by the restraints. To
   do this, ask the pair score what particles it uses.*/
//ASK DANIEL
ParticlesTemp LogicalORRestraint::get_input_particles() const
{
  ParticlesTemp ret;
  return ret;
}

/* The only container used is pc_. */
ContainersTemp LogicalORRestraint::get_input_containers() const
{
  return ContainersTemp();
}

void LogicalORRestraint::do_show(std::ostream& out) const
{
  out << "restraint0= " <<  r0_->get_name() << std::endl;
  out << "restraint11= " << r1_->get_name() << std::endl;
}

IMPISD_END_NAMESPACE
