/**
 *  \file RemoveRigidMOtionOptimizerState.cpp
 *  \brief Maintains temperature during molecular dynamics by velocity scaling.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/atom/RemoveRigidMotionOptimizerState.h>
#include <IMP/atom/MolecularDynamics.h>
#include <IMP/atom/Mass.h>
#include <IMP/core/XYZ.h>

IMPATOM_BEGIN_NAMESPACE

RemoveRigidMotionOptimizerState::RemoveRigidMotionOptimizerState(
    const Particles &pis, unsigned skip_steps) :
    pis_(pis), skip_steps_(skip_steps),
    call_number_(0)
{
  vs_[0] = FloatKey("vx");
  vs_[1] = FloatKey("vy");
  vs_[2] = FloatKey("vz");
}

void RemoveRigidMotionOptimizerState::update()
{
  if (skip_steps_ == 0 || (call_number_ % skip_steps_) == 0) {
    remove_rigid_motion();
  }
  ++call_number_;
}

void RemoveRigidMotionOptimizerState::remove_rigid_motion() const
{
  remove_linear();
  remove_angular();
}



void RemoveRigidMotionOptimizerState::remove_linear() const
{

  Float cm[3];
  Float cm_mass = 0.;

  for (unsigned i = 0; i < 3; ++i) cm[i] = 0.;

  for (unsigned int i=0; i< pis_.size(); ++i) {
    Particle *p = pis_[i];

    Float mass =  Mass(p).get_mass();
    cm_mass +=mass;

    for (unsigned i = 0; i < 3; ++i) {
      Float velocity = p->get_value(vs_[i]);
      cm[i] += mass * velocity;
    }
  }

  for (unsigned int i=0; i< pis_.size(); ++i) {
    Particle *p = pis_[i];

    for (unsigned i = 0; i < 3; ++i) {
      Float velocity = p->get_value(vs_[i]);

      velocity -= cm[i]/cm_mass;
      p->set_value(vs_[i], velocity);
    }
  }
}

void RemoveRigidMotionOptimizerState::remove_angular() const
{

  Float x[3],vx[3],v[3],vl[3],oo[3];
  Float inertia[3][3];

  for (unsigned i = 0; i < 3; ++i) {
   vl[i] = 0.;
   for (unsigned j = 0; j < 3; ++j) {
    inertia[i][j] = 0.;
   }
  }

  for (unsigned int i=0; i< pis_.size(); ++i) {
    Particle *p = pis_[i];

    Float mass =  Mass(p).get_mass();

    for (unsigned i = 0; i < 3; ++i) {
      x[i]   = core::XYZ(p).get_coordinate(i);
      vx[i]  = p->get_value(vs_[i]);
    }

    v[0] = x[1]* vx[2] - x[2]*vx[1];
    v[1] = x[2]* vx[0] - x[0]*vx[2];
    v[2] = x[0]* vx[1] - x[1]*vx[0];

    for (unsigned i = 0; i < 3; ++i) vl[i] += v[i] * mass;

    for (unsigned i = 0; i < 3; ++i)
     for (unsigned j = 0; j < 3; ++j)
      inertia[i][j] -= mass * x[i] * x[j];
   }


   Float trace = inertia[0][0] + inertia[1][1] + inertia[2][2];
   for (unsigned i = 0; i < 3; ++i) inertia[i][i] -= trace;

   Float a = inertia[0][0];
   Float b = inertia[1][1];
   Float c = inertia[2][2];
   Float d = inertia[0][1];
   Float e = inertia[0][2];
   Float f = inertia[1][2];
   Float o = vl[0];
   Float r = vl[1];
   Float q = vl[2];

   Float af_de = a*f-d*e;
   Float aq_eo = a*q-e*o;
   Float ab_dd = a*b-d*d;
   Float ac_ee = a*c-e*e;

   oo[2] = (af_de*(a*r-d*o)-ab_dd*aq_eo) / (af_de*af_de-ab_dd*ac_ee);
   oo[1] = (aq_eo - oo[2]*ac_ee)/af_de;
   oo[0] = (o - d*oo[1] - e*oo[2])/a;

   for (unsigned int i=0; i< pis_.size(); ++i) {
     Particle *p = pis_[i];

    for (unsigned i = 0; i < 3; ++i) {
      x[i]   = core::XYZ(p).get_coordinate(i);
      vx[i]  = p->get_value(vs_[i]);
    }

    v[0] = oo[1]* x[2] - oo[2]*x[1];
    v[1] = oo[2]* x[0] - oo[0]*x[2];
    v[2] = oo[0]* x[1] - oo[1]*x[0];

    for (int i = 0; i < 3; ++i) {
      vx[i]  -= v[i];
      p->set_value(vs_[i], vx[i]);
    }
   }
}
void RemoveRigidMotionOptimizerState::do_show(std::ostream &out) const
{
  out << "Remove net rigid velocity every "
      << skip_steps_ << " steps" << std::endl;
}

IMPATOM_END_NAMESPACE
