/**
 *  \file MaximumChangeXYZRScoreState.cpp
 *  \brief Keep track of the maximumimum change of a set of attributes.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */


#include <IMP/core/MaximumChangeXYZRScoreState.h>
#include <IMP/core/XYZR.h>
#include <IMP/internal/utility.h>

#include <algorithm>
#include <sstream>

IMPCORE_BEGIN_NAMESPACE

#define IMP_SIMPLE_SINGLETON_MODIFIER(Name, apply_expression)           \
  VersionInfo get_version_info() const {return VersionInfo();}          \
  void show(std::ostream &out= std::cout) const {out << "Nothing\n";}   \
  IMP_NO_SWIG(using SingletonModifier::apply);                          \
  void apply(Particle *p) const { apply_expression;}                    \
  void apply(Particle *, DerivativeAccumulator &) const {               \
    IMP_failure("Not implemented",ErrorException);}                     \
  void apply(const Particles &ps) const {                               \
    for (unsigned int i=0; i< ps.size(); ++i) {                         \
      Particle *p= const_cast<Particle*>(ps[i]);                        \
      Name::apply(p);                                                   \
    }                                                                   \
  }                                                                     \
  IMP_REF_COUNTED_DESTRUCTOR(Name);


namespace {

  class RecordXYZValues:public SingletonModifier {
    std::vector<algebra::Sphere3D> &values_;
    mutable int i_;
    FloatKey rk_;
  public:
    RecordXYZValues(std::vector<algebra::Sphere3D> &values,
                    FloatKey rk): values_(values), rk_(rk){
      i_=0;
    }
    IMP_SIMPLE_SINGLETON_MODIFIER(RecordXYZValues,
                                  {
                                    if (rk_!=FloatKey()) {
                                      XYZR d(p, rk_);
                                      values_[i_]= d.get_sphere();
                                    } else {
                                      XYZ d(p);
                                      values_[i_]
                                        = algebra::Sphere3D(d.get_coordinates(),
                                                            0);
                                    }
                                    ++i_;
                                  });
  };
  class CompareXYZValues:public SingletonModifier {
    std::vector<algebra::Sphere3D> &values_;
    mutable double change_;
    mutable int i_;
    FloatKey rk_;
  public:
    CompareXYZValues(std::vector<algebra::Sphere3D> &values,
                     FloatKey rk): values_(values), rk_(rk) {
      change_=0;
      i_=0;
    }
    double get_change() const {return change_;}
    IMP_SIMPLE_SINGLETON_MODIFIER(CompareXYZValues,
       {
         XYZ d(p);
         double lchange=std::abs(d.get_coordinate(0)
                                 - values_[i_].get_center()[0]);
         for (unsigned int i=1; i< 3; ++i) {
           lchange= std::max(lchange,
                             std::abs(d.get_coordinate(i)
                                      - values_[i_].get_center()[i]));
         }
         if (rk_!= FloatKey()) {
           XYZR d(p, rk_);
           change_=std::max(change_,
                            lchange
                            +std::abs(values_[i_].get_radius()-d.get_radius()));
         } else {
           change_= std::max(change_, lchange);
         }
         ++i_;
       });
  };
}

MaximumChangeXYZRScoreState::MaximumChangeXYZRScoreState(SingletonContainer *pc,
                                                         FloatKey rk):
  pc_(pc),
  rk_(rk)
{
  reset();
}

void MaximumChangeXYZRScoreState::do_before_evaluate()
{
  IMP_CHECK_OBJECT(pc_);
  if (rev_ != pc_->get_revision()) {
    reset();
    maximum_change_= std::numeric_limits<double>::max();
  } else {
    IMP_NEW(CompareXYZValues,  cv, (orig_values_, rk_));
    for (unsigned int i=0; i< pc_->get_number_of_particles(); ++i) {
      cv->apply(pc_->get_particle(i));
    }
    maximum_change_= cv->get_change();
  }
  IMP_LOG(TERSE, "MaximumChange update got " << maximum_change_ << std::endl);
}

void MaximumChangeXYZRScoreState::do_after_evaluate(DerivativeAccumulator *) {
}


void MaximumChangeXYZRScoreState::reset()
{
  maximum_change_=0;
  orig_values_.clear();
  orig_values_.resize(pc_->get_number_of_particles());
  IMP_NEW(RecordXYZValues, rv, (orig_values_, rk_));
  for (unsigned int i=0; i< pc_->get_number_of_particles(); ++i) {
    rv->apply(pc_->get_particle(i));
  }
  rev_=pc_->get_revision();
}

void MaximumChangeXYZRScoreState::show(std::ostream &out) const
{
  out << "MaximumChangeXYScoreState" << std::endl;
}



IMPCORE_END_NAMESPACE
