/**
 *  \file ClosePairsScoreState.cpp
 *  \brief Keep track of the maximumimum change of a set of attributes.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */


#include <IMP/core/ClosePairsScoreState.h>
#include <IMP/core/GridClosePairsFinder.h>
#include <IMP/core/BoxSweepClosePairsFinder.h>

#include <algorithm>
#include <sstream>

IMPCORE_BEGIN_NAMESPACE

ClosePairsScoreState::ClosePairsScoreState(SingletonContainer *pc,
                                     FilteredListPairContainer* out,
                                           FloatKey rk): in_(pc),
                                                         out_(out)
{
  rk_=rk;
  initialize();
}

ClosePairsScoreState::ClosePairsScoreState(SingletonContainer *pc,
                                           FloatKey rk): in_(pc)
{
  out_=new FilteredListPairContainer();
  rk_=rk;
  initialize();
}


void ClosePairsScoreState::initialize() {
  distance_=0;
  slack_=1;
#ifdef IMP_USE_CGAL
  set_close_pairs_finder(new BoxSweepClosePairsFinder());
#else
  set_close_pairs_finder(new GridClosePairsFinder());
#endif
}

void ClosePairsScoreState::set_distance(Float d) {
  distance_=d;
  f_->set_distance(distance_+slack_);
}

void ClosePairsScoreState::set_slack(Float d) {
  slack_=d;
  f_->set_distance(distance_+slack_);
}

void ClosePairsScoreState::set_singleton_container(SingletonContainer *pc) {
  // needs to be first for the case of assigning the pc that is already there
  in_=pc;
  xyzc_->set_singleton_container(in_);
  if (rc_) rc_->set_singleton_container(in_);
}

void ClosePairsScoreState::set_close_pairs_finder(ClosePairsFinder *f) {
  f_=f;
  f_->set_distance(distance_+slack_);
  f_->set_radius_key(rk_);
}

void ClosePairsScoreState::set_radius_key(FloatKey k) {
  rk_=k;
  rc_=NULL;
  xyzc_=NULL;
  f_->set_radius_key(rk_);
}


namespace {
  struct IsInactive {
    bool operator()(const ParticlePair &p) const {
      return !p[0]->get_is_active() || !p[1]->get_is_active();
    }
  };
}

void ClosePairsScoreState::do_before_evaluate()
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(in_);
  IMP_CHECK_OBJECT(out_);
  IMP_CHECK_OBJECT(f_);
  if (!xyzc_) {
    //std::cout << "Virgin ss" << std::endl;
    xyzc_ =new MaximumChangeScoreState(in_, XYZ::get_xyz_keys());
    if (rk_ != FloatKey()) {
      rc_= new MaximumChangeScoreState(in_, FloatKeys(1, rk_));
    }
    //std::cout << "adding pairs" << std::endl;
    unsigned int sz= out_->get_number_of_particle_pairs();
    out_->clear_particle_pairs();
    out_->reserve_particle_pairs(sz);
    f_->add_close_pairs(in_,out_);
    //std::cout << "done"<< std::endl;
    return;
  } else {
    xyzc_->before_evaluate(ScoreState::get_before_evaluate_iteration());
    if (rc_){
      rc_->before_evaluate(ScoreState::get_before_evaluate_iteration());
    }
    Float delta= xyzc_->get_maximum_change()
      + (rc_ ? rc_->get_maximum_change(): 0);
    if (delta*2 > slack_) {
      unsigned int sz= out_->get_number_of_particle_pairs();
      out_->clear_particle_pairs();
      out_->reserve_particle_pairs(sz);
      f_->add_close_pairs(in_, out_);
      xyzc_->reset();
      if (rc_) {
        rc_->reset();
      }
    } else {
      out_->remove_particle_pairs_if(IsInactive());
    }
  }
}
void ClosePairsScoreState::do_after_evaluate(DerivativeAccumulator*){
}


void ClosePairsScoreState::show(std::ostream &out) const
{
  out << "ClosePairsScoreState" << std::endl;
}

IMPCORE_END_NAMESPACE
