/**
 *  \file MovedSingletonContainer.cpp
 *  \brief Keep track of the maximumimum change of a set of attributes.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */


#include <IMP/core/internal/MovedSingletonContainer.h>
#include <IMP/core/XYZR.h>
#include <IMP/internal/utility.h>
#include <IMP/SingletonModifier.h>
#include <IMP/PairModifier.h>
#include <IMP/internal/InternalListSingletonContainer.h>
#include <IMP/SingletonScore.h>
#include <IMP/PairScore.h>
#include <IMP/internal/singleton_helpers.h>
#include <IMP/base/utility.h>

#include <algorithm>
#include <sstream>

IMPCORE_BEGIN_INTERNAL_NAMESPACE


MovedSingletonContainer::MovedSingletonContainer(SingletonContainer *pc,
                                                 double threshold):
  IMP::internal::ListLikeSingletonContainer(pc->get_model(),
                                       "MovedSingletonContainer%1%"),
  threshold_(threshold),
  pc_(pc)
{
  initialize_active_container(pc->get_model());
  first_call_=true;
  reset_all_=false;
  reset_moved_=false;
}


void MovedSingletonContainer::do_show(std::ostream &) const
{
}


void MovedSingletonContainer::do_after_evaluate() {
  IMP::internal::ListLikeSingletonContainer::do_after_evaluate();
  if (reset_all_) {
    do_reset_all();
    ParticleIndexes t;
    update_list(t);
  } else if (reset_moved_) {
    do_reset_moved();
    ParticleIndexes t;
    update_list(t);
  }
  reset_moved_=false;
  reset_all_=false;
  IMP_IF_CHECK(USAGE_AND_INTERNAL) {
    validate();
  }
}

void MovedSingletonContainer::do_before_evaluate()
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(pc_);
  if (first_call_ || pc_->get_contents_changed()) {
    IMP_LOG(TERSE, "First call" << std::endl);
    initialize();
    first_call_=false;
  } else {
    ParticleIndexes mved= do_get_moved();
    IMP_LOG(TERSE, "Adding to moved list: " << Showable(mved) << std::endl);
    add_to_list(mved);
  }
  IMP_IF_CHECK(USAGE_AND_INTERNAL) {
    validate();
  }
}


ParticlesTemp MovedSingletonContainer::get_state_input_particles() const {
  return pc_->get_particles();
}

ContainersTemp MovedSingletonContainer::get_state_input_containers() const {
  return ContainersTemp(1, pc_);
}

void MovedSingletonContainer::reset()
{
  reset_all_=true;
}

void MovedSingletonContainer::initialize()
{
  ParticleIndexes pt=do_initialize();
  update_list(pt);
}


void MovedSingletonContainer::reset_moved()
{
  reset_moved_=true;
}

void MovedSingletonContainer::set_threshold(double d) {
  threshold_= d;
  // could be more efficient, but why bother
  reset();
}

IMP_ACTIVE_CONTAINER_DEF(MovedSingletonContainer,);



void XYZRMovedSingletonContainer::validate() const {
  IMP_OBJECT_LOG;
  ParticleIndexes pis= get_singleton_container()->get_indexes();
  IMP_USAGE_CHECK(pis.size()==backup_.size(),
                  "Backup is not the right size");
  //using IMP::core::operator<<;
  // not sure why I need "Showable" all of a sudden, but I do
  for (unsigned int i=0; i< moved_.size(); ++i) {
    Particle *p=get_singleton_container()->get_particle(moved_[i]);
    IMP_USAGE_CHECK(get_contains(p),
                    "Particle in moved list but not in contents");
  }
}

void XYZRMovedSingletonContainer::do_reset_all() {
  IMP_OBJECT_LOG;
  backup_.clear();
  moved_.clear();
  //backup_.resize(get_singleton_container()->get_number_of_particles());
  IMP_FOREACH_SINGLETON(get_singleton_container(),{
      backup_.push_back(XYZR(_1).get_sphere());
    });
}
ParticleIndexes XYZRMovedSingletonContainer::do_initialize() {
  IMP_OBJECT_LOG;
  backup_.clear();
  moved_.clear();
  ParticleIndexes ret;
  //backup_.resize(get_singleton_container()->get_number_of_particles());
  IMP_FOREACH_SINGLETON_INDEX(get_singleton_container(),{
      backup_.push_back(XYZR(get_model(), _1).get_sphere());
      moved_.push_back(_2);
      ret.push_back(_1);
    });
  return ret;
}

void XYZRMovedSingletonContainer::do_reset_moved() {
  IMP_OBJECT_LOG;
  std::sort(moved_.begin(), moved_.end());
  for (unsigned int i=0; i< moved_.size(); ++i) {
    // skip duplicates
    if (i > 0 && moved_[i-1]== moved_[i]) continue;
    backup_[moved_[i]]
        =XYZR(get_singleton_container()->get_particle(moved_[i])).get_sphere();
  }
  moved_.clear();
}
ParticleIndexes XYZRMovedSingletonContainer::do_get_moved() {
    IMP_OBJECT_LOG;
    ParticleIndexes ret;
    Model *m= get_model();
    IMP_FOREACH_SINGLETON_INDEX(get_singleton_container(),{
        XYZR d(m, _1);
        double dr= std::abs(d.get_radius()- backup_[_2].get_radius());
        if (!algebra::get_interiors_intersect(
               algebra::Sphere3D(d.get_coordinates(),0),
               algebra::Sphere3D(backup_[_2].get_center(),
                                 std::max<double>(0,get_threshold()-dr)))) {
          ret.push_back(_1);
          moved_.push_back(_2);
        }
    });
    return ret;
  }

double XYZRMovedSingletonContainer
::do_get_distance_moved(unsigned int i) const {
  return (backup_[i].get_center()
          -core::XYZ(get_singleton_container()
                     ->get_particle(i)).get_coordinates()).get_magnitude()
    + std::abs(backup_[i].get_radius()
               - core::XYZR(get_singleton_container()
                            ->get_particle(i)).get_radius());
  }
XYZRMovedSingletonContainer
::XYZRMovedSingletonContainer(SingletonContainer *pc,
                              double threshold):
  MovedSingletonContainer(pc, threshold){
}


ParticleIndexes
XYZRMovedSingletonContainer::get_all_possible_indexes() const {
  return get_singleton_container()->get_all_possible_indexes();
}






void RigidMovedSingletonContainer::validate() const {
  IMP_OBJECT_LOG;
  normal_moved_->validate();
  IMP_USAGE_CHECK(rbs_.size()==rbs_backup_.size(),
                  "Backup is not the right size");
}

ParticleIndexes RigidMovedSingletonContainer::do_initialize() {
  IMP_OBJECT_LOG;
  ParticleIndexes normal;
  rbs_.clear();
  rbs_backup_.clear();
  rbs_members_.clear();
  int count=0;
  IMP_FOREACH_SINGLETON_INDEX(get_singleton_container(),
                        {
                          ++count;
                          if (core::RigidMember
                              ::particle_is_instance(get_model(),
                                                     _1)) {
                            core::RigidBody rb
                              = core::RigidMember(get_model(),
                                                  _1).get_rigid_body();
              if (rbs_members_.find(_1) == rbs_members_.end()) {
                rbs_.push_back(rb.get_particle_index());
                rbs_backup_.push_back(get_data(rb.get_particle_index()));
                rbs_moved_.push_back(_2);
              }
              rbs_members_[rb.get_particle_index()].push_back(_1);
         } else {
           normal.push_back(_1);
         }
                        });
  normal_->set_particles(normal);
  normal_moved_->initialize();
  //backup_.clear();
  rbs_backup_.resize(count);
  return get_singleton_container()->get_indexes();
}

void RigidMovedSingletonContainer::do_reset_all() {
  IMP_OBJECT_LOG;
  ParticleIndexes normal;
  rbs_.clear();
  rbs_backup_.clear();
  rbs_members_.clear();
  int count=0;
  IMP_FOREACH_SINGLETON_INDEX(get_singleton_container(),
                        {
                          ++count;
                          if (core::RigidMember
                              ::particle_is_instance(get_model(),
                                                     _1)) {
                            core::RigidBody rb
                              = core::RigidMember(get_model(),
                                                  _1).get_rigid_body();
              if (rbs_members_.find(_1) == rbs_members_.end()) {
                rbs_.push_back(rb.get_particle_index());
                rbs_backup_.push_back(get_data(rb.get_particle_index()));
              }
              rbs_members_[rb.get_particle_index()].push_back(_1);
         } else {
           normal.push_back(_1);
         }
                        });
  normal_->set_particles(normal);
  normal_moved_->reset();
  //backup_.clear();
  rbs_backup_.resize(count);
}
void RigidMovedSingletonContainer::do_reset_moved() {
  IMP_OBJECT_LOG;
  normal_moved_->reset_moved();
  for (unsigned int i=0; i< rbs_moved_.size(); ++i) {
    rbs_backup_[rbs_moved_[i]]=get_data(rbs_[rbs_moved_[i]]);
  }
  rbs_moved_.clear();
}


ParticleIndexes RigidMovedSingletonContainer::do_get_moved() {
    IMP_OBJECT_LOG;
    ParticleIndexes ret= normal_moved_->get_indexes();
    for (unsigned int i=0; i< rbs_.size(); ++i) {
      RigidBody rb(get_model(), rbs_[i]);
      if (get_distance_estimate(rbs_[i]) > get_threshold()) {
        ret.insert(ret.end(), rbs_members_[rbs_[i]].begin(),
                   rbs_members_[rbs_[i]].end());
        rbs_moved_.push_back(i);
      }
    }
    return ret;
  }

double RigidMovedSingletonContainer
::do_get_distance_moved(unsigned int i) const {
  Particle *p=get_singleton_container()->get_particle(i);
  if (core::RigidMember::particle_is_instance(p)) {
    core::RigidBody rb = core::RigidMember(p).get_rigid_body();
    return get_distance_estimate(rb.get_particle_index());
  } else {
    return normal_moved_->get_distance_moved(p);
  }
}
RigidMovedSingletonContainer
::RigidMovedSingletonContainer(SingletonContainer *pc,
                               double threshold):
  MovedSingletonContainer(pc, threshold){
  normal_= new IMP::internal::InternalListSingletonContainer(pc->get_model(),
                                          "Non-rigid particles %1%");
  normal_moved_= new XYZRMovedSingletonContainer(normal_, threshold);
}



ParticlesTemp RigidMovedSingletonContainer
::get_state_input_particles() const {
  ParticlesTemp ret
    = MovedSingletonContainer::get_singleton_container()
    ->get_particles();
  int sz= ret.size();
  for (int i=0; i< sz; ++i) {
    if (RigidMember::particle_is_instance(ret[i])) {
      ret.push_back(RigidMember(ret[i]).get_rigid_body());
    }
  }
  return ret;
}

ContainersTemp RigidMovedSingletonContainer
::get_state_input_containers() const {
  ContainersTemp ret;
  ret.push_back(get_singleton_container());
  ret.push_back(normal_);
  ret.push_back(normal_moved_);
  return ret;
}

ParticleIndexes
RigidMovedSingletonContainer::get_all_possible_indexes() const {
  ParticleIndexes ret= normal_moved_->get_all_possible_indexes();
  ret.insert(ret.end(), rbs_.begin(), rbs_.end());
  return ret;
}



IMPCORE_END_INTERNAL_NAMESPACE
