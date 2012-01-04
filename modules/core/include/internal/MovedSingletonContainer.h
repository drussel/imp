/**
 *  \file MovedSingletonContainer.h
 *  \brief Keep track of the maximum change of a set of attributes.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_MOVED_SINGLETON_CONTAINER_H
#define IMPCORE_INTERNAL_MOVED_SINGLETON_CONTAINER_H

#include "../core_config.h"

#include <IMP/ScoreState.h>
#include <IMP/Particle.h>
#include <IMP/SingletonContainer.h>
#include <IMP/SingletonModifier.h>
#include "CoreListSingletonContainer.h"
#include <IMP/algebra/Sphere3D.h>
#include "../XYZR.h"
#include "../rigid_bodies.h"
#include "singleton_helpers.h"
#include <IMP/internal/container_helpers.h>
#include <vector>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

class IMPCOREEXPORT MovedSingletonContainer: public ListLikeSingletonContainer
{
 private:
  double threshold_;
  Pointer<SingletonContainer> pc_;
  bool first_call_;
  bool reset_all_;
  bool reset_moved_;
  IMP_ACTIVE_CONTAINER_DECL(MovedSingletonContainer);
  virtual ParticleIndexes do_get_moved()=0;
  virtual void do_reset_all()=0;
  virtual void do_reset_moved()=0;
  virtual ParticleIndexes do_initialize()=0;
  virtual double do_get_distance_moved(unsigned int i) const=0;
  using ListLikeSingletonContainer::update_list;
public:
  void initialize();
  virtual void validate() const=0;
  //! Track the changes with the specified keys.
  MovedSingletonContainer(SingletonContainer *pc,
                          double threshold);

  //! Measure differences from the current value.
  void reset();

  //! Reset the positions for the moved particles
  void reset_moved();

  //! Return the container storing the particles
  SingletonContainer *get_singleton_container() const {
    return pc_;
  }
  void set_threshold(double d);
  double get_threshold() const {
    return threshold_;
  }
  double get_distance_moved(Particle *p) const {
    IMP_FOREACH_SINGLETON_INDEX(pc_, {
        if (_1==p->get_index()) {
          return do_get_distance_moved(_2);
        }
      });
    return -1;
  }
#ifndef IMP_DOXYGEN
  bool get_is_up_to_date() const {
    if (get_model()->get_stage() != IMP::internal::NOT_EVALUATING) {
      return get_last_update_evaluation() == get_model()->get_evaluation();
    } else {
      return true;
    }
  }
  Restraints create_decomposition(SingletonScore *) const {
    IMP_NOT_IMPLEMENTED;
  }
#endif

  IMP_OBJECT(MovedSingletonContainer);
};

class IMPCOREEXPORT XYZRMovedSingletonContainer:
  public MovedSingletonContainer
{
  vector<algebra::Sphere3D> backup_;
  Ints moved_;
  virtual ParticleIndexes do_get_moved();
  virtual void do_reset_all();
  virtual void do_reset_moved();
  virtual ParticleIndexes do_initialize();
  virtual double do_get_distance_moved(unsigned int i) const;
public:
  virtual void validate() const;
  //! Track the changes with the specified keys.
  XYZRMovedSingletonContainer(SingletonContainer *pc,
                              double threshold);
  ParticleIndexes
    get_all_possible_indexes() const;
};


class IMPCOREEXPORT RigidMovedSingletonContainer:
  public MovedSingletonContainer
{
  IMP::OwnerPointer<CoreListSingletonContainer> normal_;
  IMP::OwnerPointer<XYZRMovedSingletonContainer> normal_moved_;
  vector<std::pair<algebra::Sphere3D, algebra::Rotation3D> > rbs_backup_;
  ParticleIndexes rbs_;
  Ints rbs_moved_;
  IMP::compatibility::map<ParticleIndex, ParticleIndexes> rbs_members_;
  virtual ParticleIndexes do_get_moved();
  virtual void do_reset_all();
  virtual void do_reset_moved();
  virtual ParticleIndexes do_initialize();
  virtual double do_get_distance_moved(unsigned int i) const;
  virtual void validate() const;
  double get_distance_estimate(ParticleIndex p) const {
    unsigned int i;
    for (i=0; i< rbs_.size(); ++i) {
      if (rbs_[i]==p) break;
    }
    core::XYZR xyz(get_model(), p);
    core::RigidBody rb(get_model(), p);
    double dr= std::abs(xyz.get_radius()- rbs_backup_[i].first.get_radius());
    double dx= (xyz.get_coordinates()
                -rbs_backup_[i].first.get_center()).get_magnitude();
    algebra::Rotation3D nrot=rb.get_reference_frame()
      .get_transformation_to().get_rotation();
    algebra::Rotation3D diffrot= rbs_backup_[i].second.get_inverse()*nrot;
    double angle= algebra::get_axis_and_angle(diffrot).second;
    double drot= std::abs(angle*xyz.get_radius()); // over estimate, but easy
    return dr+dx+drot;
  }
  std::pair<algebra::Sphere3D, algebra::Rotation3D>
    get_data(ParticleIndex p) const {
    return std::make_pair(core::XYZR(get_model(), p).get_sphere(),
        core::RigidBody(get_model(), p).get_reference_frame()
                          .get_transformation_to().get_rotation());
  }
  ContainersTemp get_state_input_containers() const;
  ParticlesTemp get_state_input_particles() const;
public:
  //! Track the changes with the specified keys.
  RigidMovedSingletonContainer(SingletonContainer *pc,
                               double threshold);
  ParticleIndexes
    get_all_possible_indexes() const;
};
IMP_OBJECTS(MovedSingletonContainer, MovedSingletonContainers);


IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_INTERNAL_MOVED_SINGLETON_CONTAINER_H */
