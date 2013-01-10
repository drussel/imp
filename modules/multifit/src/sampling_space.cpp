/**
 * \file  sampling_space.cpp
 * \brief handles settting sampling space for different mappings
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#include <IMP/multifit/sampling_space.h>
#include <IMP/core/utility.h>
#include <IMP/core/XYZ.h>
#include <IMP/core/Hierarchy.h>
#include <IMP/atom/Hierarchy.h>
#include <IMP/container/ListSingletonContainer.h>
IMPMULTIFIT_BEGIN_NAMESPACE

domino1::TransformationMappedDiscreteSet*
 create_mapped_discrete_set(
  domino1::TransformationMappedDiscreteSet* full_smpl_space,
  const Particles &asmb_ap, const Particles &components,
  const Ints &mapping, Float dist_t) {

  Particles ps = full_smpl_space->get_particles();
  domino1::TransformationMappedDiscreteSet *discrete_set =
    new domino1::TransformationMappedDiscreteSet
    (new container::ListSingletonContainer(components));
  //add valid transformations for each particle
  Float dist;
  for(unsigned int i=0;i<components.size();i++) {
    IMP_LOG(VERBOSE,"working on component:" <<i<< std::endl);
    Particle *comp=components[i];
    algebra::Vector3D comp_cent=
    core::get_centroid(core::XYZs(core::get_leaves(atom::Hierarchy(comp))));
    algebra::Vector3D mapped_ap=
      core::XYZ(asmb_ap[mapping[i]]).get_coordinates();
    IMP_IF_LOG(VERBOSE) {
      IMP_LOG(VERBOSE,"component center:" <<std::endl);
      IMP_LOG_WRITE(VERBOSE,comp_cent.show());
      IMP_LOG(VERBOSE,std::endl<<"mapped anchor point center:" <<std::endl);
      IMP_LOG_WRITE(VERBOSE,mapped_ap.show());
    }
    for (int j=0;j<full_smpl_space->get_number_of_mapped_states(comp);j++){
      Particle *state_p=full_smpl_space->get_mapped_state(comp,j);
      algebra::Transformation3D t =
         domino1::Transformation(state_p).get_transformation();
      dist=algebra::get_distance(t.get_transformed(comp_cent),mapped_ap);
      IMP_LOG(VERBOSE,"for component:" <<i <<" trans: " << j<<
              " the distance is : " <<dist<<std::endl);
      if (dist<dist_t){
        discrete_set->add_mapped_state(comp,state_p);
      }
    }
    IMP_LOG(VERBOSE,"For component: " << i <<" " <<
            discrete_set->get_number_of_mapped_states(comp) <<
            " were selected out of " <<
            full_smpl_space->get_number_of_mapped_states(comp) << std::endl);
  }
  return discrete_set;
}
domino1::TransformationCartesianProductSampler*
 create_mapped_sampling_space(
  domino1::TransformationMappedDiscreteSet* full_smpl_space,
  const Particles &asmb_anchor_points, const Particles &components,
  const Ints &mapping, Float dist_t) {
  domino1::TransformationMappedDiscreteSet *tm =
    create_mapped_discrete_set(full_smpl_space,asmb_anchor_points,
                                 components,mapping,dist_t);
  return new domino1::TransformationCartesianProductSampler(tm,components,true);
}

IMPMULTIFIT_END_NAMESPACE
