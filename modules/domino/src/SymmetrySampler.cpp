/**
 * \file  SymmetrySampler.cpp
 * \brief Sample transformations of particles while preseving N-symmetry.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */
#include <IMP/domino/SymmetrySampler.h>
#include <IMP/atom/pdb.h>
#include <IMP/algebra/geometric_alignment.h>
#include <IMP/core/XYZ.h>
#include <IMP/core/Transform.h>
#include <IMP/utility.h>
IMPDOMINO_BEGIN_NAMESPACE
//  virtual void show(std::ostream& out = std::cout) const {}

//   //! Show the sampling space of a single particle
//   virtual void show_space(Particle *p,
//                       std::ostream& out = std::cout) const {}
SymmetrySampler::SymmetrySampler(
  container::ListSingletonContainer *ps,
  TransformationDiscreteSet *ts,
  const algebra::Cylinder3D &c) : cyl_(c) {
  ps_=ps;
  ts_=ts;
  for(unsigned int i=0;i<ps_->get_number_of_particles();i++){
    symm_deg_[ps_->get_particle(i)]=i;
  }
  //superpose the particles on the first one and use that as reference
  ref_[ps_->get_particle(0)]=algebra::get_identity_transformation_3d();
  std::vector<algebra::VectorD<3> > ref_positions;
  Particles ps1 =
    atom::get_by_type(atom::Hierarchy(ps_->get_particle(0)), atom::ATOM_TYPE);

  for(Particles::iterator it=ps1.begin();it!=ps1.end();it++) {
   ref_positions.push_back(core::XYZ::decorate_particle(*it).get_coordinates());
  }

  for(unsigned int i=1;i<ps_->get_number_of_particles();i++) {
    std::vector<algebra::VectorD<3> > other_positions;
    Particles ps2 =
      atom::get_by_type(atom::Hierarchy(ps_->get_particle(i)), atom::ATOM_TYPE);
    for(Particles::iterator it=ps2.begin();it!=ps2.end();it++) {
      other_positions.push_back(
         core::XYZ::decorate_particle(*it).get_coordinates());
    }
    ref_[ps_->get_particle(i)]=
      algebra::get_transformation_aligning_first_to_second(other_positions,
                                                           ref_positions);
  }
}

void SymmetrySampler::populate_states_of_particles(
   container::ListSingletonContainer* particles,
   Combinations *states) const {
  IMP_INTERNAL_CHECK(states != NULL,"the states should be initialized");
  IMP_LOG(VERBOSE,"SymmetrySampler:: start populaing states of particles");
  //std::cout<<"SymmetrySampler:: start populaing states of particles"
  // <<std::endl;
  //CombState *calc_state;
  int comb_size = particles->get_number_of_particles();
  std::vector<int> v_int(ts_->get_number_of_states());
  IMP_LOG(VERBOSE, "Combination size: " << comb_size <<
          " number of states: " << ts_->get_number_of_states());
  for (int i = 0; i < ts_->get_number_of_states(); ++i) {
    CombState *calc_state = new CombState();
    for (int j = 0; j < comb_size; j++) {
      calc_state->add_data_item(particles->get_particle(j),i);
    }
    (*states)[calc_state->get_partial_key(particles)]=calc_state;
  }
  IMP_LOG(VERBOSE,
          "SymmetrySampler:: end populaing states of particles"<<std::endl);
}
//TODO - consider keeping particles and not using get_leaves
void SymmetrySampler::reset_placement(const CombState *cs) {
  IMP_LOG(VERBOSE,"SymmetrySampler:: start reset placement"<<std::endl);
  Particle *p;
  for (CombData::const_iterator it = cs->get_data()->begin();
        it != cs->get_data()->end(); it++) {
    p = it->first;
    IMP_LOG_WRITE(VERBOSE,p->show(IMP_STREAM));
    for_each(core::get_leaves(atom::Hierarchy::decorate_particle(p)),
             SingletonFunctor(new core::Transform(ref_[p])));
    IMP_LOG(VERBOSE,"end loop iteration"<<std::endl);
  }
  IMP_LOG(VERBOSE,"SymmetrySampler:: end reset placement"<<std::endl);
}

//! Set the attributes of the particles in the combination to the states
//! indicated in the combination
void SymmetrySampler::move2state(const CombState *cs) {
  IMP_LOG(VERBOSE,"SymmetrySampler:: start moving to state"<<std::endl);
  //first move the atoms to their initial location
  reset_placement(cs);
  Particle *p;
  algebra::Transformation3D t;
  for (std::map<Particle *,unsigned int>::const_iterator
         it = cs->get_data()->begin();it != cs->get_data()->end(); it++) {
    p = it->first;
    t = ts_->get_transformation(it->second);
    double angle = 2.*PI/ps_->get_number_of_particles()*symm_deg_[p];
    // was algebra:.get_rotated(cyl_,angle).compose(t)
    algebra::Transformation3D tr
      =compose(algebra::get_rotation_about_axis(
                                     cyl_.get_segment().get_direction(),
                                                       angle),t);
    for_each(core::get_leaves(atom::Hierarchy::decorate_particle(p)),
             SingletonFunctor(new core::Transform(tr)));
    ref_[p]= compose(algebra::get_rotation_about_axis(
                      cyl_.get_segment().get_direction(),
                      angle),t).get_inverse();
 //    std::stringstream name;
//     name<<p->get_value(StringKey("name"))<<"__"<<cs->key()<<".pdb";
//     atom::write_pdb(atom::Hierarchy::decorate_particle(p),name.str());
  }
  IMP_LOG(VERBOSE,"SymmetrySampler:: end moving to state"<<std::endl);
}
IMPDOMINO_END_NAMESPACE
