/**
 *  \file Model.cpp \brief Storage of a model, its restraints,
 *                         constraints and particles.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/Model.h"
#include "IMP/Particle.h"
#include "IMP/internal/scoring_functions.h"
#include "IMP/compatibility/set.h"

IMP_BEGIN_NAMESPACE


//! Constructor
Model::Model(std::string name):
  RestraintSet(name)
{
  cur_stage_=internal::NOT_EVALUATING;
  gather_statistics_=false;
  eval_count_=0;
  set_was_used(true);
  first_call_=true;
  next_particle_=0;
  dependencies_dirty_=false;
#if IMP_BUILD < IMP_FAST
  internal::FloatAttributeTable::set_masks(&this->Masks::read_mask_,
                                           &this->Masks::write_mask_,
                                           &this->Masks::add_remove_mask_,
                                           &this->Masks::read_derivatives_mask_,
                                           &this->Masks::write_derivatives_mask_
                                           );
  internal::StringAttributeTable::set_masks(&this->Masks::read_mask_,
                                            &this->Masks::write_mask_,
                                            &this->Masks::add_remove_mask_);
  internal::IntAttributeTable::set_masks(&this->Masks::read_mask_,
                                         &this->Masks::write_mask_,
                                         &this->Masks::add_remove_mask_);
  internal::ObjectAttributeTable::set_masks(&this->Masks::read_mask_,
                                  &this->internal::Masks::write_mask_,
                                  &this->Masks::add_remove_mask_);
  internal::IntsAttributeTable::set_masks(&this->Masks::read_mask_,
                                          &this->Masks::write_mask_,
                                          &this->Masks::add_remove_mask_);
  internal::ObjectsAttributeTable::set_masks(&this->Masks::read_mask_,
                                             &this->Masks::write_mask_,
                                             &this->Masks::add_remove_mask_);
  internal::ParticleAttributeTable::set_masks(&this->Masks::read_mask_,
                                              &this->Masks::write_mask_,
                                              &this->Masks::add_remove_mask_);
  internal::ParticlesAttributeTable::set_masks(&this->Masks::read_mask_,
                                               &this->Masks::write_mask_,
                                               &this->Masks::add_remove_mask_);
#endif
  // be careful as this calls back to model
  RestraintSet::set_model(this);
}


void Model::cleanup()
{
  IMP_CHECK_OBJECT(this);
  {
    ScoreStates rs(score_states_begin(), score_states_end());
    for (unsigned int i=0; i < rs.size(); ++i) {
      rs[i]->set_model(nullptr);
    }
  }
}

IMP_LIST_ACTION_IMPL(Model, ScoreState, ScoreStates, score_state,
                     score_states, ScoreState*,
                     ScoreStates);

void Model::set_score_state_model(ScoreState *ss, Model *model) {
  IMP_CHECK_OBJECT(ss);
  if (model) {
    IMP_CHECK_OBJECT(model);
  }
  ss->set_model(model);
}
 RestraintSet *Model::get_root_restraint_set() {
  return this;
}

 double Model::get_maximum_score(Restraint *r) const {
  return r->get_maximum_score();
}
 void Model::set_maximum_score(Restraint *r, double s) {
  r->set_maximum_score(s);
}
 void Model::set_maximum_score(double s) {
  return RestraintSet::set_maximum_score(s);
}
 double Model::get_maximum_score() const {
  return RestraintSet::get_maximum_score();
}


ParticlesTemp Model::get_particles() const {
  return ParticlesTemp(particles_begin(),
                       particles_end());
}


void Model::add_particle_internal(Particle *p, bool set_name) {
    IMP_CHECK_OBJECT(this);
    IMP_CHECK_OBJECT(p);
    p->set_was_used(true);
    ParticleIndex id;
    if (free_particles_.empty()){
      id= ParticleIndex(next_particle_);
      ++next_particle_;
    } else {
      id= free_particles_.back();
      free_particles_.pop_back();
    }
    p->id_=id;
    int maxp= std::max<unsigned int>(particle_index_.size(),
                       get_as_unsigned_int(p->id_)+1);
    particle_index_.resize(maxp);
    particle_index_[p->id_]=p;
    if (set_name) {
      std::ostringstream oss;
      oss << boost::format("P%1%")
        % id;
      p->set_name(oss.str());
    }
#if IMP_BUILD < IMP_FAST
    //xstd::cout << "Resizing to " << particle_index_.size() << std::endl;
    Masks::read_mask_.resize(particle_index_.size(), true);
    Masks::write_mask_.resize(particle_index_.size(), true);
    Masks::add_remove_mask_.resize(particle_index_.size(), true);
    Masks::read_derivatives_mask_.resize(particle_index_.size(), true);
    Masks::write_derivatives_mask_.resize(particle_index_.size(), true);
#endif
  }



void Model::update() {
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(this);
  if (!get_has_dependencies()) {
    compute_dependencies();
  }
  before_evaluate(ordered_score_states_);
}

void Model::show_it(std::ostream& out) const
{
  out << get_particles().size() << " particles" << std::endl;
  out << get_number_of_restraints() << " restraints" << std::endl;
  out << get_number_of_score_states() << " score states" << std::endl;

  out << std::endl;
  IMP_CHECK_OBJECT(this);
}

void Model::remove_particle(Particle *p) {
  ParticleIndex pi= p->get_index();
  free_particles_.push_back(pi);
  p->set_tracker(p, nullptr);
  particle_index_[pi]=nullptr;
  internal::FloatAttributeTable::clear_attributes(pi);
  internal::StringAttributeTable::clear_attributes(pi);
  internal::IntAttributeTable::clear_attributes(pi);
  internal::ObjectAttributeTable::clear_attributes(pi);
  internal::IntsAttributeTable::clear_attributes(pi);
  internal::ObjectsAttributeTable::clear_attributes(pi);
  internal::ParticleAttributeTable::clear_attributes(pi);
  internal::ParticlesAttributeTable::clear_attributes(pi);
  IMP_IF_CHECK(base::USAGE) {
    ParticlesTemp cp= get_particles();
    for (unsigned int i=0; i< particle_index_.size(); ++i) {
      if (particle_index_[ParticleIndex(i)]) {
        ParticleIndex cur(i);
        {
          ParticleKeys keys
              = internal::ParticleAttributeTable::get_attribute_keys(cur);
          for (unsigned int j=0; j< keys.size(); ++j) {
            if (get_has_attribute(keys[j], cur)) {
              IMP_USAGE_CHECK(get_attribute(keys[j], cur) != pi,
                              "There is still a reference to"
                              << " removed particle in"
                              " particle "
                              << particle_index_[ParticleIndex(i)]->get_name()
                              << " attribute "
                              << keys[j]);
            }
          }
        }
        {
          ParticlesKeys keys
              = internal::ParticlesAttributeTable::get_attribute_keys(cur);
          for (unsigned int j=0; j< keys.size(); ++j) {
            if (get_has_attribute(keys[j], cur)) {
              ParticleIndexes pis
                = get_attribute(keys[j], cur);
              for (unsigned int k=0; k< pis.size(); ++k) {
                IMP_USAGE_CHECK(pis[k] != pi,
                                "There is still a reference to "
                                << "removed particle in"
                                << " particle "
                                << particle_index_[ParticleIndex(i)]->get_name()
                                << " attribute "
                                << keys[j]);
              }
            }
          }
        }
      }
    }
  }
}


void Model::add_data(ModelKey mk, Object *o) {
  model_data_.resize(std::max<int>(mk.get_index()+1,
                                   model_data_.size()));
  model_data_[mk.get_index()]=o;
}
base::Object *Model::get_data(ModelKey mk) const {
  return model_data_[mk.get_index()].get();
}
void Model::remove_data(ModelKey mk) {
  model_data_[mk.get_index()]= nullptr;
}
bool Model::get_has_data(ModelKey mk) const {
  if (model_data_.size() > mk.get_index()) {
    return model_data_[mk.get_index()];
  } else {
    return false;
  }
}

Model::ParticleIterator Model::particles_begin() const {
  return ParticleIterator(NotNull(), particle_index_.begin(),
                          particle_index_.end());
}
Model::ParticleIterator Model::particles_end() const {
  return ParticleIterator(NotNull(), particle_index_.end(),
                          particle_index_.end());
}

RestraintsTemp Model::get_known_restraints() const {
  return RestraintsTemp(RestraintTracker::tracked_begin(),
                        RestraintTracker::tracked_end());
}

IMP_END_NAMESPACE
