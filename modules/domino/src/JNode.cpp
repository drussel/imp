/**
 *  \file JNode.cpp \brief Handles all functionalities of a junction tree node.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include <IMP/domino/JNode.h>

#include <numeric>
#include <climits>

IMPDOMINO_BEGIN_NAMESPACE

//TODO: for all the intersection operations the sets should be sorted,
//make sure that in the initialization of the nodes the number of
//nodes come sorted.
JNode::JNode(const Particles &p, int node_ind): ds_(NULL)
{
  node_ind_ = node_ind;
  //enter the particles by the order of their indexes.
  sorted_particle_indexes_ = std::vector<Int>();
  particles_ = Particles();
  for (Particles::const_iterator it = p.begin(); it != p.end(); it++) {
    particles_.push_back(*it);
    sorted_particle_indexes_.push_back((*it)->get_index().get_index());
  }
  std::sort(sorted_particle_indexes_.begin(), sorted_particle_indexes_.end());
  comb_states_ = std::map<std::string, CombState *>();
}

void JNode::init_sampling(const DiscreteSampler &ds)
{
  ds_ = &ds;
  populate_states_of_particles(particles_, &comb_states_);
}

CombState* JNode::get_state(unsigned int index, bool move2state_) const {
  //TODO - think about a better implementation
  std::map<std::string, CombState *>::const_iterator it = comb_states_.begin();
  for (unsigned int i = 0;i < index;i++) {
    ++it;
  }
  if (move2state_) {
    move2state(*(it->second));
  }
  return it->second;
}


void JNode::populate_states_of_particles(const Particles &particles,
                                         std::map<std::string,
                                         CombState *> *states_) const
{
  long num_states = number_of_states();
  long global_iterator, global_index;
  CombState *calc_state_;
  for (long state_index = 0;state_index < num_states; state_index++) {
    calc_state_ = new CombState();
    global_iterator = num_states;
    global_index = state_index;
    for (Particles::const_iterator it = particles.begin();
         it != particles.end(); it++) {
      Particle* p = *it;
      long sample_size = ds_->get_space_size(**it);
      global_iterator /= sample_size;
      calc_state_->add_data_item(p, global_index / global_iterator);
      global_index -= (global_index / global_iterator) * global_iterator;
    }
    (*states_)[calc_state_->partial_key(&particles)] = calc_state_;
  } // state_index iteration
}

void JNode::show_sampling_space(std::ostream& out) const
{
  out << std::endl << " sampling space of JNode with index: " << node_ind_;
  for (Particles::const_iterator pi = particles_.begin();
       pi != particles_.end(); pi++) {
    out << std::endl << " states for particle name : " <<
    (*pi)->get_value(IMP::StringKey("name")) << ":" << std::endl;
    ds_->show_space(**pi, out);
  }
}

void JNode::show(std::ostream& out) const
{
  out << "=========================JNode  " <<  " node_index: " << node_ind_
      << std::endl;
  out << "particle_indices: " << std::endl;
  for (Particles::const_iterator it = particles_.begin(); it !=
       particles_.end(); it++) {
    out << (*it)->get_value(IMP::StringKey("name")) << " || " ;
  }
  out << std::endl << "==combinations: " << std::endl;
  for (std::map<std::string, CombState *>::const_iterator it =
         comb_states_.begin(); it != comb_states_.end(); it++) {
    out << " (" << it->first << " , " << it->second->get_total_score() << ") ";
  }
  out << std::endl;
}
bool JNode::is_part(const Particles &p) const
{
  std::vector<IMP::Int> intersection, other_sorted_particle_indexes;
  for (Particles::const_iterator it = p.begin(); it != p.end(); it++) {
    other_sorted_particle_indexes.push_back((*it)->get_index().get_index());
  }
  sort(other_sorted_particle_indexes.begin(),
       other_sorted_particle_indexes.end());
  set_intersection(sorted_particle_indexes_.begin(),
                   sorted_particle_indexes_.end(),
                   other_sorted_particle_indexes.begin(),
                   other_sorted_particle_indexes.end(),
                   std::inserter(intersection, intersection.begin()));
  if (intersection.size() == p.size()) {
    return true;
  }
  return false;
}

void JNode::get_intersection(const JNode &other, Particles &in) const
{
  // since the list should be sorted we use the indexes and not the pointers,
  // as we can not predict the order of the pointers.
  std::vector<unsigned int> inter_indexes;
  set_intersection(sorted_particle_indexes_.begin(),
                   sorted_particle_indexes_.end(),
                   other.sorted_particle_indexes_.begin(),
                   other.sorted_particle_indexes_.end(),
                   std::inserter(inter_indexes, inter_indexes.begin()));
  //TODO - do this more efficient
  for (std::vector<unsigned int>::const_iterator it = inter_indexes.begin();
       it != inter_indexes.end(); it++) {
    for (Particles::const_iterator pi = particles_.begin();
         pi != particles_.end(); pi++) {
      if (*it == (*pi)->get_index().get_index()) {
        in.push_back(*pi);
      }
    }
  }
}

void JNode::move2state(const CombState &cs) const
{
  for (std::map<Particle *,
                unsigned int>::const_iterator it = cs.get_data().begin();
       it != cs.get_data().end(); it++) {
    Particle *p = it->first;
    for (unsigned int i = 0; i < ds_->get_number_of_attributes(*p); i++) {
      p->set_value(ds_->get_attribute(*p, i),
               ds_->get_state_val(*p, it->second, ds_->get_attribute(*p, i)));
    }
  }
}

void JNode::realize(Restraint *r, float weight)
{
  std::map<std::string, float> temp_calculations;
  // stores calculated discrete values. It might be that each appears more
  // than once, since the node may contain more particles than the ones
  // indicated by the restraint.
  std::map<std::string, float> result_cache; // to avoid multiple calculation
                                             // of the same configuration.
  float score;
  Particles r_particles;
  ParticlesList pl = r->get_interacting_particles();
  for(ParticlesList::iterator it1 = pl.begin();
      it1 != pl.end(); it1++){
    for(Particles::iterator it2 = it1->begin(); it2 != it1->end(); it2++) {
      r_particles.push_back(*it2);
     }
  }
  for (std::map<std::string, CombState *>::iterator it =  comb_states_.begin();
       it != comb_states_.end(); it++) {
    std::string partial_key = it->second->partial_key(&(r_particles));
    if (result_cache.find(partial_key) == result_cache.end()) {
      move2state(*(it->second));
      score = r->evaluate(NULL) * weight;
      result_cache[partial_key] = score;
    } else {
      score = result_cache.find(partial_key)->second;
    }
    it->second->update_total_score(0.0, score);
  }
}

//! Return the optimal score for the separator, for the given separator
//! find the optimal combination of the rest of the components.
std::vector<CombState *> JNode::min_marginalize(const CombState &s,
                                                bool move2state_)
{
  float min_score = INT_MAX;
  std::vector<CombState *> min_comb;
  min_comb = std::vector<CombState *>();
  for (std::map<std::string, CombState *>::iterator it =  comb_states_.begin();
       it != comb_states_.end(); it++) {
    if (it->second->is_part(s)) {
      // #TODO: too expensive , should be the other way around -
      // build all combinations according to separator.comb_key
      if (it->second->get_total_score() < min_score) {
        min_score = it->second->get_total_score();
      }
    }
  }

  for (std::map<std::string, CombState *>::iterator it = comb_states_.begin();
       it != comb_states_.end(); it++) {
    if (it->second->is_part(s)) {
      if (it->second->get_total_score() ==  min_score) {
        min_comb.push_back(it->second);
      }
    }
  }
  std::stringstream error_message;
  error_message << "JNode::min_marginalize couldn't marg over separator:";
  s.show(error_message);
  IMP_assert(min_score < INT_MAX, error_message.str());
  if (move2state_) {
    move2state(*(min_comb[0]));
  }
  return min_comb;
}

void JNode::update_potentials(
  const std::map<std::string, float> &old_score_separators,
  const std::map<std::string, float> &new_score_separators,
  const Particles &intersection_particles)
{
  for (std::map<std::string, CombState *>::iterator it = comb_states_.begin();
       it != comb_states_.end(); it++) {
    std::string s_key = it->second->partial_key(&intersection_particles);
    //separator_comb=v.comb_key.intersection_by_feature_key(edge_data.s_ij)
    // s_key=separator_comb.key()
    //TODO - add validation tests
    //check if the node is the from or to of the edge:
    it->second->update_total_score(old_score_separators.find(s_key)->second,
                                   new_score_separators.find(s_key)->second);
  }
}

std::vector<CombState *>* JNode::find_minimum(bool move2state_) const
{
  //find the value of the minimum
  float min_val = INT_MAX;
  for (std::map<std::string,CombState *>::const_iterator it=
    comb_states_.begin();it != comb_states_.end(); it++) {
    if (it->second->get_total_score() < min_val) {
      min_val = it->second->get_total_score();
    }
  }
  std::vector<CombState *>* min_combs = new std::vector<CombState *>;
  //iterate over all the nodes again and find all combinations that reach
  //the global minimum
  for (std::map<std::string,
                CombState *>::const_iterator it = comb_states_.begin();
       it != comb_states_.end(); it++) {
    if (it->second->get_total_score() ==  min_val) {
      min_combs->push_back(it->second);
    }
  }
  if (move2state_) {
    move2state(**(min_combs->begin()));
  }
  return min_combs;
}
void JNode::clear() {
  for(std::map<std::string, CombState *>::iterator it =  comb_states_.begin();
      it != comb_states_.end();it++) {
    delete(it->second);
  }
  comb_states_.clear();
  ds_=NULL;
}

IMPDOMINO_END_NAMESPACE
