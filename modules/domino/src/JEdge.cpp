/**
 *  \file JEdge.cpp
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#include <IMP/domino/JEdge.h>

IMPDOMINO_BEGIN_NAMESPACE

JEdge::JEdge(JNode *source, JNode *target)
{
  if (source->get_node_index() < target->get_node_index()) {
    source_ = source;
    target_ = target;
  } else {
    target_ = source;
    source_ = target;
  }
  separators_ =   std::map<std::string, CombState *>();
  source_old_score_separators_ =   std::map<std::string, float>();
  target_old_score_separators_ =   std::map<std::string, float>();
  source_new_score_separators_ =   std::map<std::string, float>();
  target_new_score_separators_ =   std::map<std::string, float> ();
}

const std::map<std::string, float> * JEdge::get_old_separators(JNode *n) const
{
  if (source_->get_node_index() == n->get_node_index()) {
    return &source_old_score_separators_;
  }
  return &target_old_score_separators_;
}

const std::map<std::string, float> * JEdge::get_new_separators(JNode *n) const
{
  if (source_->get_node_index() == n->get_node_index()) {
    return &source_new_score_separators_;
  }
  return &target_new_score_separators_;
}

void JEdge::init_separators()
{
  //get the set of interacing particles
  Particles *intersection_set = new Particles();
  source_->get_intersection(*target_, *intersection_set);
  source_->populate_states_of_particles(intersection_set, &separators_);
  source_old_score_separators_ = std::map<std::string, float>();
  target_old_score_separators_ = std::map<std::string, float>();
  source_new_score_separators_ = std::map<std::string, float>();
  target_new_score_separators_ = std::map<std::string, float>();
  for (std::map<std::string, CombState *>::iterator e = separators_.begin();
       e != separators_.end(); e++) {
    source_old_score_separators_[e->first] = 0.0;
    target_old_score_separators_[e->first] = 0.0;
    source_new_score_separators_[e->first] = 0.0;
    target_new_score_separators_[e->first] = 0.0;
  }
  delete intersection_set;
}

void JEdge::min_marginalize(JNode *from_node, JNode *to_node)
{
  JNode *fn, *tn;
  std::map<std::string, float> *fnoss, *tnoss, *fnnss, *tnnss;
  fn = source_;
  tn = target_;

  fnoss = &source_old_score_separators_;
  tnoss = &target_old_score_separators_;

  fnnss = &source_new_score_separators_;
  tnnss = &target_new_score_separators_;

  if (!(source_->get_node_index() == from_node->get_node_index())) {
    tn = source_;
    fn = target_;
    tnoss = &source_old_score_separators_;
    fnoss = &target_old_score_separators_;
    tnnss = &source_new_score_separators_;
    fnnss = &target_new_score_separators_;
  }

  for (std::map<std::string, CombState *>::iterator e = separators_.begin();
       e != separators_.end(); e++) {

    std::cout<<e->first<<std::endl;
    e->second->show();

    std::vector<CombState *> min_p_all;
    //marginalize over all particles except for those that are part
    //of the separator.
    min_p_all = fn->min_marginalize(*(e->second));

    CombState *min_p = min_p_all[0];

    //      (*fnmp)[e->first]=min_p;
    IMP_LOG(VERBOSE,"JEdge::min_marginalize for separator : " << e->first
       <<  " : the optimal from combination is : " << min_p << std::endl);
    (*tnoss)[e->first] = (*tnnss)[e->first];

    (*tnnss)[e->first] = min_p->get_total_score();

    // I think that we should release min_p here - it was allocated
    //in min_marginalize
  }
}
CombState * JEdge::get_separator(const CombState &other_comb) const
{
  /*std::cout << " JEdge::get_separator an edge between nodes "
            << source_->get_node_index() << "  " << target_->get_node_index()
            << std::endl;*/
  std::string key = generate_key(other_comb);
  std::stringstream error_message;
  error_message << " JEdge::get_separator a combination with index  : "
                << key << " is not part of the edge separators" ;
  IMP_INTERNAL_CHECK(separators_.find(key) != separators_.end(),
                     error_message.str());
  return separators_.find(key)->second;
}

const std::string JEdge::generate_key(const CombState &other_comb) const
{
  Particles *intersection_set = new Particles();
  source_->get_intersection(*target_, *intersection_set);
  std::string key = other_comb.partial_key(intersection_set);
  delete (intersection_set);
  return key;
}

void JEdge::show(std::ostream& out) const
{
  out << "=========================JEdge between  "
      << source_->get_node_index();
  out << " and " << target_->get_node_index()
      << " separator keys and values : " << std::endl;
  for (std::map<std::string,
                CombState *>::const_iterator it = separators_.begin();
       it != separators_.end(); it++) {
    out << "(" << it->first << " , " << it->second->get_total_score() << ") ";
  }
  out << std::endl;
}
void JEdge::clear() {
  for(std::map<std::string, CombState *>::iterator it =  separators_.begin();
    it != separators_.end(); it++) {
    delete(it->second);
  }
  separators_.clear();
  source_old_score_separators_.clear();
  target_old_score_separators_.clear();
  source_new_score_separators_.clear();
  target_new_score_separators_.clear();
}

IMPDOMINO_END_NAMESPACE
