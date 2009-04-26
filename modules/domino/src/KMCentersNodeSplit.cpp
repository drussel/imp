/**
 * \file KMCenetersNodeSplit.cpp
 * \brief a split node in the kc-tree with two children
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/domino/KMCentersNodeSplit.h>
#include <IMP/domino/random_generator.h>
IMPDOMINO_BEGIN_NAMESPACE

void KMCentersNodeSplit::compute_sums() {
  n_data_=1;
  for(int i=0;i<2;i++) {
    children_[i]->compute_sums();
    const KMPoint *s_child = children_[i]->get_sums();
    for (int d = 0; d < bnd_box_.get_dim(); d++) {
      sum_[d] += (*s_child)[d];
    }
    sum_sq_ += children_[i]->get_sum_sq();
    n_data_ += children_[i]->get_subtree_size();
  }
}

KMPoint KMCentersNodeSplit::sample_center()
{
  int r = random_int(get_subtree_size());
  if (r == 0) {// sample from this node
    KMRectangle exp_box = bnd_box_.expand(3); // compute 3x expanded box
    return exp_box.sample();
  }
  else if (r <= children_[0]->get_subtree_size()) { // sample from left
    return children_[0]->sample_center();
  }
  else { // sample from right subtree
    return children_[1]->sample_center();
  }
}

void KMCentersNodeSplit::show(std::ostream &out) const {
  children_[1]->show();
  out << "    ";
  for (int i = 0; i < level_; i++) {
    out << ".";
  }
  out.precision(4);
  out << "Split" << " cd=" << cut_dim_ << " cv=" << std::setw(6) << cut_val_
      << " nd=" << n_data_ << " sm=";
  print_point(sum_, out);
  out << " ss=" << sum_sq_ << "\n";
  children_[0]->show();
}

void KMCentersNodeSplit::get_neighbors(const std::vector<int> &cands,
     KMPointArray *sums, KMPoint *sum_sqs,std::vector<int> *weights)
{
  if (cands.size() == 1) {
    // post points as neighbors
    post_neighbor(sums, sum_sqs, weights,cands[0]);
  }
  //get cloest candidate to the box represented by the node
  else {
    std::vector<int> new_cands;
    compute_close_centers(cands,&new_cands);
    children_[0]->get_neighbors(new_cands,sums,sum_sqs,weights);
    children_[1]->get_neighbors(new_cands,sums,sum_sqs,weights);
  }
}
void KMCentersNodeSplit::get_assignments(const std::vector<int> &cands,
 std::vector<int> &close_center){
  if (cands.size() == 1) {
    children_[0]->get_assignments(cands,close_center);
    children_[1]->get_assignments(cands,close_center);
  }
  else {// get closest cand to box
    std::vector<int> new_cands;
    compute_close_centers(cands,&new_cands);
    // apply to children
    children_[0]->get_assignments(new_cands,close_center);
    children_[1]->get_assignments(new_cands,close_center);
  }
}
IMPDOMINO_END_NAMESPACE
