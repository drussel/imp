/**
 *  \file domino/DominoSampler.h
 *  \brief A beyesian infererence-based sampler.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/domino/assignment_containers.h>
#include <IMP/domino/Subset.h>
#include <IMP/domino/utility.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

IMPDOMINO_BEGIN_NAMESPACE

AssignmentContainer::AssignmentContainer(std::string name): Object(name){}

AssignmentContainer::~AssignmentContainer(){}

ListAssignmentContainer::ListAssignmentContainer(std::string name):
  AssignmentContainer(name){}


void ListAssignmentContainer::do_show(std::ostream &out) const {
  out << "size: " << get_number_of_assignments() << std::endl;
}



PackedAssignmentContainer::PackedAssignmentContainer(std::string name):
  AssignmentContainer(name), width_(-1){}


void PackedAssignmentContainer::do_show(std::ostream &out) const {
  out << "size: " << get_number_of_assignments() << std::endl;
  out << "width: " << width_ << std::endl;
}



SampleAssignmentContainer::SampleAssignmentContainer(unsigned int k,
                                                     std::string name):
  AssignmentContainer(name), width_(-1), k_(k), i_(0), select_(0,1),
  place_(0, k_-1) {}


void SampleAssignmentContainer::do_show(std::ostream &out) const {
  out << "size: " << get_number_of_assignments() << std::endl;
  out << "width: " << width_ << std::endl;
}

void SampleAssignmentContainer::add_assignment(const Assignment& a) {
  IMP_USAGE_CHECK(width_==-1 || static_cast<int>(a.size())== width_,
                  "Sizes don't match " << width_
                  << " vs " << a.size());
  if (width_==-1) {
    width_=a.size();
  }
  ++i_;
  if (get_number_of_assignments() < k_) {
    d_.insert(d_.end(), a.begin(), a.end());
  } else {
    double prob= static_cast<double>(k_)/i_;
    if (select_(random_number_generator) < prob) {
      int replace= place_(random_number_generator);
      std::copy(a.begin(), a.end(), d_.begin()+width_*replace);
    }
  }
}

Ints get_order(const Subset &s,
               const ParticlesTemp &all_particles);

#ifdef IMP_DOMINO_USE_IMP_RMF


WriteHDF5AssignmentContainer
::WriteHDF5AssignmentContainer(RMF::HDF5Group parent,
                               const Subset &s,
                               const ParticlesTemp &all_particles,
                               std::string name):
  AssignmentContainer(name), ds_(parent.add_child_index_data_set_2d(name)),
  order_(get_order(s, all_particles)),
  max_cache_(10000) {
  RMF::HDF5IndexDataSet2D::Index sz;
  sz[0]=0; sz[1]=s.size();
  ds_.set_size(sz);
}


WriteHDF5AssignmentContainer
::WriteHDF5AssignmentContainer(RMF::HDF5IndexDataSet2D dataset,
                          const Subset &s,
                          const ParticlesTemp &all_particles,
                          std::string name):
  AssignmentContainer(name), ds_(dataset),
  order_(get_order(s, all_particles)),
  max_cache_(10000) {
  if (ds_.get_size()[1] != s.size()) {
    RMF::HDF5IndexDataSet2D::Index sz;
    sz[0]=0; sz[1]=s.size();
    ds_.set_size(sz);
  }
}


unsigned int WriteHDF5AssignmentContainer::get_number_of_assignments() const {
  return ds_.get_size()[0]+cache_.size()/order_.size();
}

Assignment WriteHDF5AssignmentContainer::get_assignment(unsigned int) const {
  IMP_NOT_IMPLEMENTED;
  return Assignment();
}

void WriteHDF5AssignmentContainer::flush() {
  if (cache_.empty()) return;
  RMF::HDF5IndexDataSet2D::Index size= ds_.get_size();
  RMF::HDF5IndexDataSet2D::Index nsize=size;
  int num_items=cache_.size()/order_.size();
  IMP_LOG(VERBOSE, "Flushing cache of size "
          << num_items << " to disk"
          << std::endl);
  nsize[0]+= num_items;
  ds_.set_size(nsize);
  RMF::HDF5IndexDataSet2D::Index write_size;
  write_size[0]=num_items;
  write_size[1]=order_.size();
  size[1]=0;
  ds_.set_block(size, write_size, cache_);
  cache_.clear();
  cache_.reserve(max_cache_);
  IMP_IF_CHECK(USAGE_AND_INTERNAL) {
    unsigned int num=cache_.size()/order_.size();
    Assignments n(num);
    for (unsigned int i=0; i< num;++i) {
      n[i]=Assignment(cache_.begin()+i*order_.size(),
                      cache_.begin()+(i+1)*order_.size());
    }
    IMP_INTERNAL_CHECK(ds_.get_size()[0] >= num,
                       "Not enough on disk: " << ds_.get_size()[0]
                       << " vs " << num);
    for (unsigned int i=0; i< num; ++i) {
      Assignment read=get_assignment(get_number_of_assignments()-num+i);
      IMP_INTERNAL_CHECK(read==n[i], "Mismatch on read: " << read
                         << " vs " << n[i]);
    }
  }
}

void WriteHDF5AssignmentContainer::set_cache_size(unsigned int words) {
  max_cache_=words;
  if (cache_.size()>max_cache_) flush();
}

void WriteHDF5AssignmentContainer::add_assignment(const Assignment& a) {
  IMP_RMF_USAGE_CHECK(a.size()==order_.size(),
                      "Sizes don't match: " << a.size()
                      << " vs " << order_.size());
  Ints save= get_output(a, order_);
  cache_.insert(cache_.end(), save.begin(), save.end());
  if (cache_.size() > max_cache_) flush();
}




ReadHDF5AssignmentContainer
::ReadHDF5AssignmentContainer(RMF::HDF5IndexConstDataSet2D dataset,
                          const Subset &s,
                          const ParticlesTemp &all_particles,
                          std::string name):
  AssignmentContainer(name), ds_(dataset),
  order_(get_order(s, all_particles)),
  max_cache_(10000){
}


unsigned int ReadHDF5AssignmentContainer::get_number_of_assignments() const {
  return ds_.get_size()[0]+cache_.size()/order_.size();
}

Assignment ReadHDF5AssignmentContainer::get_assignment(unsigned int i) const {
  RMF::Ints is= ds_.get_row(RMF::HDF5DataSetIndexD<1>(i));
  Assignment ret(is.size());
  IMP_USAGE_CHECK(ret.size()== order_.size(), "Wrong size assignment");
  for (unsigned int i=0; i< ret.size(); ++i) {
    ret.set_item(order_[i], is[i]);
  }
  return ret;
}


void ReadHDF5AssignmentContainer::set_cache_size(unsigned int words) {
  max_cache_=words;
}

void ReadHDF5AssignmentContainer::add_assignment(const Assignment& ) {
  IMP_NOT_IMPLEMENTED;
}

#endif



WriteAssignmentContainer
::WriteAssignmentContainer(std::string dataset,
                          const Subset &s,
                          const ParticlesTemp &all_particles,
                          std::string name):
  AssignmentContainer(name),
  order_(get_order(s, all_particles)),
  max_cache_(10000) {
  cache_.reserve(max_cache_);
  f_=open(dataset.c_str(), O_WRONLY|O_APPEND|O_CREAT |O_TRUNC,
          S_IRUSR|S_IWUSR);
  number_=0;
}


unsigned int WriteAssignmentContainer::get_number_of_assignments() const {
  return number_;
}

Assignment WriteAssignmentContainer::get_assignment(unsigned int) const {
  IMP_NOT_IMPLEMENTED;
}

void WriteAssignmentContainer::flush() {
  if (cache_.empty()) return;
  int ret=write(f_, &cache_[0], cache_.size()*sizeof(int));
  IMP_USAGE_CHECK(ret == static_cast<int>(cache_.size()*sizeof(int)),
                  "Not everything written: " << ret
                  << " of " << cache_.size()*sizeof(int));
  cache_.clear();
  cache_.reserve(max_cache_);
}

void WriteAssignmentContainer::set_cache_size(unsigned int words) {
  max_cache_=words;
  if (cache_.size()>max_cache_) flush();
}

void WriteAssignmentContainer::add_assignment(const Assignment& a) {
  IMP_RMF_USAGE_CHECK(a.size()==order_.size(),
                      "Sizes don't match: " << a.size()
                      << " vs " << order_.size());
  Ints ret= get_output(a, order_);
  cache_.insert(cache_.end(), ret.begin(), ret.end());
  ++number_;
  IMP_LOG(VERBOSE, "Added " << a << " cache is now " << cache_
          << std::endl);
  if (cache_.size() > max_cache_) flush();
}


///

ReadAssignmentContainer
::ReadAssignmentContainer(std::string dataset,
                          const Subset &s,
                          const ParticlesTemp &all_particles,
                          std::string name):
  AssignmentContainer(name),
  order_(get_order(s, all_particles)),
  max_cache_(10000) {
  cache_.reserve(max_cache_);
  struct stat data;
  stat(dataset.c_str(), &data);
  size_=data.st_size/sizeof(int)/s.size();
  IMP_LOG(TERSE, "Opened binary file of size " << size_ << std::endl);
  f_=open(dataset.c_str(), O_RDONLY,0);
  offset_=-1;
}


unsigned int ReadAssignmentContainer::get_number_of_assignments() const {
  return size_;
}

Assignment ReadAssignmentContainer::get_assignment(unsigned int i) const {
  if (i < static_cast<unsigned int>(offset_)
      || (i+1-offset_)*order_.size() > cache_.size()) {
    cache_.resize(max_cache_);
    lseek(f_, i*sizeof(int)*order_.size(), SEEK_SET);
    int rd= read(f_, &cache_[0], max_cache_*sizeof(int));
    cache_.resize(rd/sizeof(int));
    offset_=i;
    IMP_LOG(VERBOSE, "Cache is " << cache_ << " at " << offset_
            << std::endl);
  }
  return get_from_output(cache_.begin()+i*order_.size(),
                  cache_.begin()+(i+1)*order_.size(),
                  order_);
}

void ReadAssignmentContainer::set_cache_size(unsigned int words) {
  max_cache_=words;
}

void ReadAssignmentContainer::add_assignment(const Assignment&) {
  IMP_NOT_IMPLEMENTED;
}





////////////////////////// RangeViewAssignmentContainer

inline unsigned int
RangeViewAssignmentContainer::get_number_of_assignments() const {
  return end_-begin_;
}

inline Assignment
RangeViewAssignmentContainer::get_assignment(unsigned int i) const {
  IMP_USAGE_CHECK(i < get_number_of_assignments(),
                  "Invalid assignment requested: " << i);
  return inner_->get_assignment(i+begin_);
}
RangeViewAssignmentContainer
::RangeViewAssignmentContainer(AssignmentContainer *inner,
                               unsigned int begin, unsigned int):
  AssignmentContainer("RangeViewAssignmentContainer%1%"),
  inner_(inner), begin_(begin),
  end_(std::min<unsigned int>(end_, inner->get_number_of_assignments())) {}

void RangeViewAssignmentContainer::do_show(std::ostream &out) const {
  out << "inner: " << inner_->get_name() << std::endl;
  out << "range: " << begin_ << "..." << end_ << std::endl;
}



void RangeViewAssignmentContainer::add_assignment(const Assignment&) {
  IMP_NOT_IMPLEMENTED;
}


////////////////////////// HEAP ASSIGNMENT CONTAINER

inline unsigned int
HeapAssignmentContainer::get_number_of_assignments() const {
  return d_.size();
}

inline Assignment
HeapAssignmentContainer::get_assignment(unsigned int i) const {
  IMP_USAGE_CHECK(i < get_number_of_assignments(),
                  "Invalid assignment requested: " << i);
  return d_[i].first;
}
HeapAssignmentContainer::HeapAssignmentContainer(
                               unsigned int k,
                               RestraintScoreSubsetFilter *rssf,
                               std::string name):
  AssignmentContainer(name), k_(k), rssf_(rssf) {}

void HeapAssignmentContainer::do_show(std::ostream &out) const {
  out << "number of assignments: " << get_number_of_assignments();
  out << ", max heap size: " << k_ << std::endl;
}



void HeapAssignmentContainer::add_assignment(const Assignment& a) {
  IMP_IF_CHECK(USAGE_AND_INTERNAL) {
    for (unsigned int i=0; i< get_number_of_assignments(); ++i) {
      IMP_INTERNAL_CHECK(get_assignment(i) != a,
                         "Assignment " << a << " already here.");
    }
  }
  //rssf_ may be null if no restraints are assigned to the particles
  double score=INT_MAX;
  if (rssf_){
    score=rssf_->get_score(a);
  }
  d_.push_back(AP(a,score));
  std::push_heap(d_.begin(), d_.end(), GreaterSecond());
  while (d_.size() > k_){
    std::pop_heap(d_.begin(), d_.end(),
                  GreaterSecond());
    d_.pop_back();
  }
  // if (d_.size()%1000000 == 0) {
  //   std::cout<<"Current subset size:"<<d_.size()<<" : "<<a<<std::endl;
  // }
}

//////////// CLUSTERED
ClusteredAssignmentContainer
::ClusteredAssignmentContainer(unsigned int k,
                               Subset s,
                               ParticleStatesTable *pst):
  k_(k), s_(s), pst_(pst), r_(0), metrics_(s.size()) {}


void ClusteredAssignmentContainer::add_metric(Particle *p,
                                              statistics::Metric *m) {
  for (unsigned int i=0; i< s_.size(); ++i) {
    if (s_[i]==p) {
      metrics_[i]=m;
    }
  }
}

inline unsigned int
ClusteredAssignmentContainer::get_number_of_assignments() const {
  return d_.size();
}

inline Assignment
ClusteredAssignmentContainer::get_assignment(unsigned int i) const {
  IMP_USAGE_CHECK(i < get_number_of_assignments(),
                  "Invalid assignment requested: " << i);
  return d_[i];
}

void ClusteredAssignmentContainer::do_show(std::ostream &out) const {
  out << "number of assignments: " << get_number_of_assignments();
  out << ", max size: " << k_ << std::endl;
}

bool ClusteredAssignmentContainer
::get_in_cluster(const Assignment &v) const {
  for (unsigned int i=0; i< d_.size(); ++i) {
    double dist=get_distance_if_smaller_than(s_, d_[i], v,
                                             pst_,
                                             metrics_, r_);
    if (dist < r_) {
      IMP_LOG(VERBOSE, v << " is in cluster with center " << d_[i]
              << " with radius " << r_ << std::endl);
      return true;
    }
  }
  return false;
}

double ClusteredAssignmentContainer::get_minimum_distance() const {
  double md= std::numeric_limits<double>::max();
  for (unsigned int i=0; i< d_.size(); ++i) {
    for (unsigned int j=0; j< i; ++j) {
      double cd= get_distance_if_smaller_than(s_, d_[i],
                                              d_[j],
                                              pst_,
                                              metrics_,
                                              md);
      if (cd <md) md=cd;
    }
  }
  return 2*md+.1;
}

void ClusteredAssignmentContainer::recluster() {
  IMP_LOG(VERBOSE, "Reclustering from " << d_ << std::endl);
  vector<Assignment> nd_;
  std::swap(nd_, d_);
  for (unsigned int i=0; i< nd_.size(); ++i) {
    if (!get_in_cluster(nd_[i])) {
      IMP_LOG(VERBOSE, "Adding state " << nd_[i] << std::endl);
      d_.push_back(nd_[i]);
    }
  }
  IMP_LOG(VERBOSE, "Reclustered to " << d_ << std::endl);
}


void ClusteredAssignmentContainer::add_assignment(const Assignment& a) {
  IMP_OBJECT_LOG;
  if (r_==0) {
    IMP_LOG(VERBOSE, "Adding state to list" << std::endl);
    d_.push_back(a);
  } else {
    IMP_INTERNAL_CHECK(r_ > 0,
                       "R is not initialized");
    if (get_in_cluster(a)) {
      IMP_LOG(VERBOSE, "State covered by existing cluster with radius " << r_
              << std::endl);
      return;
    } else {
      IMP_LOG(VERBOSE, "State added to new cluster "
              << std::endl);
      // perhaps update search structure
      d_.push_back(a);
    }
  }
  if (d_.size() >k_) {
    if (r_==0) {
      r_= get_minimum_distance();
      IMP_LOG(VERBOSE, "Initial distance is " << r_ << std::endl);
    } else {
      r_*=2;
    }
    recluster();
  }
}



IMPDOMINO_END_NAMESPACE
