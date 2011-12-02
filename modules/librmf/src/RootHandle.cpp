/**
 *  \file RMF/Category.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <RMF/RootHandle.h>

namespace RMF {

RootHandle::RootHandle(internal::SharedData *shared): NodeHandle(0, shared) {}

RootHandle::RootHandle(HDF5Group root, std::string name, bool create):
    NodeHandle( 0, new internal::SharedData(root, name, create))  {
}

NodeHandle RootHandle::get_node_handle_from_id(NodeID id) const {
  return NodeHandle(id.get_index(), shared_.get());
}


NodeHandle RootHandle::get_node_handle_from_association(void*d) const {
  if (! shared_->get_has_association(d)) {
    return NodeHandle();
  } else {
    return NodeHandle(shared_->get_association(d), shared_.get());
  }
}

std::string RootHandle::get_description() const {
  return shared_->get_group().get_char_attribute("description");
}
void RootHandle::set_description(std::string descr) {
  IMP_RMF_USAGE_CHECK(descr.empty()
                      || descr[descr.size()-1]=='\n',
                      "Description should end in a newline.");
  shared_->get_group().set_char_attribute("description", descr);
}

vector<std::pair<NodeHandle, NodeHandle> > RootHandle::get_bonds() const {
  vector<std::pair<NodeHandle, NodeHandle> > ret(get_number_of_bonds());
  for (unsigned int i=0; i< ret.size(); ++i) {
    ret[i]= get_bond(i);
  }
  return ret;
}


void RootHandle::flush() {
  shared_->flush();
}



Floats get_values(const NodeHandles &nodes,
                  FloatKey k,
                  unsigned int frame,
                  Float missing_value) {
  Floats ret(nodes.size(), missing_value);
  for (unsigned int i=0; i< nodes.size(); ++i) {
    if (nodes[i].get_has_value(k, frame)) {
      ret[i]=nodes[i].get_value(k, frame);
    }
  }
  return ret;
}



} /* namespace RMF */
