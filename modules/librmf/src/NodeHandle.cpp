/**
 *  \file RMF/Category.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <RMF/NodeHandle.h>
#include <boost/tuple/tuple.hpp>
#include <RMF/Category.h>
#include <RMF/FileHandle.h>

namespace RMF {

NodeHandle::NodeHandle(int node, internal::SharedData *shared):
    NodeConstHandle(node, shared) {
}

NodeHandle NodeHandle::add_child(std::string name, NodeType t) {
  return NodeHandle(get_shared_data()->add_child(get_node_id(), name, t),
                    get_shared_data());
}


FileHandle NodeHandle::get_file() const {
  return FileHandle(get_shared_data());
}

vector<NodeHandle> NodeHandle::get_children() const {
  Ints children= get_shared_data()->get_children(get_node_id());
  vector<NodeHandle> ret(children.size());
  for (unsigned int i=0; i< ret.size(); ++i) {
    ret[i]= NodeHandle(children[ret.size()-i-1], get_shared_data());
  }
  return ret;
}


} /* namespace RMF */
