/**
 *  \file RMF/Category.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include <RMF/FileConstHandle.h>
#include <RMF/internal/SharedData.h>

namespace RMF {

FileConstHandle::FileConstHandle(internal::SharedData *shared):
    shared_(shared) {}

  // \exception RMF::IOException couldn't create file,
  //                             or if unsupported file format
  FileConstHandle::FileConstHandle(std::string name):
    shared_(internal::create_read_only_shared_data(name))  {
}

NodeConstHandle FileConstHandle::get_node_from_id(NodeID id) const {
  return NodeConstHandle(id.get_index(), shared_.get());
}


std::string FileConstHandle::get_description() const {
  return shared_->get_description();
}

void FileConstHandle::flush() {
  shared_->flush();
}



Floats get_values(const NodeConstHandles &nodes,
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

  FileConstHandle open_rmf_file_read_only(std::string path) {
    return FileConstHandle(path);
  }


  BondPairs FileConstHandle::get_bonds()const {
    NodePairConstHandles nhs= get_node_pairs();
    BondPairs ret;
    for (unsigned int i=0; i< nhs.size(); ++i) {
      if (nhs[i].get_type()==BOND) {
        ret.push_back(BondPair(nhs[i].get_node(0),
                               nhs[i].get_node(1)));
      }
    }
    return ret;
  }

void FileConstHandle::validate(std::ostream &out=std::cerr) const {
  get_shared_data()->validate(out);
}

} /* namespace RMF */
