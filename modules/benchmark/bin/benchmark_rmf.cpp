/**
 * Copyright 2007-2011 IMP Inventors. All rights reserved.
 */
#include <IMP/benchmark/benchmark_config.h>
#ifdef IMP_BENCHMARK_USE_IMP_RMF
#include <RMF/RootHandle.h>
#include <IMP/internal/graph_utility.h>
#include <boost/timer.hpp>
#include <IMP/benchmark/utility.h>
#include <IMP/benchmark/benchmark_macros.h>
#include <sstream>

namespace {
  template <class T>
  double count(T t) {
    return t;
  }
  double count(std::string nm) {
    return nm.size();
  }
  double count(RMF::NodeID nm) {
    return nm.get_index();
  }

template <class TypeT, int Arity>
  double show_type_data_xml(RMF::NodeHandle nh,
                            RMF::Category kc) {
    double ret=0;
    RMF::RootHandle rh= nh.get_root_handle();
    std::vector< RMF::Key<TypeT, Arity> > keys= rh.get_keys<TypeT, Arity>(kc);
    for (unsigned int i=0; i< keys.size(); ++i) {
      if (nh.get_has_value(keys[i])) {
        ret+= count(nh.get_value(keys[i]));
        ret+= rh.get_name(keys[i]).size();
      }
    }
    return ret;
  }
  double show_data_xml(RMF::NodeHandle nh,
                     RMF::Category kc) {
    double ret=0;
    ret+=show_type_data_xml< RMF::IntTraits, 1>(nh, kc);
    ret+=show_type_data_xml< RMF::FloatTraits, 1>(nh, kc);
    ret+=show_type_data_xml< RMF::IndexTraits, 1>(nh, kc);
    ret+=show_type_data_xml< RMF::StringTraits, 1>(nh, kc);
    ret+=show_type_data_xml< RMF::NodeIDTraits, 1>(nh, kc);
    ret+=show_type_data_xml< RMF::IndexDataSet2DTraits, 1>(nh, kc);
    ret+=show_type_data_xml< RMF::FloatDataSet2DTraits, 1>(nh, kc);
    return ret;
  }

  double show_xml(RMF::NodeHandle nh) {
    double ret=0;
    ret+=nh.get_name().size();
    ret+=nh.get_id().get_index();
    ret+=show_data_xml(nh, RMF::Physics);
    ret+=show_data_xml(nh, RMF::Sequence);
    ret+=show_data_xml(nh, RMF::Shape);
    ret+=show_data_xml(nh, RMF::Feature);
    RMF::NodeHandles children= nh.get_children();
    for (unsigned int i=0; i< children.size(); ++i) {
      ret+=show_xml(children[i]);
    }
    return ret;
  }
}


double traverse(std::string name) {
  double ret=0;
  RMF::RootHandle rh= RMF::open_rmf_file(name);
  ret+=show_xml(rh);
  RMF::NodePairHandles ps= rh.get_node_tuples<2>();
  for (unsigned int i=0; i< ps.size(); ++i) {
    std::pair< RMF::NodeHandle, RMF::NodeHandle> handles
        (ps[i].get_node(0),ps[i].get_node(1));
    ret+=handles.first.get_id().get_index();
    ret+=handles.second.get_id().get_index();
  }
  return ret;
}

int main(int, char **) {
  try {
    double runtime, dist=0;
    IMP_TIME(
             {
               dist+=traverse(IMP::benchmark::get_data_path("rnapii.rmf"));
             }, runtime);
    /*std::cout << "TEST1 (decorator_access)  took " << runtime
      << " (" << dist << ")"<< std::endl;*/
    IMP::benchmark::report("rmf large", runtime, dist);
  } catch (const std::exception &e) {
    std::cerr << "Exception thrown: " << e.what() << std::endl;
  }
  return 0;
}
#else
int main(int, const char*) {
  return 1;
}
#endif
