/**
 *  \file IMP/rmf/Category.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/rmf/restraint_io.h>
#include <IMP/rmf/internal/imp_operations.h>
#include <IMP/scoped.h>
#include <boost/shared_array.hpp>
IMPRMF_BEGIN_NAMESPACE
using namespace RMF;

#define  IMP_HDF5_CREATE_RESTRAINT_KEYS(node)                           \
  RMF::RootHandle imp_f=node.get_root_handle();                         \
  RMF::Category Feature= imp_f.get_or_add_category<1>("feature");       \
  RMF::FloatKey sk                                                    \
  = internal::get_or_add_key<FloatTraits>(imp_f, Feature, "score",      \
                                           true);                       \
  RMF::NodeIDsKey nk                                                  \
  = internal::get_or_add_key<NodeIDsTraits>(imp_f, Feature, "representation");

#define IMP_HDF5_ACCEPT_RESTRAINT_KEYS\
  RMF::FloatKey sk, RMF::NodeIDsKey nk

#define IMP_HDF5_PASS_RESTRAINT_KEYS\
  sk, nk

namespace {

  class  Subset {
    boost::shared_array<Particle*> ps_;
    unsigned int sz_;
    int compare(const Subset &o) const {
      if (sz_ < o.sz_) return -1;
      else if (sz_ > o.sz_) return 1;
      for (unsigned int i=0; i< size(); ++i) {
        if (ps_[i] < o[i]) return -1;
        else if (ps_[i] > o[i]) return 1;
      }
      return 0;
    }
  public:
    Subset(): sz_(0){}
    Subset(ParticlesTemp ps) {
      std::sort(ps.begin(), ps.end());
      ps.erase(std::unique(ps.begin(), ps.end()), ps.end());
      sz_= ps.size();
      ps_.reset(new Particle*[ps.size()]);
      std::copy(ps.begin(), ps.end(), ps_.get());
      IMP_USAGE_CHECK(std::unique(ps.begin(), ps.end()) == ps.end(),
                      "Duplicate particles in set");
      IMP_IF_CHECK(USAGE) {
        for (unsigned int i=0; i< ps.size(); ++i) {
          IMP_CHECK_OBJECT(ps[i]);
        }
      }
    }
    unsigned int size() const {
      return sz_;
    }
    Particle *operator[](unsigned int i) const {
      IMP_USAGE_CHECK( i < sz_, "Out of range");
      return ps_[i];
    }
    typedef Particle** const_iterator;
    const_iterator begin() const {
      return ps_.get();
    }
    const_iterator end() const {
      return ps_.get()+sz_;
    }
    IMP_SHOWABLE(Subset);
    std::string get_name() const;
    IMP_HASHABLE_INLINE(Subset, return boost::hash_range(begin(),
                                                         end()););
    IMP_COMPARISONS(Subset);
  };

  void Subset::show(std::ostream &out) const {
    out << "[";
    for (unsigned int i=0; i< size(); ++i) {
      out << "\"" <<  ps_[i]->get_name() << "\" ";
    }
    out << "]";
  }

  std::string Subset::get_name() const {
    std::ostringstream oss;
    show(oss);
    return oss.str();
  }
  inline std::size_t hash_value(const Subset &t) {
    return t.__hash__();
  }


  typedef IMP::compatibility::map<Subset, RMF::NodeHandle> Index;
  void build_index(RMF::NodeHandle parent,
                   Index &nodes,
                   IMP_HDF5_ACCEPT_RESTRAINT_KEYS) {
    IMP_UNUSED(sk);
    NodeHandles children= parent.get_children();
    for (unsigned int i=0; i< children.size(); ++i) {
      ParticlesTemp pt;
      if (children[i].get_has_value(nk)) {
        NodeIDs ids= children[i].get_value(nk);
        for (unsigned int j=0; j< ids.size(); ++j) {
          RMF::NodeHandle nh
            = parent.get_root_handle().get_node_handle_from_id(ids[j]);
          Particle *p= reinterpret_cast<Particle*>(nh.get_association());
          pt.push_back(p);
        }
      }
      IMP_USAGE_CHECK(!pt.empty(), "No used particles found. Not so good: "
                      << children[i].get_name());
      Subset s(pt);
      /*std::cout << "Adding index entry for " << s << " to "
        << parent.get_name()
        << std::endl;*/
      nodes[s]=children[i];
    }
  }

  void set_particles(RMF::NodeHandle nh,
                     const ParticlesTemp& ps,
                     IMP_HDF5_ACCEPT_RESTRAINT_KEYS) {
    IMP_UNUSED(sk);
    NodeIDs ids(ps.size());
    for (unsigned int i=0; i< ps.size(); ++i) {
      NodeID id
        =nh.get_root_handle().get_node_handle_from_association(ps[i]).get_id();
      ids[i]=id;
    }
    if (!ids.empty()) {
      nh.set_value(nk, ids);
    }
  }

  RMF::NodeHandle get_child(RMF::NodeHandle parent,
                       Restraint *r,
                       Index &nodes,
                       IMP_HDF5_ACCEPT_RESTRAINT_KEYS) {
    ParticlesTemp ip= r->get_input_particles();
    Subset s(ip);
    if (nodes.find(s) == nodes.end()) {
      std::ostringstream oss;
      oss << "Feature " << nodes.size();
      RMF::NodeHandle c= parent.add_child(oss.str(), FEATURE);
      /*std::cout << "Created node for " << s
        << " under " << parent.get_name() << std::endl;*/
      nodes[s]=c;
      c.set_association(r);
      set_particles(c, ip, IMP_HDF5_PASS_RESTRAINT_KEYS);
      return c;
    } else {
      return nodes[s];
    }
  }


  void add_restraint_internal(Restraint *r,
                              RMF::NodeHandle parent,
                              IMP_HDF5_ACCEPT_RESTRAINT_KEYS) {
    RMF::NodeHandle cur= parent.add_child(r->get_name(), FEATURE);
    cur.set_association(r);
    //
    ParticlesTemp ip= r->get_input_particles();
    double s=r->evaluate(false);
    cur.set_value(sk, s, 0);

    Pointer<Restraint> rd= r->create_current_decomposition();
    if (!rd) return;
    RestraintSet *rs= dynamic_cast<RestraintSet*>(rd.get());
    if (rs) {
      Index index;
      for (unsigned int i=0; i<rs->get_number_of_restraints(); ++i) {
        //ScopedRestraint sr(rd[i], r->get_model()->get_root_restraint_set());
        rs->get_restraint(i)->set_was_used(true);
        RMF::NodeHandle rc=get_child(cur,rs->get_restraint(i), index,
                                IMP_HDF5_PASS_RESTRAINT_KEYS);
        double score = rs->get_restraint(i)->unprotected_evaluate(NULL);
        rc.set_value(sk, score, 0);
      }
    }
    set_particles(cur, ip, IMP_HDF5_PASS_RESTRAINT_KEYS);
  }
}

void add_restraint(RMF::RootHandle parent, Restraint *r) {
  IMP_FUNCTION_LOG;
  IMP_HDF5_CREATE_RESTRAINT_KEYS(parent);
  add_restraint_internal(r, parent, IMP_HDF5_PASS_RESTRAINT_KEYS);
  parent.flush();
}

namespace {

  void save_restraint_internal(Restraint *r,
                               RMF::RootHandle f,
                               int frame,
                               IMP_HDF5_ACCEPT_RESTRAINT_KEYS) {
    RMF::NodeHandle rn= f.get_node_handle_from_association(r);
    Index index;
    build_index(rn, index, IMP_HDF5_PASS_RESTRAINT_KEYS);
    double s=r->evaluate(false);
    rn.set_value(sk, s, frame);
    Pointer<Restraint> rd= r->create_current_decomposition();
    if (!rd) return;
    RestraintSet *rs= dynamic_cast<RestraintSet*>(rd.get());
    if (!rs) return;
    for (unsigned int i=0; i< rs->get_number_of_restraints(); ++i) {
      //ScopedRestraint sr(rd[i], r->get_model()->get_root_restraint_set());
      rs->get_restraint(i)->set_was_used(true);
      RMF::NodeHandle rc=get_child(rn, rs->get_restraint(i), index,
                              IMP_HDF5_PASS_RESTRAINT_KEYS);
      double score = rs->get_restraint(i)->unprotected_evaluate(NULL);
      rc.set_value(sk, score, frame);
    }
  }
}

void save_frame(RMF::RootHandle f, int frame, Restraint *r) {
  IMP_FUNCTION_LOG;
  IMP_HDF5_CREATE_RESTRAINT_KEYS(f);
  save_restraint_internal(r, f, frame, IMP_HDF5_PASS_RESTRAINT_KEYS);
  f.flush();
}

ParticlesTemp get_restraint_particles(RMF::NodeHandle f,
                                      int frame) {
  IMP_FUNCTION_LOG;
  IMP_HDF5_CREATE_RESTRAINT_KEYS(f);
  IMP_UNUSED(sk);
  IMP_USAGE_CHECK(f.get_type()== FEATURE,
                  "Get restraint particles called on non-restraint node "
                  << f.get_name());
  RMF::RootHandle rh=f.get_root_handle();
  if (f.get_has_value(nk, frame)) {
    NodeIDs ids= f.get_value(nk, frame);
    ParticlesTemp ret(ids.size());
    for (unsigned int i=0; i< ids.size(); ++i) {
      RMF::NodeHandle nh= rh.get_node_handle_from_id(ids[i]);
      Particle *p= reinterpret_cast<Particle*>(nh.get_association());
      ret[i]=p;
    }
    return ret;
  } else {
    return ParticlesTemp();
  }
}



double get_restraint_score(RMF::NodeHandle f,
                           int frame) {
  IMP_FUNCTION_LOG;
  IMP_HDF5_CREATE_RESTRAINT_KEYS(f);
  IMP_UNUSED(nk);
  if (f.get_has_value(sk, frame)) {
    return f.get_value(sk, frame);
  } else {
    return -std::numeric_limits<double>::infinity();
  }
}




IMPRMF_END_NAMESPACE
