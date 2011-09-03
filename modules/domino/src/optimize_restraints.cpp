/**
 *  \file interaction_graph.cpp
 *  \brief Score particles with respect to a tunnel.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/domino/optimize_restraints.h>
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/Restraint.h>
#include <IMP/ScoreState.h>
#include <IMP/compatibility/map.h>
#include <boost/graph/graphviz.hpp>
#include <IMP/internal/graph_utility.h>
#include <IMP/domino/internal/restraint_evaluator.h>
#include <IMP/RestraintSet.h>
#include <IMP/core/internal/CoreClosePairContainer.h>
#include <IMP/domino/particle_states.h>
#include <IMP/core/GridClosePairsFinder.h>
#include <IMP/core/ClosePairsPairScore.h>
#include <IMP/container/PairsRestraint.h>
#include <IMP/core/TableRefiner.h>
#include <IMP/core/PairRestraint.h>


IMP_BEGIN_NAMESPACE
IMPEXPORT bool
get_is_static_container(Container *c,
                        const DependencyGraph &dg,
                        const ParticlesTemp &pst);
IMP_END_NAMESPACE
IMPDOMINO_BEGIN_NAMESPACE


namespace {
 typedef boost::graph_traits<DependencyGraph> DGTraits;
  typedef DGTraits::vertex_descriptor DGVertex;
  typedef boost::property_map<DependencyGraph,
                              boost::vertex_name_t>::type DGVertexMap;
  typedef boost::property_map<DependencyGraph,
                              boost::vertex_name_t>::const_type
  DGConstVertexMap;
}

namespace {

  /*
ParticlesTemp pt= c_->get_particles();
    // we don't want the slack any more
    cpf_->set_distance(distance_);
    IntPairs ips= cpf_->get_close_pairs(bbs);
    cpf_->set_distance(distance_+2*slack_);
    ParticlePairsTemp val(ips.size());
    for (unsigned int i=0; i< ips.size(); ++i) {
      val[i]= ParticlePair(pt[ips[i].first], pt[ips[i].second]);
    }
    update_list(val);
   */

  ParticlePairsTemp
  get_static_pairs(core::internal::CoreClosePairContainer *cpc,
                   const ParticleStatesTable *pst) {
    const Subset optimized= pst->get_subset();
    ParticleStatesList psl(optimized.size());
    unsigned int max=0;
    for (unsigned int i=0; i< optimized.size(); ++i) {
      psl[i]= pst->get_particle_states(optimized[i]);
      max= std::max(psl[i]->get_number_of_particle_states(), max);
    }
    ParticlesTemp inputs
      = cpc->get_singleton_container()->get_contained_particles();
    algebra::BoundingBox3Ds bbs(inputs.size());
    for (unsigned int i=0; i< max; ++i) {
      for (unsigned int j=0; j< optimized.size(); ++j) {
        psl[j]->load_particle_state(std::min(i,
                       psl[j]->get_number_of_particle_states()-1),
                                      optimized[j]);
      }
      // make sure invariants are updated
      cpc->get_model()->update();
      for (unsigned int j=0; j< inputs.size(); ++j) {
        core::XYZ d(inputs[j]);
        bbs[j]+= d.get_coordinates();
      }
    }
    for (unsigned int j=0; j< inputs.size(); ++j) {
      core::XYZR d(inputs[j]);
      bbs[j]+= d.get_radius() + cpc->get_distance();
    }
    IMP_NEW(core::GridClosePairsFinder, gcpf, ());
    gcpf->set_distance(cpc->get_distance());
    IntPairs ips= gcpf->get_close_pairs(bbs);
    ParticlePairsTemp ret(ips.size());
    for (unsigned int i=0; i< ips.size(); ++i) {
      ret[i]= ParticlePair(inputs[ips[i].first], inputs[ips[i].second]);
    }
    return ret;
  }

  bool handle_nbl(Restraint *r, RestraintSet *p,
                          const DependencyGraph &,
                          const IMP::compatibility::map<Object*,
                          unsigned int> &,
                          const ParticleStatesTable *pst,
                          boost::ptr_vector<ScopedRemoveRestraint> &removed,
                          boost::ptr_vector<ScopedRestraint> &added) {
    container::PairsRestraint *pr= dynamic_cast<container::PairsRestraint*>(r);
    if (!pr) return false;
    core::internal::CoreClosePairContainer *cpc
  = dynamic_cast<core::internal::CoreClosePairContainer*>(pr->get_container());
    if (!cpc) return false;
    IMP_LOG(VERBOSE, "Decomposing nbl restraint " << r->get_name()
            << std::endl);
    PairScore *ssps= pr->get_score();
    ParticlePairsTemp pairs= get_static_pairs(cpc, pst);

    IMP::compatibility::map<Particle*, ParticlesTemp> rbs;
    ParticlePairsTemp rb_pairs;
    ParticlePairsTemp stray_pairs;
    double max= r->get_maximum_score();
    IMP_NEW(RestraintSet, cur, (r->get_name()+" set"));
    added.push_back(new ScopedRestraint(cur, p));
    if (r->get_maximum_score()
        < std::numeric_limits<double>::max()) {
      cur->set_maximum_score(r->get_maximum_score());
    }
    for (unsigned int i=0; i< pairs.size(); ++i) {
      bool rb=false;
      Particle *p0, *p1;
      if (core::RigidMember::particle_is_instance(pairs[i][0])) {
        rb=true;
        p0= core::RigidMember(pairs[i][0]).get_rigid_body();
        rbs[p0].push_back(pairs[i][0]);
      } else {
        p0= pairs[i][0];
      }
      if (core::RigidMember::particle_is_instance(pairs[i][1])) {
        rb=true;
        p1= core::RigidMember(pairs[i][1]).get_rigid_body();
        rbs[p1].push_back(pairs[i][1]);
      } else {
        p1= pairs[i][1];
      }
      if (!rb) {
        stray_pairs.push_back(ParticlePair(p0, p1));
      } else if (p0 != p1) {
        rb_pairs.push_back(ParticlePair(p0, p1));
      }
    }
    if (!rbs.empty()) {
      IMP_NEW(core::TableRefiner, tr, ());
      for (IMP::compatibility::map<Particle*, ParticlesTemp>::const_iterator
             it= rbs.begin();
           it != rbs.end(); ++it) {
        tr->add_particle(it->first, it->second);
      }
      std::sort(rb_pairs.begin(), rb_pairs.end());
      rb_pairs.erase(std::unique(rb_pairs.begin(), rb_pairs.end()),
                     rb_pairs.end());
      IMP_NEW(core::ClosePairsPairScore, cpps, (ssps, tr, cpc->get_distance()));
      for (unsigned int i=0; i< rb_pairs.size(); ++i) {
        std::ostringstream oss;
        oss << r->get_name() << " rb " << i;
        IMP_NEW(core::PairRestraint, pr, (cpps, rb_pairs[i]));
        pr->set_log_level(r->get_log_level());
        IMP_LOG(VERBOSE, "Adding rigid body close pair score between "
                << rb_pairs[i][0]->get_name() << " and "
                << rb_pairs[i][1]->get_name() << std::endl);
        cur->add_restraint(pr);
      }
    }

    for (unsigned int i=0; i< stray_pairs.size(); ++i) {
      std::ostringstream oss;
      oss << r->get_name() << " " << i;
      IMP_NEW(core::PairRestraint, npr, (ssps, stray_pairs[i]));
      npr->set_name(oss.str());
      npr->set_log_level(r->get_log_level());
      IMP_LOG(VERBOSE, "Adding normal close pair score between "
                << stray_pairs[i][0]->get_name() << " and "
                << stray_pairs[i][1]->get_name() << std::endl);
      cur->add_restraint(npr);
      if (max < std::numeric_limits<double>::max()) {
        npr->set_maximum_score(max);
      }
    }
    removed.push_back(new ScopedRemoveRestraint(r, p));
    return true;
  }

  bool handle_decomposable(Restraint *r, RestraintSet *p,
                           const DependencyGraph &dg,
                           const IMP::compatibility::map<Object*,
                                                    unsigned int> &index,
                           const ParticleStatesTable *pst,
                           boost::ptr_vector<ScopedRemoveRestraint> &removed,
                           boost::ptr_vector<ScopedRestraint> &added) {
    // containers can be particles, skip them if they are input particles
    ContainersTemp ic= r->get_input_containers();
    ParticlesTemp optimized= pst->get_particles();
    std::sort(optimized.begin(), optimized.end());
    for (unsigned int i=0; i< ic.size(); ++i) {
      Container *c= ic[i];
      if (!std::binary_search(optimized.begin(), optimized.end(),
                              static_cast<Particle*>(c))) {
        int start=index.find(ic[i])->second;
        if (IMP::internal::get_has_ancestor(dg, start, optimized)) {
          IMP_LOG(TERSE, "Container " << ic[i]->get_name()
                  << " has an optimized anscestor" << std::endl);
          return false;
        }
      }
    }
    Pointer<Restraint> rd= r->create_decomposition();
    RestraintSet *rs= dynamic_cast<RestraintSet*>(rd.get());
    if (rs) {
      IMP_LOG(TERSE, "Restraint \"" << r->get_name()
              << "\" is being decompsed into " << rs->get_number_of_restraints()
              << " restraints"
              << std::endl);
      added.push_back(new ScopedRestraint(rs, p));
      removed.push_back(new ScopedRemoveRestraint(r, p));
    } else {
    }
    return true;
  }



  void optimize_restraint(Restraint *r, RestraintSet *p,
                          const DependencyGraph &dg,
                          const IMP::compatibility::map<Object*,
                          unsigned int> &index,
                          const ParticleStatesTable *pst,
                          boost::ptr_vector<ScopedRemoveRestraint> &removed,
                          boost::ptr_vector<ScopedRestraint> &added) {
    if (handle_nbl(r, p, dg, index, pst, removed, added)) {
      return;
    } else if (handle_decomposable(r, p, dg, index, pst, removed, added)) {
      return;
    } else {
      return;
    }
  }

  void optimize_restraint_parent(RestraintSet *p,
                                 const DependencyGraph &dg,
                                 const IMP::compatibility::map<Object*,
                                 unsigned int> &index,
                                 const ParticleStatesTable *pst,
                           boost::ptr_vector<ScopedRemoveRestraint> &removed,
                                 boost::ptr_vector<ScopedRestraint> &added) {
    Restraints all(p->restraints_begin(), p->restraints_end());
    for (unsigned int i=0; i < all.size(); ++i) {
      Restraint *r=all[i];
      RestraintSet *rs= dynamic_cast<RestraintSet*>(r);
      if (rs) {
        optimize_restraint_parent(rs, dg, index, pst,
                                  removed,
                                  added);
      } else {
        optimize_restraint(all[i], p, dg, index, pst,
                           removed, added);
      }
    }
  }
  void optimize_score_state(ScoreState *ss,
                            const DependencyGraph &dg,
                            const IMP::compatibility::map<Object*,
                                           unsigned int> &index,
                            const ParticleStatesTable *pst,
                  boost::ptr_vector<ScopedRemoveScoreState> &removed,
                  boost::ptr_vector<ScopedScoreState> &added) {
    ContainersTemp ic= ss->get_input_containers();
    ParticlesTemp optimized= pst->get_particles();
    std::sort(optimized.begin(), optimized.end());
    for (unsigned int i=0; i< ic.size(); ++i) {
      Container *c=ic[i];
      if (!std::binary_search(optimized.begin(), optimized.end(),
                              static_cast<Particle*>(c))) {
        IMP_USAGE_CHECK(index.find(ic[i]) != index.end(),
                        "I don't know anything about object "
                        << ic[i]->get_name());
        int start=index.find(ic[i])->second;
        if (IMP::internal::get_has_ancestor(dg, start, optimized)) {
          IMP_LOG(TERSE, "Container " << ic[i]->get_name()
                  << " has an optimized anscestor" << std::endl);
          return;
        }
      }
    }
    ScoreStates sss= ss->create_decomposition();
    if (sss.size() >1) {
      Model *m= ss->get_model();
      IMP_LOG(TERSE, "Decomposing state " << ss->get_name()
              << " into " << sss.size() << " states" << std::endl);
      removed.push_back(new ScopedRemoveScoreState(ss, m));
      for (unsigned int i=0; i< sss.size(); ++i) {
        added.push_back(new ScopedScoreState(sss[i], m));
      }
    } else {
      IMP_LOG(TERSE, "State " << ss->get_name()
              << " cannot be decomposed" << std::endl);
    }
  }
}




void OptimizeRestraints::optimize_model(RestraintSet *m,
                                        const ParticleStatesTable *pst) {
  IMP_FUNCTION_LOG;
  //std::cout << "new gra[j is \n";
  //IMP::internal::show_as_graphviz(m->get_dependency_graph(), std::cout);
  {
    const DependencyGraph dg
      =get_dependency_graph(get_restraints(RestraintsTemp(1, m)));
    DGConstVertexMap vm= boost::get(boost::vertex_name,dg);
    IMP::compatibility::map<Object*, unsigned int> index;
    unsigned int nv=boost::num_vertices(dg);
    for (unsigned int i=0; i< nv; ++i) {
      index[vm[i]]= i;
    }
    // optimize score states
    ScoreStates ss(m->get_model()->score_states_begin(),
                   m->get_model()->score_states_end());
    for (unsigned int i=0; i< ss.size(); ++i) {
      optimize_score_state(ss[i], dg, index, pst,
                           removeds_, addeds_);
    }
  }
    // recompute graph

  {
    const DependencyGraph dg
      =get_dependency_graph(get_restraints(RestraintsTemp(1,m)));
    IMP::compatibility::map<Object*, unsigned int> index;
    // now do restraints
    DGConstVertexMap vm= boost::get(boost::vertex_name,dg);
    unsigned int nv=boost::num_vertices(dg);
    for (unsigned int i=0; i< nv; ++i) {
      index[vm[i]]= i;
    }
    optimize_restraint_parent(m,
                              dg, index,
                              pst, removed_,  added_);
  }
}


IMPDOMINO_END_NAMESPACE
