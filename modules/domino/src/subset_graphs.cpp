/**
 *  \file domino/DominoSampler.h \brief A beyesian infererence-based
 *  sampler.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/domino/domino_config.h>
#include <IMP/domino/subset_graphs.h>
#include <IMP/domino/internal/inference_utility.h>
#include <IMP/domino/utility.h>
#include <IMP/domino/optimize_restraints.h>
#include <IMP/domino/particle_states.h>
#include <IMP/internal/graph_utility.h>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/copy.hpp>
#include <IMP/domino/internal/maximal_cliques.h>
#include <IMP/compatibility/vector_property_map.h>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>



IMPDOMINO_BEGIN_NAMESPACE


namespace {
  typedef boost::graph_traits<InteractionGraph> IGTraits;
  typedef IGTraits::edge_descriptor IGEdge;
  typedef IGTraits::vertex_descriptor IGVertex;

  typedef boost::property_map<InteractionGraph,
                              boost::vertex_name_t>::type IGVertexMap;
  typedef boost::property_map<InteractionGraph,
                              boost::edge_name_t>::type IGEdgeMap;
  typedef boost::property_map<InteractionGraph,
                              boost::vertex_name_t>::const_type
  IGVertexConstMap;
  typedef boost::property_map<InteractionGraph,
                              boost::edge_name_t>::const_type IGEdgeConstMap;


  typedef boost::graph_traits<DependencyGraph> DGTraits;
  typedef DGTraits::vertex_descriptor DGVertex;
  typedef boost::property_map<DependencyGraph,
                              boost::vertex_name_t>::type DGVertexMap;
  typedef boost::property_map<DependencyGraph,
                              boost::vertex_name_t>::const_type
  DGConstVertexMap;



  /*typedef boost::adjacency_list<boost::vecS, boost::vecS,
    boost::undirectedS,
    boost::property<boost::vertex_name_t,
    std::string>,
    boost::property<boost::edge_weight_t,
    double> > CliqueGraph;*/
  typedef boost::graph_traits<CliqueGraph> CGTraits;
  typedef CGTraits::vertex_descriptor CGVertex;
  typedef CGTraits::edge_descriptor CGEdge;
  typedef boost::property_map<CliqueGraph,
                              boost::vertex_name_t>::type CGVertexMap;
  typedef boost::property_map<CliqueGraph,
             boost::vertex_name_t>::const_type CGVertexConstMap;


  typedef boost::property_map<SubsetGraph,
                              boost::vertex_name_t>::type SGVertexMap;
  typedef boost::property_map<SubsetGraph,
                    boost::vertex_name_t>::const_type SGConstVertexMap;
  typedef boost::graph_traits<SubsetGraph> SGTraits;

}

Subsets get_subsets(const SubsetGraph &g){
  boost::property_map< SubsetGraph, boost::vertex_name_t>::const_type
    subset_map= boost::get(boost::vertex_name, g);
  Subsets output;
  for (unsigned int vi = 0;vi < boost::num_vertices(g);vi++) {
    output.push_back(boost::get(subset_map,vi));
  }
  return output;
}







SubsetGraph get_restraint_graph(RestraintSet *irs,
                                const ParticleStatesTable *pst) {
  OptimizeRestraints ro(irs, pst);
  RestraintsTemp rs= get_restraints(RestraintsTemp(1, irs));
  //ScoreStatesTemp ss= get_required_score_states(rs);
  SubsetGraph ret(rs.size());// + ss.size());
  IMP_LOG(TERSE, "Creating restraint graph on "
          << rs.size() << " restraints." << std::endl);
  IMP::compatibility::map<Particle*, int> map;
  SGVertexMap pm= boost::get(boost::vertex_name, ret);
  DependencyGraph dg = get_dependency_graph(rs);
  /*IMP_IF_LOG(VERBOSE) {
    IMP_LOG(VERBOSE, "dependency graph is \n");
    IMP::internal::show_as_graphviz(dg, std::cout);
    }*/
  Subset ps= pst->get_subset();
  for (unsigned int i=0; i< ps.size(); ++i) {
    ParticlesTemp t= get_dependent_particles(ps[i],
                                             ParticlesTemp(ps.begin(),
                                                           ps.end()),
                                             dg);
    for (unsigned int j=0; j< t.size(); ++j) {
      IMP_USAGE_CHECK(map.find(t[j]) == map.end(),
                      "Currently particles which depend on more "
                      << "than one particle "
                      << "from the input set are not supported."
                      << "  Particle \"" << t[j]->get_name()
                      << "\" depends on \"" << ps[i]->get_name()
                      << "\" and \"" << ps[map.find(t[j])->second]->get_name()
                      << "\"");
      map[t[j]]= i;
    }
    IMP_IF_LOG(VERBOSE) {
      IMP_LOG(VERBOSE, "Particle \"" << ps[i]->get_name() << "\" controls ");
      for (unsigned int i=0; i< t.size(); ++i) {
        IMP_LOG(VERBOSE, "\""<< t[i]->get_name() << "\" ");
      }
      IMP_LOG(VERBOSE, std::endl);
    }
  }
  for (unsigned int i=0; i < rs.size(); ++i) {
    ParticlesTemp pl= rs[i]->get_input_particles();
    std::sort(pl.begin(), pl.end());
    pl.erase(std::unique(pl.begin(), pl.end()), pl.end());
    Subset os(pl);
    for (unsigned int j=0; j< pl.size(); ++j) {
      pl[j]= ps[map[pl[j]]];
    }
    std::sort(pl.begin(), pl.end());
    pl.erase(std::unique(pl.begin(), pl.end()), pl.end());
    Subset s(pl);
    IMP_LOG(VERBOSE, "Subset for restraint " << rs[i]->get_name()
            << " is " << s << " from " << os << std::endl);
    pm[i]=s;
  }
  /*ScoreStatesTemp ss= get_required_score_states(rs);
    for (ScoreStatesTemp::const_iterator it= ss.begin();
    it != ss.end(); ++it) {
    ParticlesTemp pl= (*it)->get_input_particles();
    add_edges(ps, pl, map, *it, ret);
    ParticlesTemp opl= (*it)->get_output_particles();
    add_edges(ps, opl, map, *it, ret);
    }
    IMP_INTERNAL_CHECK(boost::num_vertices(ret) == ps.size(),
    "Wrong number of vertices "
    << boost::num_vertices(ret)
    << " vs " << ps.size());*/
  for (unsigned int i=0; i< boost::num_vertices(ret); ++i) {
    for (unsigned int j=0; j< i; ++j) {
      if (get_intersection(pm[i], pm[j]).size() >0) {
        boost::add_edge(i,j,ret);
        IMP_LOG(VERBOSE, "Connecting " << rs[i]->get_name()
                << " with " << rs[j]->get_name() << std::endl);
      }
    }
  }
  return ret;
}




IMPDOMINOEXPORT CliqueGraph get_clique_graph(const InteractionGraph& cig) {
  IGVertexConstMap pm= boost::get(boost::vertex_name, cig);
  typedef compatibility::checked_vector<IGVertex> Clique;
  compatibility::checked_vector<Clique> cliques;
  internal::maximal_cliques(cig, std::back_inserter(cliques));
  for (unsigned int i=0; i< cliques.size(); ++i) {
    /*std::cout << "Clique is ";
      for (unsigned int j=0; j< cliques[i].size(); ++j) {
      std::cout << cliques[i][j] << " ";
      }*/
    std::sort(cliques[i].begin(), cliques[i].end());
  }
  CliqueGraph cg(cliques.size());
  CGVertexMap cm
    = boost::get(boost::vertex_name, cg);
  for (unsigned int i=0; i< cliques.size(); ++i) {
    ParticlesTemp cur;
    for (unsigned int j=0; j< cliques[i].size(); ++j) {
      cur.push_back(pm[cliques[i][j]]);
    }
    Subset ss(cur);
    cm[i]=ss;
  }
  for (unsigned int i=0; i< cliques.size(); ++i) {
    for (unsigned int j=0; j< i; ++j) {
      Subset intersection= get_intersection(cm[i], cm[j]);
      if (intersection.size() >0) {
        double minus_weight=intersection.size();
        /*std::cout << "edge " << i << " " << j
          << " has weight "
          << -static_cast<int>(intersection.size()) << std::endl;*/
        boost::add_edge(i, j,
                        CliqueGraph::edge_property_type(-minus_weight),
                        cg);
      }
    }
  }
  return cg;
}


namespace {
  struct LessDegree {
    const InteractionGraph &ig_;
    LessDegree(InteractionGraph &ig): ig_(ig){}
    bool operator()(int a, int b) const {
      return boost::degree(a, ig_) > boost::degree(b, ig_);
    }
  };
  void triangulate(InteractionGraph &ig) {
    typedef std::pair<IGTraits::adjacency_iterator,
                      IGTraits::adjacency_iterator>
      AdjacencyRange;
    typedef std::pair<IGTraits::vertex_iterator, IGTraits::vertex_iterator>
      VertexRange;
    typedef std::pair<IGTraits::out_edge_iterator, IGTraits::out_edge_iterator>
      EdgeRange;
    InteractionGraph mig;
    boost::copy_graph(ig, mig);
    IMP::compatibility::map<Particle*, int> vmap;
    IGVertexMap mpm= boost::get(boost::vertex_name, mig);
    for(VertexRange be = boost::vertices(ig);
        be.first != be.second; ++be.first) {
      /*std::cout << "Vertex " << *be.first
        << " is particle " << boost::get(mpm, *be.first)->get_name()
        << std::endl;*/
      vmap[boost::get(mpm, *be.first)]=*be.first;
    }
    while (boost::num_vertices(mig) >0) {
      int maxv=-1;
      int maxd=std::numeric_limits<int>::max();
      for ( VertexRange be = boost::vertices(mig);
            be.first != be.second; ++be.first) {
        int d= boost::degree(*be.first, mig);
        if (d < maxd) {
          maxd=d;
          maxv=*be.first;
        }
      }
      //unsigned int v= vmap.find(mpm[maxv])->second;
      /*std::cout << "Triangulating vertex "
        << vmap.find(mpm[maxv])->second
        << " with degree " << maxd << std::endl;
        std::cout << boost::num_vertices(mig)
        << " remaining" << std::endl;*/
      AdjacencyRange be  = boost::adjacent_vertices(maxv, mig);
      const compatibility::checked_vector<unsigned int>
        neighbors(be.first, be.second);
      /*std::cout << "Neighbors are ";
        for (unsigned int i=0; i < neighbors.size(); ++i) {
        std::cout << neighbors[i] << " ";
        }
        std::cout << std::endl;*/
      for (unsigned int i=1; i< neighbors.size(); ++i) {
        //std::cout << "neighbor 0 is "
        // << boost::get(mpm, neighbors[i])->get_name() << std::endl;
        IGVertex o0 =  vmap.find(boost::get(mpm, neighbors[i]))->second;
        for (unsigned int j=0; j<i; ++j) {
          /*std::cout << "neighbor 1 is "
            << boost::get(mpm, neighbors[j])->get_name() << std::endl;*/
          IGVertex o1 = vmap.find(boost::get(mpm, neighbors[j]))->second;
          // check for adjacency in ig, ick. painful
          AdjacencyRange be01 = boost::adjacent_vertices(o0, ig);
          for (;be01.first != be01.second; ++be01.first) {
            if (*be01.first == o1) break;
          }
          // connect if not adjacent
          if (be01.first == be01.second) {
            //std::cout << "Connecting " << o0 << " " << o1 << std::endl;
            //std::cout << "and " << neighbors[i] << " "
            //<< neighbors[j] << std::endl;
            boost::add_edge(neighbors[i], neighbors[j], mig);
            boost::add_edge(o0, o1, ig);
          } else {
            //std::cout << "Already connected " << o0
            //<< " " << o1 << std::endl;
          }
        }
      }
      EdgeRange er=boost::out_edges(maxv, mig);
      while (er.first != er.second) {
        boost::remove_edge(*er.first, mig);
        er= boost::out_edges(maxv, mig);
      }
      boost::remove_vertex(maxv, mig);
      /*std::cout << "MIG graph is " << std::endl;
        IMP::internal::show_as_graphviz(mig, std::cout);
        std::cout << "Output graph is " << std::endl;
        IMP::internal::show_as_graphviz(ig, std::cout);*/
    }
  }

}

InteractionGraph get_triangulated(const InteractionGraph& ig) {
  InteractionGraph cig;
  boost::copy_graph(ig, cig);
  /*std::cout << "Input graph is " << std::endl;
    IMP::internal::show_as_graphviz(ig, std::cout);*/
  triangulate(cig);
  IMP_LOG(VERBOSE, "Triangulated graph is " << std::endl);
  IMP_LOG_WRITE(VERBOSE, IMP::internal::show_as_graphviz(cig, IMP_STREAM));
  return cig;

}


SubsetGraph get_minimum_spanning_tree(const CliqueGraph& cg) {
  compatibility::checked_vector<CGEdge> mst;
  boost::kruskal_minimum_spanning_tree(cg, std::back_inserter(mst));
  SubsetGraph jt(boost::num_vertices(cg));
  SGVertexMap cm= boost::get(boost::vertex_name, jt);
  CGVertexConstMap cgm= boost::get(boost::vertex_name, cg);
  for (unsigned int i=0; i< boost::num_vertices(cg); ++i) {
    cm[i]= cgm[i];
  }
  for (unsigned int i=0; i< mst.size(); ++i) {
    boost::add_edge(boost::source(mst[i], cg),
                    boost::target(mst[i], cg), jt);
  }
  // find disjoint sets and connect them arbitrarily
  {
    typedef boost::vector_property_map<unsigned int> Index;
    typedef Index Parent;
    typedef boost::disjoint_sets<Index,Parent> UF;
    Index index;
    Parent parent;
    UF uf(index, parent);
    for (unsigned int i=0; i< boost::num_vertices(cg); ++i) {
      uf.make_set(i);
    }
    for (std::pair<SGTraits::edge_iterator,
                   SGTraits::edge_iterator> be= boost::edges(jt);
         be.first != be.second; ++be.first) {
      uf.union_set(boost::source(*be.first, jt),
                   boost::target(*be.first, jt));
    }
    for (unsigned int i=1; i< boost::num_vertices(cg); ++i) {
      if (uf.find_set(i) != uf.find_set(i-1)) {
        boost::add_edge(i,i-1, jt);
        uf.union_set(i,i-1);
      }
    }
  }
  return jt;
}

SubsetGraph get_junction_tree(const InteractionGraph &ig) {
  IMP_FUNCTION_LOG;
  InteractionGraph cig = get_triangulated(ig);
  CliqueGraph cg= get_clique_graph(cig);
  SubsetGraph jt= get_minimum_spanning_tree(cg);

  /*std::cout << "JT graph is " << std::endl;
    IMP::internal::show_as_graphviz(jt, std::cout);
    {
    SubsetGraph njt=jt;
    std::cout << "JT graph is " << std::endl;
    IMP::internal::show_as_graphviz(njt, std::cout);
    }
    std::cout << "JT graph is " << std::endl;
    IMP::internal::show_as_graphviz(jt, std::cout);*/
  return jt;
}




namespace {


  bool get_has_edge(InteractionGraph &graph,
                    IGVertex va,
                    IGVertex vb) {
    std::pair<IGTraits::out_edge_iterator,
              IGTraits::out_edge_iterator> edges= boost::out_edges(va, graph);
    for (; edges.first != edges.second;++edges.first) {
      if (boost::target(*edges.first, graph) == vb) return true;
    }
    return false;
  }

  void add_edges( const Subset &ps,
                  ParticlesTemp pt,
                  const IMP::compatibility::map<Particle*, int> &map,
                  Object *blame,
                  InteractionGraph &g) {
    IGEdgeMap om= boost::get(boost::edge_name, g);
    std::sort(pt.begin(), pt.end());
    pt.erase(std::unique(pt.begin(), pt.end()), pt.end());
    for (unsigned int i=0; i< pt.size(); ++i) {
      if (map.find(pt[i]) == map.end()) continue;
      int vj=map.find(pt[i])->second;
      for (unsigned int k=0; k< i; ++k) {
        if (map.find(pt[k]) == map.end()) continue;
        int vk= map.find(pt[k])->second;
        if (vj != vk && !get_has_edge(g, vj, vk)) {
          IMP_LOG(VERBOSE, "Adding edge between \"" << ps[vj]->get_name()
                  << "\" and \"" << ps[vk]->get_name()
                  << "\" due to \"" << blame->get_name() << "\"" << std::endl);
          IGEdge e;
          bool inserted;
          boost::tie(e, inserted)= boost::add_edge(vj, vk, g);
          if (inserted) {
            om[e]=blame;
          }
        }
      }
    }
  }

}

InteractionGraph get_interaction_graph(RestraintSet *irs,
                                       const ParticleStatesTable* pst) {
  Subset ps = pst->get_subset();
  InteractionGraph ret(ps.size());
  RestraintsTemp rs= get_restraints(RestraintsTemp(1,irs));
  //Model *m= ps[0]->get_model();
  IMP::compatibility::map<Particle*, int> map;
  IGVertexMap pm= boost::get(boost::vertex_name, ret);
  DependencyGraph dg = get_dependency_graph(rs);
  /*IMP_IF_LOG(VERBOSE) {
    IMP_LOG(VERBOSE, "dependency graph is \n");
    IMP::internal::show_as_graphviz(dg, std::cout);
    }*/
  for (unsigned int i=0; i< ps.size(); ++i) {
    ParticlesTemp t= get_dependent_particles(ps[i],
                                             ParticlesTemp(ps.begin(),
                                                           ps.end()),
                                             dg);
    for (unsigned int j=0; j< t.size(); ++j) {
      IMP_USAGE_CHECK(map.find(t[j]) == map.end(),
                      "Currently particles which depend on more "
                      << "than one particle "
                      << "from the input set are not supported."
                      << "  Particle \"" << t[j]->get_name()
                      << "\" depends on \"" << ps[i]->get_name()
                      << "\" and \"" << ps[map.find(t[j])->second]->get_name()
                      << "\"");
      map[t[j]]= i;
    }
    IMP_IF_LOG(VERBOSE) {
      IMP_LOG(VERBOSE, "Particle \"" << ps[i]->get_name() << "\" controls ");
      for (unsigned int i=0; i< t.size(); ++i) {
        IMP_LOG(VERBOSE, "\""<< t[i]->get_name() << "\" ");
      }
      IMP_LOG(VERBOSE, std::endl);
    }
    pm[i]= ps[i];
  }
  for (RestraintsTemp::const_iterator it= rs.begin();
       it != rs.end(); ++it) {
    ParticlesTemp pl= (*it)->get_input_particles();
    add_edges(ps, pl, map, *it, ret);
  }
  ScoreStatesTemp ss= get_required_score_states(rs);
  for (ScoreStatesTemp::const_iterator it= ss.begin();
       it != ss.end(); ++it) {
    ParticlesTemp pl= (*it)->get_input_particles();
    add_edges(ps, pl, map, *it, ret);
    ParticlesTemp opl= (*it)->get_output_particles();
    add_edges(ps, opl, map, *it, ret);
  }
  IMP_INTERNAL_CHECK(boost::num_vertices(ret) == ps.size(),
                     "Wrong number of vertices "
                     << boost::num_vertices(ret)
                     << " vs " << ps.size());
  return ret;
}


display::Geometries
get_interaction_graph_geometry(const InteractionGraph &ig) {
  display::Geometries ret;
  IGVertexConstMap vm= boost::get(boost::vertex_name, ig);
  IGEdgeConstMap em= boost::get(boost::edge_name, ig);
  IMP::compatibility::map<std::string, display::Color> colors;
  for (std::pair<IGTraits::vertex_iterator,
         IGTraits::vertex_iterator> be= boost::vertices(ig);
       be.first != be.second; ++be.first) {
    Particle *p= dynamic_cast<Particle*>(vm[*be.first]);
    core::XYZ pd(p);
    for (std::pair<IGTraits::out_edge_iterator,
           IGTraits::out_edge_iterator> ebe= boost::out_edges(*be.first, ig);
         ebe.first != ebe.second; ++ebe.first) {
      unsigned int target= boost::target(*ebe.first, ig);
      if (target > *be.first) continue;
      Particle *op= dynamic_cast<Particle*>(vm[target]);
      core::XYZ od(op);
      std::string on= em[*ebe.first]->get_name();
      IMP_NEW(display::SegmentGeometry, cg,
              (algebra::Segment3D(pd.get_coordinates(),
                                  od.get_coordinates())));
      if (colors.find(em[*ebe.first]->get_name()) == colors.end()) {
        colors[em[*ebe.first]->get_name()]
          = display::get_display_color(colors.size());
      }
      cg->set_color(colors[em[*ebe.first]->get_name()]);
      cg->set_name(on);
      ret.push_back(cg.get());
    }
  }
  return ret;
}


display::Geometries
get_subset_graph_geometry(const SubsetGraph &ig) {
  display::Geometries ret;
  SGConstVertexMap vm= boost::get(boost::vertex_name, ig);
  for (std::pair<IGTraits::vertex_iterator,
         IGTraits::vertex_iterator> be= boost::vertices(ig);
       be.first != be.second; ++be.first) {
    Subset s= vm[*be.first];
    display::Color c= display::get_display_color(*be.first);
    for (unsigned int i=0; i< s.size(); ++i) {
      core::XYZ pi(s[i]);
      IMP_NEW(display::SphereGeometry, cg,
              (algebra::Sphere3D(pi.get_coordinates(), 1)));
      cg->set_color(c);
      cg->set_name(s.get_name());
      ret.push_back(cg.get());
    }
  }
  return ret;
}

/* DFV
   on end child:
   union sets
   create child node with cur union
   link created child node to child being visited
   on visit:
   look up cur node in map


*/


namespace {
  typedef boost::graph_traits<MergeTree> MTTraits;
  typedef CGTraits::vertex_descriptor MTVertex;
  typedef CGTraits::edge_descriptor MTEdge;
  typedef boost::property_map<MergeTree,
                              boost::vertex_name_t>::type MTVertexMap;



  int create_set_node(const Subset &s,
                      MergeTree& merge_tree,
                      boost::property_map<MergeTree,
                      boost::vertex_name_t>::type &mt_sets) {
    int vi= boost::add_vertex(merge_tree);
    mt_sets[vi]=s;
    return vi;
  }
  int create_merge_tree_internal(const SubsetGraph& junction_tree,
                                 const  boost::property_map<SubsetGraph,
                                 boost::vertex_name_t>::const_type &jt_sets,
                                 int cur_jt,
                                 int last_jt, // avoid parent
                                 MergeTree& merge_tree,
                                 boost::property_map<MergeTree,
                                 boost::vertex_name_t>::type &mt_sets) {
    Subset cur_subset=jt_sets[cur_jt];
    int cur_merge= create_set_node(cur_subset, merge_tree, mt_sets);
    for (std::pair<SGTraits::out_edge_iterator,
           SGTraits::out_edge_iterator> ebe= boost::out_edges(cur_jt,
                                                              junction_tree);
         ebe.first != ebe.second; ++ebe.first) {
      int target= boost::target(*ebe.first, junction_tree);
      if (target== last_jt) continue;
      int child= create_merge_tree_internal(junction_tree,
                                            jt_sets,
                                            target, cur_jt,
                                            merge_tree, mt_sets);
      Subset child_set= mt_sets[child];
      Subset u= get_union(child_set,cur_subset);
      int merged= create_set_node(u, merge_tree, mt_sets);
      boost::add_edge(merged, cur_merge, merge_tree);
      boost::add_edge(merged, child, merge_tree);
      cur_merge=merged;
      cur_subset=u;
    }
    return cur_merge;
  }
}


MergeTree get_merge_tree(const SubsetGraph& junction_tree/*, int start*/) {
  IMP_IF_CHECK(USAGE) {
    Ints comp(boost::num_vertices(junction_tree));
    int cc= boost::connected_components(junction_tree, &comp[0]);
    IMP_USAGE_CHECK(cc==1, "Graph is not connected: " << cc);
  }
  int start =0;
  MergeTree merge_tree;
  boost::property_map<MergeTree,
    boost::vertex_name_t>::type mt_sets
    =boost::get(boost::vertex_name, merge_tree);
  unsigned int root= create_merge_tree_internal(junction_tree,
                                                boost::get(boost::vertex_name,
                                                           junction_tree),
                                                start, -1,
                                                merge_tree,
                                                mt_sets);
  IMP_USAGE_CHECK(root== boost::num_vertices(merge_tree)-1,
                  "Root is not last vertex");
  // create dfv
  // add first node to jt/merge map/merge graph with all particles
  // do dft add nodes and computing graph as we go
  IMP_INTERNAL_CHECK(get_is_merge_tree(merge_tree, mt_sets[root], true),
                     "Returned tree is not merge tree");
  return merge_tree;
}

namespace {
  bool get_is_merge_tree(const MergeTree& tree,
                         const boost::property_map<MergeTree,
                         boost::vertex_name_t>::const_type&
                         mt_sets,
                         bool verbose,
                         int cur, int parent) {
    Subset cur_set= mt_sets[cur];
    Subset children;
    int nc=0;
    bool ret=true;
    bool has_children=false;
    for (std::pair<MTTraits::out_edge_iterator,
           MTTraits::out_edge_iterator> ebe= boost::out_edges(cur, tree);
         ebe.first != ebe.second; ++ebe.first) {
      int target= boost::target(*ebe.first, tree);
      has_children=true;
      if (target== parent) continue;
      ++nc;
      Subset curs = mt_sets[target];
      children= get_union(children, curs);
      ret= ret&& get_is_merge_tree(tree, mt_sets, verbose, target, cur);
    }
    if (!has_children) return cur_set.size()>0;
    if (cur_set != children) {
      if (verbose) {
        IMP_WARN("Subsets don't match " << cur_set << " vs " << children);
        return false;
      }
    }
    if (nc != 2 && nc != 0) {
      IMP_WARN("It is not a binary tree");
      return false;
    }
    return ret;
  }
}

bool get_is_merge_tree(const MergeTree& tree, Subset all, bool verbose) {
  boost::property_map<MergeTree,
    boost::vertex_name_t>::const_type mt_sets
    = boost::get(boost::vertex_name, tree);
  int nv= boost::num_vertices(tree);
  Subset maybeall= mt_sets[nv-1];
  if (maybeall != all) {
    if (verbose) {
      IMP_WARN("Root does not contain all particles: " << all
               << " vs " << maybeall);
      return false;
    }
  }
  return get_is_merge_tree(tree, mt_sets,
                           verbose, nv-1, -1);
}




MergeTree get_merge_tree(RestraintSet *rs,
                         const ParticleStatesTable *pst) {
  OptimizeRestraints ors(rs, pst);
  InteractionGraph ig= get_interaction_graph(rs, pst);
  SubsetGraph jt= get_junction_tree(ig);
  return get_merge_tree(jt);
}

namespace {
  typedef boost::adjacency_list<boost::listS,
                                boost::listS,
                                boost::undirectedS,
                                boost::property<boost::vertex_name_t,
                                                Subset>,
                                boost::property<boost::edge_name_t,
                                                int> > StableSubsetGraph;
  typedef boost::graph_traits<StableSubsetGraph>::edge_descriptor SSGED;
  typedef boost::graph_traits<StableSubsetGraph>::vertex_descriptor SSGVD;
  compatibility::checked_vector<SSGED>
  get_independent_edge_set(const StableSubsetGraph &sg) {
    compatibility::checked_vector<SSGED> ret;
    compatibility::set<SSGVD> seen;
    typedef boost::graph_traits<StableSubsetGraph>::edge_iterator EIt;
    std::pair<EIt, EIt> ep= boost::edges(sg);
    compatibility::checked_vector<SSGED> edges(ep.first, ep.second);
    std::reverse(edges.begin(), edges.end());
    for (unsigned int i=0; i< edges.size(); ++i) {
      SSGVD source= boost::source(edges[i], sg);
      SSGVD target= boost::target(edges[i], sg);
      if (seen.find(source) == seen.end()
          && seen.find(target) == seen.end()) {
        ret.push_back(edges[i]);
        seen.insert(source);
        seen.insert(target);
      }
    }
    return ret;
  }
  SSGVD merge_edge(SSGED e, StableSubsetGraph &jt) {
    SSGVD source= boost::source(e, jt);
    SSGVD target= boost::target(e, jt);
    IMP_INTERNAL_CHECK(source!=target,
                       "Can't handle self edges");
    boost::property_map<StableSubsetGraph,
      boost::vertex_name_t>::type jt_sets
      = boost::get(boost::vertex_name, jt);
    typedef boost::graph_traits<StableSubsetGraph>::adjacency_iterator AIt;
    std::pair<AIt, AIt> be  = boost::adjacent_vertices(target, jt);
    const compatibility::checked_vector< SSGVD> neighbors(be.first, be.second);
    for (unsigned int i=0; i< neighbors.size(); ++i) {
      if (neighbors[i] != source) {
        if (!boost::edge(source, neighbors[i], jt).second) {
          boost::add_edge(source, neighbors[i], jt);
        }
      }
    }
    Subset u= get_union(jt_sets[source], jt_sets[target]);
    jt_sets[source]=u;
    boost::clear_vertex(target, jt);
    boost::remove_vertex(target, jt);
    return source;
  }
}


MergeTree get_balanced_merge_tree( const SubsetGraph& jti) {
  StableSubsetGraph junction_tree;
  boost::copy_graph(jti, junction_tree);
  MergeTree ret;
  boost::property_map<StableSubsetGraph,
    boost::vertex_name_t>::type jt_sets
    = boost::get(boost::vertex_name, junction_tree);
  boost::property_map<MergeTree,
    boost::vertex_name_t>::type mt_sets
    = boost::get(boost::vertex_name, ret);
  compatibility::map<SSGVD,int> vertex_map;
  for (unsigned int i=0; i< boost::num_vertices(junction_tree); ++i) {
    SSGVD vd= boost::vertex(i, junction_tree);
    mt_sets[boost::add_vertex(ret)]=jt_sets[vd];
    vertex_map[vd]=i;
  }
  while (boost::num_vertices(junction_tree) >1) {
    compatibility::checked_vector<SSGED> is
      = get_independent_edge_set(junction_tree);
    IMP_INTERNAL_CHECK(is.size()>0, "No edges found");
    for (unsigned int i=0; i< is.size(); ++i) {
      int mn=boost::add_vertex(ret);
      SSGVD source=boost::source(is[i], junction_tree);
      SSGVD target=boost::target(is[i], junction_tree);
      boost::add_edge(mn, vertex_map.find(source)->second, ret);
      boost::add_edge(mn, vertex_map.find(target)->second, ret);
      vertex_map.erase(source);
      vertex_map.erase(target);
      SSGVD jn= merge_edge(is[i], junction_tree);
      mt_sets[mn]= jt_sets[jn];
      vertex_map[jn]=mn;
    }
    //IMP::internal::show_as_graphviz(ret, std::cout);
    /*{
      SubsetGraph sg;
      boost::copy_graph(junction_tree, sg);
      IMP::internal::show_as_graphviz(sg, std::cout);
      }*/
  }
  IMP_INTERNAL_CHECK(boost::num_vertices(junction_tree)==1,
                     "There is not one vertext left");
  IMP_INTERNAL_CHECK(get_is_merge_tree(ret,
                                       jt_sets[*boost::vertices(junction_tree)
                                               .first],
                                       true),
                     "Result is not merge tree");
  return ret;
}

IMPDOMINO_END_NAMESPACE
