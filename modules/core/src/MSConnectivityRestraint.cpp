/**
 *  \file MSConnectivityRestraint.cpp  \brief Mass Spec Connectivity restraint.
 *
 *  Restrict max distance between at least one pair of particles of any
 *  two distinct types. It also handles multiple copies of the same particles.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <algorithm>

#include <IMP/core/MSConnectivityRestraint.h>

#include <IMP/Model.h>
#include <IMP/Particle.h>
#include <IMP/log.h>
#include <IMP/PairScore.h>
#include <IMP/core/PairRestraint.h>
#include <IMP/core/internal/CoreListSingletonContainer.h>

#include <climits>

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/algorithm/string.hpp>

#include <limits>

IMPCORE_BEGIN_NAMESPACE


bool ExperimentalTree::is_consistent(size_t node_index) const
{
  const Node *node = get_node(node_index);
  const Node::Label &label = node->get_label();
  // the node must have fewer proteins than each of its parents
  for ( size_t i = 0; i < node->get_number_of_parents(); ++i )
  {
    const Node *parent = get_node(node->get_parent(i));
    const Node::Label &parent_label = parent->get_label();
    size_t pi = 0, ci = 0;
    while ( ci < label.size() && pi < parent_label.size() )
    {
      // skip proteins that the parent has but the child does not have
      while ( pi < parent_label.size() &&
              parent_label[pi].first < label[ci].first )
        ++pi;
      if ( pi == parent_label.size() || parent_label[pi].first
              != label[ci].first )
      {
        // this means the child has more proteins than parent,
        // example: parent = "AA", child = "AAB"
        return false;
      }
      if ( parent_label[pi].second < label[ci].second )
      {
        // this means the child has more instances of a given protein,
        // example: parent = "AA", child = "AAA"
        return false;
      }
      ++pi;
      ++ci;
    }
    // last check - child has to have at least one fewer proteins
    // than every parent
    int parent_total = 0, child_total = 0;
    for ( size_t j = 0; j < label.size(); ++j )
      child_total += label[j].second;
    for ( size_t j = 0; j < parent_label.size(); ++j )
      parent_total += parent_label[j].second;
    if ( child_total >= parent_total )
      return false;
  }
  return true;
}


size_t ExperimentalTree::add_node(const std::string &desc)
{
  if ( finalized_ )
    IMP_THROW("Cannot add new nodes to finalized tree", IMP::ValueException);
  Node node;
  desc_to_label(desc, node.label_);
  size_t idx = nodes_.size();
  nodes_.push_back(node);
  return idx;
}


void ExperimentalTree::connect(size_t parent, size_t child)
{
  if ( finalized_ )
    IMP_THROW("Cannot add new edges to finalized tree", IMP::ValueException);
  nodes_[parent].children_.push_back(child);
  nodes_[child].parents_.push_back(parent);
}


void ExperimentalTree::finalize()
{
  if ( finalized_ )
    return;
  for ( size_t i=0; i<nodes_.size(); ++i )
  {
    if ( nodes_[i].is_root() )
    {
      if ( root_ == size_t(-1) )
        root_ = i;
      else
        IMP_THROW("Experimental tree has multiple roots", IMP::ValueException);
    }
  }
  if ( find_cycle(root_) )
    IMP_THROW("Experimental tree has a cycle", IMP::ValueException);
  for ( size_t i = 0; i < nodes_.size(); ++i )
    if ( !is_consistent(i) )
    {
      IMP_THROW("Experimental tree is inconsistent: a child has to "
        "have fewer proteins than its parent", IMP::ValueException);
    }
  finalized_ = true;
}


bool ExperimentalTree::find_cycle(size_t node_index)
{
  Node &node = nodes_[node_index];
  if ( node.visited_ )
    return true;
  node.visited_ = true;
  bool cycle_found = false;
  for ( size_t i = 0; i < node.get_number_of_children(); ++i )
    if ( find_cycle(node.get_child(i)) )
    {
      cycle_found = true;
      break;
    }
  node.visited_ = false;
  return cycle_found;
}


size_t ExperimentalTree::classify_protein(
    const std::string &desc) const
{
  ProteinMap::const_iterator p = protein_to_id_.find(desc);
  if ( p == protein_to_id_.end() )
    return size_t(-1);
  return p->second;
}


size_t ExperimentalTree::protein_to_id(
    const std::string &desc)
{
  size_t id = classify_protein(desc);
  if ( id == size_t(-1) )
  {
    id = number_of_proteins_++;
    protein_to_id_[desc] = id;
    id_to_protein_.push_back(desc);
  }
  return id;
}


void ExperimentalTree::desc_to_label(const std::string &desc,
    Node::Label &label)
{
  label.clear();
  std::vector<std::string> protein_names;
  boost::split(protein_names, desc, boost::is_space());
  std::sort(protein_names.begin(), protein_names.end());
  for ( size_t i=0; i<protein_names.size(); ++i )
    if ( !protein_names[i].empty() )
    {
      size_t id = protein_to_id(protein_names[i]);
      if ( label.empty() || label.back().first != id )
        label.push_back(std::make_pair(id, 1));
      else
        label.back().second++;
    }
  std::sort(label.begin(), label.end());
}



MSConnectivityRestraint::MSConnectivityRestraint(PairScore *ps,
  ExperimentalTree *tree, double eps,
                                             SingletonContainer *sc):
  Restraint("MSConnectivityRestraint %1%"),
  ps_(ps),
  tree_(tree),
  eps_(eps)
{
  if (sc) {
    sc_= sc;
  } else {
  }
  tree_->finalize();
}



namespace
{



class ParticleMatrix
{
public:

  class ParticleData
  {
  public:
    ParticleData(Particle *p, size_t id)
      : particle_(p)
      , id_(id)
    {}

    Particle *get_particle() const
    {
      return particle_;
    }

    size_t get_id() const
    {
      return id_;
    }
  private:
    Particle *particle_;
    size_t id_;
  };

  ParticleMatrix(size_t number_of_classes)
    : protein_by_class_(number_of_classes)
    , min_distance_(std::numeric_limits<double>::max())
    , max_distance_(0)
  {}

  ParticleMatrix()
    : min_distance_(std::numeric_limits<double>::max())
    , max_distance_(0)
  {}

  void resize(size_t number_of_classes)
  {
    protein_by_class_.resize(number_of_classes);
  }

  size_t add_particle(Particle *p, size_t id);
  void create_distance_matrix(const PairScore *ps);
  void clear_particles()
  {
    particles_.clear();
    for ( size_t i = 0; i < protein_by_class_.size(); ++i )
      protein_by_class_[i].clear();
  }
  size_t size() const
  {
    return particles_.size();
  }
  size_t get_number_of_classes() const
  {
    return protein_by_class_.size();
  }
  double get_distance(size_t p1, size_t p2) const
  {
    return dist_matrix_[p1*size() + p2];
  }
  std::vector<size_t> const &get_ordered_neighbors(size_t p) const
  {
    return order_[p];
  }
  ParticleData const &get_particle(size_t p) const
  {
    return particles_[p];
  }
  std::vector<size_t> const &get_all_proteins_in_class(
      size_t id) const
  {
    return protein_by_class_[id];
  }
  double max_distance() const
  {
    return max_distance_;
  }
  double min_distance() const
  {
    return min_distance_;
  }
private:
  class DistCompare
  {
  public:
    DistCompare(size_t source, ParticleMatrix const &parent)
      : parent_(parent)
      , source_(source)
    {}

    bool operator()(size_t p1, size_t p2) const
    {
      return parent_.get_distance(source_, p1) <
        parent_.get_distance(source_, p2);
    }
  private:
    ParticleMatrix const &parent_;
    size_t source_;
  };

  std::vector<ParticleData> particles_;
  std::vector<double> dist_matrix_;
  std::vector< std::vector<size_t> > order_;
  std::vector< std::vector<size_t> > protein_by_class_;
  double min_distance_;
  double max_distance_;
};


size_t ParticleMatrix::add_particle(Particle *p, size_t id)
{
  int n = particles_.size();
  particles_.push_back(ParticleData(p, id));
  if ( id < protein_by_class_.size() )
    protein_by_class_[id].push_back(n);
  return n;
}


void ParticleMatrix::create_distance_matrix(const PairScore *ps)
{
  size_t n = size();
  dist_matrix_.resize(n*n);
  for ( size_t r = 0; r < n; ++r )
    for ( size_t c = r; c < n; ++ c )
    {
      double d = ps->evaluate(ParticlePair(particles_[r].get_particle(),
        particles_[c].get_particle()), 0);
      dist_matrix_[r*n + c] = dist_matrix_[c*n + r] = d;
      min_distance_ = std::min(min_distance_, d);
      max_distance_ = std::max(max_distance_, d);
    }
  order_.clear();
  order_.resize(n);
  for ( size_t i = 0; i < n; ++i )
  {
    for ( size_t j = 0; j < n; ++j )
      if ( j != i )
        order_[i].push_back(j);
    std::sort(order_[i].begin(), order_[i].end(), DistCompare(i, *this));
  }
}


typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::property<boost::vertex_name_t, size_t> VertexId;
typedef boost::adjacency_list<boost::setS, boost::vecS,
        boost::undirectedS, VertexId, EdgeWeight> NNGraph;


class Tuples
{
public:
  Tuples(const std::vector<size_t> &elements, size_t k)
    : current_tuple_(k)
    , k_(k)
  {
    if ( k > 0 )
      generate_all_tuples(0, 0, elements);
  }
  size_t size() const
  {
    return all_tuples_.size();
  }
  std::vector<size_t> const &get_tuple(size_t i) const
  {
    return all_tuples_[i];
  }

private:
  void generate_all_tuples(size_t idx, size_t start,
      const std::vector<size_t> &elements);

  std::vector<size_t> current_tuple_;
  std::vector< std::vector<size_t> > all_tuples_;
  size_t k_;
};


void Tuples::generate_all_tuples(size_t idx, size_t start,
    const std::vector<size_t> &elements)
{
  if ( idx == k_ )
  {
    all_tuples_.push_back(current_tuple_);
    return;
  }
  for ( size_t i = start; i < elements.size(); ++i )
  {
    current_tuple_[idx] = elements[i];
    generate_all_tuples(idx + 1, i + 1, elements);
  }
}


class Assignment
{
public:
  Assignment(std::vector<Tuples> const &tuples)
    : current_assignment_(tuples.size())
    , tuples_(tuples)
  {
    for ( size_t i=0; i<current_assignment_.size(); ++i )
      if ( tuples[i].size() == 0 )
        current_assignment_[i] = -1;
  }

  size_t const &operator[](size_t i) const
  {
    return current_assignment_[i];
  }

  bool empty() const
  {
    for ( size_t i=0; i<current_assignment_.size(); ++i )
      if ( current_assignment_[i] != size_t(-1) )
        return false;
    return true;
  }

  size_t size() const
  {
    return current_assignment_.size();
  }

  bool next();

private:
  std::vector<size_t> current_assignment_;
  std::vector<Tuples> const &tuples_;
};


bool Assignment::next()
{
  for ( size_t i = 0; i < current_assignment_.size(); ++i )
  {
    if ( current_assignment_[i] == size_t(-1) )
      continue;
    ++current_assignment_[i];
    if ( current_assignment_[i] < tuples_[i].size() )
      return true;
    current_assignment_[i] = 0;
  }
  return false;
}


bool is_connected(NNGraph &G)
{
  std::vector<int> components(num_vertices(G));
  return boost::connected_components(G, &components[0]) == 1;
}


typedef std::set< std::pair<size_t, size_t> > EdgeSet;


EdgeSet mst(NNGraph &G)
{
  boost::property_map<NNGraph, boost::vertex_name_t>::type vertex_id =
    boost::get(boost::vertex_name, G);
  typedef boost::graph_traits<NNGraph>::edge_descriptor Edge;
  std::vector<Edge> spanning_tree;
  boost::kruskal_minimum_spanning_tree(G, std::back_inserter(spanning_tree));
  EdgeSet edges;
  for ( size_t i = 0; i < spanning_tree.size(); ++i )
  {
    size_t src = boost::get(vertex_id, source(spanning_tree[i], G));
    size_t dst = boost::get(vertex_id, target(spanning_tree[i], G));
    edges.insert(std::make_pair(src, dst));
  }
  return edges;
}

}


class MSConnectivityScore
{
public:
  MSConnectivityScore(const ExperimentalTree &tree,
    const SingletonContainer *sc,
    const PairScore *ps, double eps);
  double score(DerivativeAccumulator *accum) const;
  EdgeSet get_connected_pairs() const;
  Particle *get_particle(size_t p) const
  {
    return pm_.get_particle(p).get_particle();
  }

private:
  NNGraph create_nn_graph(double threshold) const;
  NNGraph build_subgraph_from_assignment(NNGraph &G,
    Assignment const &assignment, std::vector<Tuples> const &all_tuples) const;
  bool check_assignment(NNGraph &G, size_t node_handle,
    Assignment const &assignment,
    std::vector<Tuples> const &all_tuples,
    EdgeSet &picked) const;
  bool perform_search(NNGraph &G, EdgeSet &picked) const;
  NNGraph pick_graph(EdgeSet const &picked) const;
  NNGraph find_threshold() const;

  ParticleMatrix pm_;
  const PairScore *ps_;
  const ExperimentalTree &tree_;
  double eps_;
};


MSConnectivityScore::MSConnectivityScore(const ExperimentalTree &tree,
  const SingletonContainer *sc, const PairScore *ps, double eps)
    : pm_(tree.get_number_of_classes())
    , ps_(ps)
    , tree_(tree)
    , eps_(eps)
{
  StringKey id_key("id");
  for ( unsigned i = 0; i < sc->get_number_of_particles(); ++i )
  {
    Particle *p = sc->get_particle(i);
    size_t id = tree.classify_protein(p->get_value(id_key));
    if ( id == size_t(-1) )
      continue;
    pm_.add_particle(p, id);
  }
  pm_.create_distance_matrix(ps);
}


NNGraph MSConnectivityScore::create_nn_graph(double threshold) const
{
  size_t n = pm_.size();
  NNGraph G(n);
  boost::property_map<NNGraph, boost::vertex_name_t>::type vertex_id =
    boost::get(boost::vertex_name, G);
  boost::property_map<NNGraph, boost::edge_weight_t>::type dist =
    boost::get(boost::edge_weight, G);
  for ( size_t i = 0; i < n; ++i )
  {
    boost::put(vertex_id, i, i);
    std::vector<size_t> const &neighbors = pm_.get_ordered_neighbors(i);
    for ( size_t j = 0; j < neighbors.size(); ++j )
    {
      double d = pm_.get_distance(i, neighbors[j]);
      if ( d > threshold )
        break;
      NNGraph::edge_descriptor e = boost::add_edge(i, neighbors[j], G).first;
      boost::put(dist, e, d);
    }
  }
  return G;
}


NNGraph MSConnectivityScore::build_subgraph_from_assignment(NNGraph &G,
    Assignment const &assignment, std::vector<Tuples> const &all_tuples) const
{
  size_t num_particles = pm_.size();
  std::vector<size_t> vertices;
  for ( size_t i = 0; i < assignment.size(); ++i )
    if ( assignment[i] != size_t(-1) )
    {
      std::vector<size_t> const &conf =
        all_tuples[i].get_tuple(assignment[i]);
      for ( size_t j = 0; j < conf.size(); ++j )
        vertices.push_back(conf[j]);
    }
  boost::property_map<NNGraph, boost::vertex_name_t>::type vertex_id =
    boost::get(boost::vertex_name, G);
  boost::property_map<NNGraph, boost::edge_weight_t>::type dist =
    boost::get(boost::edge_weight, G);
  NNGraph ng(vertices.size());
  boost::property_map<NNGraph, boost::vertex_name_t>::type new_vertex_id =
    boost::get(boost::vertex_name, ng);
  boost::property_map<NNGraph, boost::edge_weight_t>::type new_dist =
    boost::get(boost::edge_weight, ng);
  for ( size_t i = 0; i < vertices.size(); ++i )
    boost::put(new_vertex_id, i, vertices[i]);
  std::vector<size_t> vertex_id_to_idx(num_particles, -1);
  for ( size_t i = 0; i < vertices.size(); ++i )
    vertex_id_to_idx[vertices[i]] = i;
  NNGraph::edge_iterator e, end;
  for ( boost::tie(e, end) = edges(G); e != end; ++e )
  {
    size_t source_id = boost::get(vertex_id, source(*e, G));
    size_t dest_id = boost::get(vertex_id, target(*e, G));
    size_t p_src = vertex_id_to_idx[source_id];
    size_t p_dst = vertex_id_to_idx[dest_id];
    if ( p_src == size_t(-1) || p_dst == size_t(-1) )
      continue;
    NNGraph::edge_descriptor ed = boost::add_edge(p_src, p_dst, ng).first;
    double d = boost::get(dist, *e);
    boost::put(new_dist, ed, d);
  }
  return ng;
}


bool MSConnectivityScore::check_assignment(NNGraph &G, size_t node_handle,
    Assignment const &assignment,
    std::vector<Tuples> const &all_tuples,
    EdgeSet &picked) const
{
  ExperimentalTree::Node const *node = tree_.get_node(node_handle);
  ExperimentalTree::Node::Label const &lb = node->get_label();
  std::vector<Tuples> new_tuples;
  std::vector<size_t> empty_vector;
  for ( size_t i = 0; i < lb.size(); ++i )
  {
    int prot_count = lb[i].second;
    size_t id = lb[i].first;
    while ( new_tuples.size() < id )
      new_tuples.push_back(Tuples(empty_vector, 0));
    if ( prot_count > 0 )
    {
      if ( assignment[id] != size_t(-1) )
      {
        std::vector<size_t> const &configuration =
          all_tuples[id].get_tuple(assignment[id]);
        if ( prot_count > int(configuration.size()) )
        {
          IMP_THROW("Experimental tree is inconsistent", IMP::ValueException);
        }
        new_tuples.push_back(Tuples(configuration, prot_count));
      }
      else
      {
        IMP_THROW("Experimental tree is inconsistent", IMP::ValueException);
      }
    }
    else
      new_tuples.push_back(Tuples(empty_vector, 0));
  }
  while ( new_tuples.size() < pm_.get_number_of_classes() )
    new_tuples.push_back(Tuples(empty_vector, 0));
  Assignment new_assignment(new_tuples);
  if ( new_assignment.empty() )
    return false;
  do
  {
    NNGraph ng = build_subgraph_from_assignment(G, new_assignment,
        new_tuples);
    if ( is_connected(ng) )
    {
      EdgeSet n_picked = mst(ng);
      bool good = true;
      for ( size_t i = 0; i < node->get_number_of_children(); ++i )
      {
        size_t child_handle = node->get_child(i);
        if ( !check_assignment(ng, child_handle, new_assignment,
              new_tuples, n_picked) )
        {
          good = false;
          break;
        }
      }
      if ( good )
      {
        picked.insert(n_picked.begin(), n_picked.end());
        return true;
      }
    }
  } while ( new_assignment.next() );
  return false;
}


bool MSConnectivityScore::perform_search(NNGraph &G,
  EdgeSet &picked) const
{
  size_t root_handle = tree_.get_root();
  ExperimentalTree::Node const *node = tree_.get_node(root_handle);
  ExperimentalTree::Node::Label const &lb = node->get_label();
  std::vector<Tuples> tuples;
  std::vector<size_t> empty_vector;
  for ( size_t i = 0; i < lb.size(); ++i )
  {
    int prot_count = lb[i].second;
    size_t id = lb[i].first;
    while ( tuples.size() < id )
      tuples.push_back(Tuples(empty_vector, 0));
    if ( prot_count > 0 )
    {
      tuples.push_back(Tuples(pm_.get_all_proteins_in_class(id),
            prot_count));
    }
    else
      tuples.push_back(Tuples(empty_vector, 0));
  }
  while ( tuples.size() < pm_.get_number_of_classes() )
    tuples.push_back(Tuples(empty_vector, 0));
  Assignment assignment(tuples);
  if ( assignment.empty() )
    return false;
  do
  {
    NNGraph ng = build_subgraph_from_assignment(G, assignment, tuples);
    if ( is_connected(ng) )
    {
      EdgeSet n_picked = mst(ng);
      bool good = true;
      for ( size_t i = 0; i < node->get_number_of_children(); ++i )
      {
        size_t child_handle = node->get_child(i);
        if ( !check_assignment(ng, child_handle, assignment,
              tuples, n_picked) )
        {
          good = false;
          break;
        }
      }
      if ( good )
      {
        picked.insert(n_picked.begin(), n_picked.end());
        return true;
      }
    }
  } while ( assignment.next() );
  return false;
}


NNGraph MSConnectivityScore::pick_graph(EdgeSet const &picked) const
{
  EdgeSet::const_iterator p;
  std::map<size_t, int> idx_to_vtx;
  int n_vert = 0;
  for ( p = picked.begin(); p != picked.end(); ++p )
  {
    std::map<size_t, int>::iterator q = idx_to_vtx.find(p->first);
    if ( q == idx_to_vtx.end() )
      idx_to_vtx[p->first] = n_vert++;
    q = idx_to_vtx.find(p->second);
    if ( q == idx_to_vtx.end() )
      idx_to_vtx[p->second] = n_vert++;
  }
  NNGraph ng(n_vert);
  boost::property_map<NNGraph, boost::vertex_name_t>::type vertex_id =
    boost::get(boost::vertex_name, ng);
  boost::property_map<NNGraph, boost::edge_weight_t>::type dist =
    boost::get(boost::edge_weight, ng);
  for ( std::map<size_t, int>::iterator q = idx_to_vtx.begin();
      q != idx_to_vtx.end(); ++q )
  {
    boost::put(vertex_id, q->second, q->first);
  }
  for ( p = picked.begin(); p != picked.end(); ++p )
  {
    NNGraph::edge_descriptor e = boost::add_edge(idx_to_vtx[p->first],
        idx_to_vtx[p->second], ng).first;
    double d = pm_.get_distance(p->first, p->second);
    boost::put(dist, e, d);
  }
  return ng;
}


NNGraph MSConnectivityScore::find_threshold() const
{
  double max_dist = 1.1*pm_.max_distance(), min_dist = pm_.min_distance();
  NNGraph g = create_nn_graph(min_dist);
  {
    std::set< std::pair<size_t, size_t> > picked;
    if ( perform_search(g, picked) )
    {
      return pick_graph(picked);
    }
    g = create_nn_graph(max_dist);
    if ( !perform_search(g, picked) )
    {
      IMP_THROW("Cannot build a nearest neighbor graph", IMP::ValueException);
    }
  }
  EdgeSet picked;
  while ( max_dist - min_dist > eps_ )
  {
    double dist = 0.5*(max_dist + min_dist);
    g = create_nn_graph(dist);
    EdgeSet tmp_picked;
    if ( perform_search(g, tmp_picked) )
    {
      picked.swap(tmp_picked);
      max_dist = dist;
    }
    else
      min_dist = dist;
  }
  return pick_graph(picked);
}


double MSConnectivityScore::score(DerivativeAccumulator *accum) const
{
  EdgeSet edges = get_connected_pairs();
  double sc = 0;
  for ( EdgeSet::iterator p = edges.begin();
    p != edges.end(); ++p )
  {
    if ( accum )
    {
      sc += ps_->evaluate(ParticlePair(
        pm_.get_particle(p->first).get_particle(),
        pm_.get_particle(p->second).get_particle()), accum);
    }
    else
    {
      sc += pm_.get_distance(p->first, p->second);
    }
  }
  return sc;
}


EdgeSet MSConnectivityScore::get_connected_pairs() const
{
  NNGraph g = find_threshold();
  EdgeSet edges = mst(g);
  return edges;
}


namespace {
  internal::CoreListSingletonContainer* ms_get_list(SingletonContainer *sc) {
    internal::CoreListSingletonContainer *ret
      = dynamic_cast<internal::CoreListSingletonContainer*>(sc);
    if (!ret) {
      IMP_THROW("Can only use the set and add methods when no container"
                << " was passed on construction of MSConnectivityRestraint.",
                ValueException);
    }
    return ret;
  }
}

void MSConnectivityRestraint::set_particles(const Particles &ps) {
  if (!sc_ && !ps.empty()) {
    sc_= new internal::CoreListSingletonContainer(ps[0]->get_model(),
                                                  "msconnectivity list");
  }
  ms_get_list(sc_)->set_particles(ps);
}

void MSConnectivityRestraint::add_particles(const Particles &ps) {
  if (!sc_&& !ps.empty()) {
    sc_= new internal::CoreListSingletonContainer(ps[0]->get_model(),
                                                  "msconnectivity list");
  }
  ms_get_list(sc_)->add_particles(ps);
}

void MSConnectivityRestraint::add_particle(Particle *ps) {
  if (!sc_) {
    sc_= new internal::CoreListSingletonContainer(ps->get_model(),
                                                  "msconnectivity list");
  }
  ms_get_list(sc_)->add_particle(ps);
}


double
MSConnectivityRestraint::unprotected_evaluate(
  DerivativeAccumulator *accum) const
{
  IMP_CHECK_OBJECT(ps_.get());
  IMP_OBJECT_LOG;
  MSConnectivityScore mcs(*tree_, sc_.get(), ps_.get(), eps_);
  return mcs.score(accum);
}


Restraints MSConnectivityRestraint::get_instant_decomposition() const {
  ParticlePairs pp= get_connected_pairs();
  Restraints ret(pp.size());
  for (unsigned int i=0; i< pp.size(); ++i) {
    IMP_NEW(PairRestraint, pr, (ps_, pp[i]));
    std::ostringstream oss;
    oss << get_name() << " " << i;
    pr->set_name(oss.str());
  }
  return ret;
}


ParticlePairs MSConnectivityRestraint::get_connected_pairs() const {
  IMP_CHECK_OBJECT(ps_.get());
  MSConnectivityScore mcs(*tree_, sc_.get(), ps_.get(), eps_);
  EdgeSet edges = mcs.get_connected_pairs();
  ParticlePairs ret(edges.size());
  unsigned index = 0;
  for ( EdgeSet::iterator p = edges.begin(); p != edges.end(); ++p )
  {
    ret.set(index++, ParticlePair(mcs.get_particle(p->first),
      mcs.get_particle(p->second)));
  }
  return ret;
}


ParticlesTemp MSConnectivityRestraint::get_input_particles() const {
  if (!sc_) return ParticlesTemp();
  ParticlesTemp ret;
  IMP_FOREACH_SINGLETON(sc_, {
      ParticlesTemp cs = ps_->get_input_particles(_1);
      ret.insert(ret.end(), cs.begin(), cs.end());
    });
  return ret;
}

ContainersTemp MSConnectivityRestraint::get_input_containers() const {
  if (!sc_) return ContainersTemp();
  ContainersTemp ret;
  IMP_FOREACH_SINGLETON(sc_, {
      ContainersTemp cs
        = ps_->get_input_containers(_1);
      ret.insert(ret.end(), cs.begin(), cs.end());
    });
  return ret;
}


void MSConnectivityRestraint::do_show(std::ostream& out) const
{
  if (!sc_) {
    out << "container is NULL" << std::endl;
  } else {
    out << "container is " << *sc_ << std::endl;
  }
}

IMPCORE_END_NAMESPACE
