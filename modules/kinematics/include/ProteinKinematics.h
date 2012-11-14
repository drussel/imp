/**
 * \file ProteinKinematics
 * \brief functionality for defining a kinematic forest for proteins
 *
 * Copyright 2007-2012 IMP Inventors. All rights reserved.
 *  \authors Dina Schneidman, Barak Raveh
 *
 */

#ifndef IMPKINEMATICS_PROTEIN_KINEMATICS_H
#define IMPKINEMATICS_PROTEIN_KINEMATICS_H

#include "kinematics_config.h"

#include <IMP/kinematics/revolute_joints.h>

#include <IMP/core/rigid_bodies.h>
#include <IMP/atom/Atom.h>

#include <vector>
#include <iostream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/undirected_dfs.hpp>

IMPKINEMATICS_BEGIN_NAMESPACE

typedef boost::adjacency_list <boost::vecS,
                               boost::vecS,
                               boost::undirectedS,
                               boost::no_property,
    boost::property<boost::edge_color_t, boost::default_color_type> > Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor MyVertex;

class IMPKINEMATICSEXPORT ProteinKinematics {
public:
  /* Constructors */

  // all phi/psi rotatable
  ProteinKinematics(IMP::atom::Hierarchy mhd);

  // only torsions from dihedral_angles list are rotatable
  ProteinKinematics(IMP::atom::Hierarchy mhd,
                    const std::vector<IMP::atom::Atoms>& dihedral_angles);


  /* Access methods */

  double get_phi(const IMP::atom::Residue r) const {
    return get_phi_joint(r)->get_angle();
  }

  double get_psi(const IMP::atom::Residue r) const {
    return get_psi_joint(r)->get_angle();
  }

  // TODO: add chi

  /* Modifier methods */

  void set_phi(const IMP::atom::Residue r, double angle) {
    get_phi_joint(r)->set_angle(angle);
  }

  void set_psi(const IMP::atom::Residue r, double angle) {
    get_psi_joint(r)->set_angle(angle);
  }

  // TODO: add chi


private:
  void build_topology_graph();

  void mark_rotatable_angles(
                        const std::vector<IMP::atom::Atoms>& dihedral_angles);

  void build_rigid_bodies();
  void add_joints(const std::vector<IMP::atom::Atoms>& dihedral_angles);

  /* Joint access methods */
  DihedralAngleRevoluteJoint* get_phi_joint(const IMP::atom::Residue r) const;

  DihedralAngleRevoluteJoint* get_psi_joint(const IMP::atom::Residue r) const;

  DihedralAngleRevoluteJoints get_joints(const IMP::atom::Residue r) const;

 private:
  // protein hierarchy
  IMP::atom::Hierarchy mhd_;

  // atom particles
  IMP::ParticlesTemp atom_particles_;

  // topology graph: nodes = atoms, edges = bonds
  Graph graph_;

  // mapping between atom ParticleIndex and node number in the graph
  IMP::compatibility::map<IMP::ParticleIndex, int> particle_index_to_node_map_;
  IMP::compatibility::vector<IMP::ParticleIndex> node_to_particle_index_map_;

  // rigid bodies
  IMP::core::RigidBodies rbs_;

  // joints
  std::vector<DihedralAngleRevoluteJoint*> joints_;

  // map between residue phi/psi and joints
};

IMPKINEMATICS_END_NAMESPACE

#endif /* IMPKINEMATICS_PROTEIN_KINEMATICS_H */
