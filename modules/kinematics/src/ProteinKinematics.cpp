/**
 * \file ProteinKinematics
 * \brief functionality for defining a kinematic forest for proteins
 *
 * Copyright 2007-2012 IMP Inventors. All rights reserved.
 *  \authors Dina Schneidman, Barak Raveh
 *
 */

#include <IMP/kinematics/ProteinKinematics.h>

#include <IMP/exception.h>

#include <boost/graph/connected_components.hpp>

IMPKINEMATICS_BEGIN_NAMESPACE

ProteinKinematics::ProteinKinematics(IMP::atom::Hierarchy mhd,
                          const std::vector<IMP::atom::Atoms>& dihedral_angles):
  mhd_(mhd),
  atom_particles_(IMP::atom::get_by_type(mhd_, IMP::atom::ATOM_TYPE)),
  graph_(atom_particles_.size())
{

  build_topology_graph();
  mark_rotatable_angles(dihedral_angles);
  build_rigid_bodies();
  add_joints(dihedral_angles);
}

void ProteinKinematics::build_topology_graph() {

  // map graph nodes (=atoms) to ParticleIndex
  for(unsigned int i=0; i<atom_particles_.size(); i++) {
    IMP::ParticleIndex pindex = atom_particles_[i]->get_index();
    particle_index_to_node_map_[pindex] = i;
    node_to_particle_index_map_.push_back(pindex);
  }

  // add edges to graph
  IMP::atom::Bonds bonds = IMP::atom::get_internal_bonds(mhd_);
  for(unsigned int i=0; i<bonds.size(); i++) {
    IMP::atom::Bonded p1 = IMP::atom::Bond(bonds[i]).get_bonded(0);
    IMP::atom::Bonded p2 = IMP::atom::Bond(bonds[i]).get_bonded(1);
    int atom_index1 = particle_index_to_node_map_[p1->get_index()];
    int atom_index2 = particle_index_to_node_map_[p2->get_index()];
    boost::add_edge(atom_index1, atom_index2, graph_);
  }

  std::vector<int> component(boost::num_vertices(graph_));
  unsigned int num = boost::connected_components(graph_, &component[0]);
  std::cerr << "CC NUM before removal of rotatable bonds = "
            << num << std::endl;
}

void ProteinKinematics::mark_rotatable_angles(
                         const std::vector<IMP::atom::Atoms>& dihedral_angles) {
  for(unsigned int i=0; i<dihedral_angles.size(); i++) {

    // get the ParticleIndex and map it to graph node
    IMP::Particle* p1 = dihedral_angles[i][1].get_particle();
    IMP::Particle* p2 = dihedral_angles[i][2].get_particle();
    int atom_index1 = 0;
    int atom_index2 = 0;
    if(particle_index_to_node_map_.find(p1->get_index()) !=
       particle_index_to_node_map_.end()) {
      atom_index1 = particle_index_to_node_map_[p1->get_index()];
    } else {
      IMP_THROW("cannot find node index for angle", IMP::ValueException);
    }
    if(particle_index_to_node_map_.find(p2->get_index()) !=
       particle_index_to_node_map_.end()) {
      atom_index2 = particle_index_to_node_map_[p2->get_index()];
    } else {
      IMP_THROW("cannot find node index for angle", IMP::ValueException);
    }

    boost::remove_edge(atom_index1, atom_index2, graph_);
  }
}

void ProteinKinematics::build_rigid_bodies() {
  // compute connected components that represent rigid parts
  std::vector<int> component(boost::num_vertices(graph_));
  unsigned int num = boost::connected_components(graph_, &component[0]);
  std::cerr << "CC NUM = " << num << std::endl;

  // store the atoms of each rigid body using node index
  std::vector<std::vector<int> > rigid_bodies_atoms(num);
  for(unsigned int i=0; i<component.size(); i++) {
    rigid_bodies_atoms[component[i]].push_back(i);
  }

  // build the rigid bodies
  IMP::Model* m = mhd_->get_model();
  for(unsigned int i=0; i<rigid_bodies_atoms.size(); i++) {
    IMP::Particle *rbp= new IMP::Particle(m);
    std::string name = "rb_name"; // TODO: add rb id
    rbp->set_name(name);
    // rb atoms, get Particles from node indexes
    IMP::ParticlesTemp all;
    for(unsigned int j=0; j<rigid_bodies_atoms[i].size(); j++) {
      all.push_back(
        m->get_particle(node_to_particle_index_map_[rigid_bodies_atoms[i][j]]));
    }
    IMP::core::RigidBody rbd
      = IMP::core::RigidBody::setup_particle(rbp, core::XYZs(all));
    rbd.set_coordinates_are_optimized(true);
    rbs_.push_back(rbd);
  }
}

void ProteinKinematics::add_joints(
                      const std::vector<IMP::atom::Atoms>& dihedral_angles) {
  for(unsigned int i=0; i<dihedral_angles.size(); i++) {

    // get the ParticleIndex and map it to graph node
    IMP::Particle* p1 = dihedral_angles[i][1].get_particle();
    IMP::Particle* p2 = dihedral_angles[i][2].get_particle();

    if(IMP::core::RigidMember::particle_is_instance(p1) &&
       IMP::core::RigidMember::particle_is_instance(p2)) {
      IMP::core::RigidBody rb1 = IMP::core::RigidMember(p1).get_rigid_body();
      IMP::core::RigidBody rb2 = IMP::core::RigidMember(p2).get_rigid_body();

      IMP_NEW(IMP::kinematics::DihedralAngleRevoluteJoint, joint,
              (rb1, rb2,
               IMP::core::XYZ(dihedral_angles[i][0].get_particle()),
               IMP::core::XYZ(dihedral_angles[i][1].get_particle()),
               IMP::core::XYZ(dihedral_angles[i][2].get_particle()),
               IMP::core::XYZ(dihedral_angles[i][3].get_particle())));

      joints_.push_back(joint);

    } else {
      IMP_THROW("cannot find rigid bodies for dihedral angle",
                IMP::ValueException);
    }
  }
}

IMPKINEMATICS_END_NAMESPACE
