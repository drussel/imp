/**
   This is the program for creating a simple kinematic tree

*/
#include <IMP/Model.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/atom/pdb.h>
#include <IMP/atom/Hierarchy.h>
#include "KinematicForest.h"
#include <IMP/core/rigid_bodies.h>
#include "joints.h"
#include <string>


int main(int argc, char **argv)
{
  // output arguments
  for (int i = 0; i < argc; i++) std::cerr << argv[i] << " ";
  std::cerr << std::endl;

  if(argc != 2) {
    std::cerr << "Usage: " << argv[0] << " pdb " << std::endl;
    exit(1);
  }
  std::string fname(argv[1]);
  std::cout << fname << std::endl;

  // read pdb
  IMP::Model* model = new IMP::Model();
  IMP::atom::Hierarchy mhd;
  mhd = IMP::atom::read_pdb(fname,
                            model,
                            new IMP::atom::NonWaterNonHydrogenPDBSelector(),
                            // don't add radii
                            true, true);
   IMP::FloatKey radius_key= IMP::FloatKey("radius");

   // create rigid bodies
   IMP::ParticlesTemp hr =
     IMP::atom::get_by_type(mhd, IMP::atom::RESIDUE_TYPE);

   IMP::ParticlesTemp ps = get_by_type(mhd, IMP::atom::ATOM_TYPE);
   for(unsigned int i=0; i<ps.size(); i++) {
     ps[i]->add_attribute(radius_key, 1.5);
   }

   std::cerr << hr.size() << " residues were read from pdb file" << std::endl;

   IMP::core::RigidBodies rbs;
   for(unsigned int i=0; i<hr.size(); i++) {
     rbs.push_back(IMP::atom::create_rigid_body(IMP::atom::Hierarchy(hr[i])));
     std::cerr << "Rigid body number " << i << " is " << rbs[i] << std::endl;
   }

   std::cout << "initial coords "
             << rbs[0].get_coordinates()
             << ", "
             << rbs[1].get_coordinates()
             << std::endl;

   IMP_NEW(IMP::core::PrismaticJoint, pj, (rbs[0], rbs[1]));
   IMP_NEW(IMP::core::TransformationJoint, tj, (rbs[1], rbs[2]));

   IMP_NEW(IMP::core::KinematicForest, kf, (model) );
   kf->add_edge(pj);
   kf->add_edge(tj);

   std::cout << "coords after KinematicForest ctr "
             << kf->get_coordinates_safe( rbs[0] )
             << ", "
             << kf->get_coordinates_safe( rbs[1] )
             << ", "
             << kf->get_coordinates_safe( rbs[2] )
             << std::endl;
   std::cerr << "Distance 0-1: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[0] ),
                                            kf->get_coordinates_safe( rbs[1] ) )
             << std::endl;
   std::cerr << "Distance 1-2: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[1] ),
                                            kf->get_coordinates_safe( rbs[2] ) )
             << std::endl;
   std::cout << "length 0-1 = "
             << pj->get_length()
             << std::endl;
   std::cout << "trans 0-1 = "
             << pj->get_transformation_child_to_parent()
             << std::endl;
   std::cout << "trans 1-2 = "
             << tj->get_transformation_child_to_parent()
             << std::endl;

   std::cout << "Setting length to 10.0: " << std::endl;

   pj->set_length(10.0);
   std::cerr << "coords after set_length(10.0) " << std::endl;
   std::cerr << kf->get_coordinates_safe( rbs[0] )
             << ", "
             << kf->get_coordinates_safe( rbs[1] )
             << ", "
             << kf->get_coordinates_safe( rbs[2] )
             << std::endl;
   std::cerr << "Distance 0-1: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[0] ),
                                            kf->get_coordinates_safe( rbs[1] ) )
             << std::endl;
   std::cerr << "Distance 1-2: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[1] ),
                                            kf->get_coordinates_safe( rbs[2] ) )
             << std::endl;

   std::cout << "length 0-1 = "
             << pj->get_length()
             << std::endl;
   std::cout << "trans 0-1 = "
             << pj->get_transformation_child_to_parent()
             << std::endl;
   std::cout << "trans 1-2 = "
             << tj->get_transformation_child_to_parent()
             << std::endl;

  //IMP::atom::write_pdb
  //  (mhd, "./after_set_length_10_and_get_coords_safe.pdb");

  //
   std::cout << "Setting length to 20.0: " << std::endl;
   pj->set_length(20.0);
   kf->update_all_external_coordinates();
   std::cerr << "Distance 0-1: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[0] ),
                                            kf->get_coordinates_safe( rbs[1] ) )
             << std::endl;
   std::cerr << "Distance 1-2: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[1] ),
                                            kf->get_coordinates_safe( rbs[2] ) )
             << std::endl;

   IMP::atom::write_pdb
    (mhd, "./after_set_length_20_and_update_all_external.pdb");

   std::cout << "length 0-1 = "
             << pj->get_length()
             << std::endl;
   std::cout << "trans 0-1 = "
             << pj->get_transformation_child_to_parent()
             << std::endl;
   std::cout << "trans 1-2 = "
             << tj->get_transformation_child_to_parent()
             << std::endl;

  // test that set_coords safe works
   std::cout << "setting rbs[0] coords to 0,0,3" << std::endl;
   kf->set_coordinates_safe
     ( rbs[0], IMP::algebra::Vector3D(0,0,3) );
   std::cout << "rb[0] coords after set_coords_safe "
             <<   kf->get_coordinates_safe( rbs[0] )           << std::endl;
   std::cout << "rb[1] coords "
             <<   kf->get_coordinates_safe( rbs[1] )           << std::endl;
   std::cout << "rb[2] coords "
             << kf->get_coordinates_safe( rbs[2] )
             << std::endl;
     IMP::atom::write_pdb
     (mhd, "./after_set_coords_safe_rb0__0_0_3.pdb");
   std::cout << "updated length 0-1 = "
             << pj->get_length()
             << std::endl;
   std::cout << "updated trans 0-1 = "
             << pj->get_transformation_child_to_parent()
             << std::endl;
   std::cout << "updated trans 1-2 = "
             << tj->get_transformation_child_to_parent()
             << std::endl;
   std::cerr << "Distance 0-1: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[0] ),
                                            kf->get_coordinates_safe( rbs[1] ) )
             << std::endl;
   std::cerr << "Distance 1-2: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[1] ),
                                            kf->get_coordinates_safe( rbs[2] ) )
             << std::endl;

  // test that set_coords safe works
   std::cout << "setting rbs[1] coords to 0,0,0" << std::endl;
   kf->set_coordinates_safe
     ( rbs[1], IMP::algebra::Vector3D(0,0,0) );
   std::cout << "rb[0] coords after set_coords_safe "
             <<   kf->get_coordinates_safe( rbs[0] )           << std::endl;
   std::cout << "rb[1] coords "
             <<   kf->get_coordinates_safe( rbs[1] )           << std::endl;
   std::cout << "rb[2] coords "
             << kf->get_coordinates_safe( rbs[2] )
             << std::endl;
     IMP::atom::write_pdb
     (mhd, "./after_set_coords_safe_rb1__0_0_0.pdb");
   std::cout << "updated length 0-1 = "
             << pj->get_length()
             << std::endl;
   std::cout << "updated trans 0-1 = "
             << pj->get_transformation_child_to_parent()
             << std::endl;
   std::cout << "updated trans 1-2 = "
             << tj->get_transformation_child_to_parent()
             << std::endl;
   std::cerr << "Distance 0-1: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[0] ),
                                            kf->get_coordinates_safe( rbs[1] ) )
             << std::endl;
   std::cerr << "Distance 1-2: "
             << IMP::algebra::get_distance( kf->get_coordinates_safe( rbs[1] ),
                                            kf->get_coordinates_safe( rbs[2] ) )
             << std::endl;
}
