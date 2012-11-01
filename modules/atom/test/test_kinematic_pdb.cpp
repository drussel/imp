/**
   This is the program for creating a simple kinematic tree

*/
#include <IMP/Model.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/atom/pdb.h>
#include <IMP/atom/Hierarchy.h>
#include <IMP/core/KinematicForest.h>
#include <IMP/core/rigid_bodies.h>
#include <IMP/core/joints.h>
#include <string>

void stupid_test()
{
  using namespace IMP::core;
  //  RigidBody a = RigidBody::setup(0,0,1);
  //  RigidBody b(0,0,2);
  //  Rig(0,0,3);

}

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

  // read pdb
  IMP::Model *model = new IMP::Model();
  IMP::atom::Hierarchy mhd;
  mhd = IMP::atom::read_pdb(fname,
                            model,
                            new IMP::atom::NonWaterNonHydrogenPDBSelector(),
                            // don't add radii
                            true, true);

  // create rigid bodies
  IMP::atom::Hierarchies hr =
    IMP::atom::get_by_type(mhd, IMP::atom::RESIDUE_TYPE);

  std::cerr << hr.size() << " residues were read from pdb file" << std::endl;

  IMP::core::RigidBodies rbs;
  for(unsigned int i=0; i<hr.size(); i++)
    rbs.push_back(IMP::atom::create_rigid_body(hr[i]));

  std::cout << "initial coords "
            << rbs[0].get_coordinates()
            << ", "
            << rbs[1].get_coordinates()
            << std::endl;

  IMP_NEW(IMP::core::PrismaticJoint, pj, (rbs[0], rbs[1]));

  IMP_NEW(IMP::core::KinematicForest, kf, (model) );
  kf->add_edge(pj);

  std::cout << "coords after KinematicForest ctr "
            << kf->get_coordinates_safe( rbs[0] )
            << ", "
            << kf->get_coordinates_safe( rbs[1] )
            << std::endl;

  pj->set_length(10.0);
  std::cerr << "coords after set_length(10.0) " << std::endl;
        std::cerr    << kf->get_coordinates_safe( rbs[0] )
            << ", "
            << kf->get_coordinates_safe( rbs[1] )
            << std::endl;

  //IMP::atom::write_pdb
  //  (mhd, "./after_set_length_10_and_get_coords_safe.pdb");

  //
  pj->set_length(20.0);
  kf->update_all_external_coordinates();
  //IMP::atom::write_pdb
  // (mhd, "./after_set_length_20_and_update_all_external.pdb");

  // test that set_coords safe works
  kf->set_coordinates_safe
    ( rbs[0], IMP::algebra::Vector3D(0,0,3) );
  std::cout << "rb[0] coords after set_coords_safe "
            << rbs[0].get_coordinates()
            << std::endl;
  IMP::atom::write_pdb
    (mhd, "./after_set_coords_safe_rb0__0_0_3.pdb");
  std::cout << "updated length = "
            << pj->get_length()
            << std::endl;
}
