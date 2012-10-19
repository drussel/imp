/**
   This is the program for creating a simple kinematic tree

*/
#include <IMP/Model.h>
#include <IMP/atom/pdb.h>
#include <IMP/core/KinematicForest.h>

int main(int argc, char **argv)
{
  // output arguments
  for (int i = 0; i < argc; i++) std::cerr << argv[i] << " ";
  std::cerr << std::endl;

  if(argc != 2) { std::cerr << "Usage: " << argv[0] << " pdb " << std::endl; }

  // read pdb
  IMP::Model *model = new IMP::Model();
  IMP::atom::Hierarchy mhd;
  mhd = IMP::atom::read_pdb(files[i], model,
                            new IMP::atom::NonWaterNonHydrogenPDBSelector(),
                            // don't add radii
                            true, true);

  // create rigid bodies
  IMP::atom::Hierarchies pr = IMP::atom::get_by_type(mhd, RESIDUE_TYPE);
  IMP::atom::create_rigid_bodies(pr);
