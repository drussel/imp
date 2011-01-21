/**
 *  \fileatoms2anchors.cpp
 *  \brief Find an anchor graph segmentation of a protein
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
**/

#include <string>
#include "IMP/em/DensityMap.h"
#include "IMP/em/MRCReaderWriter.h"
#include "IMP/Model.h"
#include "IMP/atom/Hierarchy.h"
#include "IMP/atom/pdb.h"
#include "IMP/atom/force_fields.h"
#include <IMP/multifit/VQClustering.h>
#include <IMP/multifit/DataPoints.h>
#include <IMP/multifit/DataPointsAssignment.h>
#include <IMP/multifit/anchors_reader.h>
//others
#include <boost/program_options.hpp>
#include <boost/format.hpp>

using namespace IMP;

int parse_input(int argc, char *argv[],std::string &pdb_filename,
                int &num_means,std::string &cmm_filename,
                std::string &output_pdb_filename,std::string &seg_filename) {
  cmm_filename="";
  seg_filename="";
  std::stringstream usage;
  usage<<"The program segments all CA atoms into K clusters using a GMM "
       <<"procedure\n\nUsage: atoms2anchors <input pdb filename> "
       <<"<number of clusters> <output pdb filename>\n\n";
  boost::program_options::options_description optional_params("Allowed options")
    ,po,ao,required_params("Hideen options");
  required_params.add_options()
    ("pdb",boost::program_options::value<std::string>(&pdb_filename)
     ,"the name of the PDB file")
    ("num",boost::program_options::value<int>(&num_means)
     ,"number of points that describe the structure")
    ("output-pdb",boost::program_options::value<std::string>
     (&output_pdb_filename),"the output centers as CA atoms ina PDB file");
  optional_params.add_options()
    ("help", usage.str())
    ("cmm",boost::program_options::value<std::string>(&cmm_filename),
     "the output centers as points in a CMM file")
    ("seg",boost::program_options::value<std::string>(&seg_filename),
     "print each cluster as a PDB file <seg>_i.pdb");
  boost::program_options::positional_options_description p;
  p.add("pdb", 1);
  p.add("num", 1);
  p.add("output-pdb", 1);
  boost::program_options::options_description all;
  all.add(optional_params).add(required_params);

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::command_line_parser(
                       argc, argv).options(all).positional(p).run(),vm);
   boost::program_options::notify(vm);
   if (vm.count("help")) {
     std::cout << optional_params << "\n";
     return 1;
   }
   if (not (vm.count("pdb")+vm.count("num")+vm.count("output-pdb") == 3)) {
     std::cout<<optional_params<<std::endl;
     return 1;
   }

   set_log_level(VERBOSE);
   IMP_LOG(VERBOSE,"============= parameters ============"<<std::endl);
   IMP_LOG(VERBOSE,"pdb_filename : " << pdb_filename <<std::endl);
   IMP_LOG(VERBOSE,"num_centers : " << num_means <<std::endl);
   IMP_LOG(VERBOSE,"output_pdb_filename : " << output_pdb_filename <<std::endl);
   IMP_LOG(VERBOSE,"output_cmm_filename : " << cmm_filename <<std::endl);
 IMP_LOG(VERBOSE,"segment pdb files names : " << seg_filename << std::endl);
   IMP_LOG(VERBOSE,"====================================="<<std::endl);
   return 0;
}

int main(int argc, char *argv[]) {
  std::string pdb_filename,cmm_filename, output_pdb_filename,seg_filename;
  int num_means;
  if (parse_input(argc,argv,pdb_filename, num_means,cmm_filename,
                  output_pdb_filename,seg_filename)){
    exit(0);
  }

  //read the pdb
  Model *m = new Model();
  atom::Hierarchy mh;
  mh = atom::read_pdb(pdb_filename,m);
  multifit::ParticlesDataPoints ddp(core::get_leaves(mh));
  multifit::VQClustering vq(&ddp,num_means);
  vq.run();
  multifit::DataPointsAssignment assignment(&ddp,&vq);
  multifit::AnchorsData ad(
                          assignment.get_centers(),
                          *(assignment.get_edges()));
  multifit::write_pdb(output_pdb_filename,assignment);

  if (not (cmm_filename == "")) {
    multifit::write_cmm(cmm_filename,"anchor_graph",ad);
  }
  return 0;
}
