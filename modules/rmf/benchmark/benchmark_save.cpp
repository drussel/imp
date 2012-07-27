/**
 * Copyright 2007-2012 IMP Inventors. All rights reserved.
 */
#include <IMP/rmf/atom_io.h>
#include <IMP/rmf/frames.h>
#include <IMP/atom/pdb.h>
#include <RMF/FileHandle.h>
#include <IMP/benchmark/benchmark_macros.h>
#include <IMP/benchmark/utility.h>
#include <IMP/benchmark/command_line_macros.h>

void test_it(std::string file_name, std::string test_name,
             IMP::atom::Hierarchy h) {
  RMF::FileHandle fh= RMF::create_rmf_file(file_name);
  IMP::rmf::add_hierarchies(fh, IMP::atom::Hierarchies(1, h));
  double time;
  int nframes=0;
  IMP_TIME({IMP::rmf::save_frame(fh, nframes); ++nframes;}, time);
  IMP::benchmark::report("rmf save", test_name, time, 1);
}

int main(int argc, char *argv[]) {
  try {
    IMP_BENCHMARK( );
    IMP_NEW(IMP::Model, m, ());
    IMP::atom::Hierarchy h
        = IMP::atom::read_pdb(IMP::rmf::get_data_path("huge.pdb"), m);
    test_it(IMP::base::create_temporary_file_name("benchmark_save", ".rmf"),
            "rmf", h);
#ifdef RMF_USE_PROTOBUF
    test_it(IMP::base::create_temporary_file_name("benchmark_save", ".prmf"),
            "prmf", h);
#endif
  } catch (const IMP::base::Exception &e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
  }
  return 0;
}
