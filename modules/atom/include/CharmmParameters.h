/**
 * \file atom/CharmmParameters.h \brief access to Charmm force field parameters
 *
 * Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */
#ifndef IMPATOM_CHARMM_PARAMETERS_H
#define IMPATOM_CHARMM_PARAMETERS_H

#include "ForceFieldParameters.h"

#include <string>
#include <fstream>

IMPATOM_BEGIN_NAMESPACE

//! Charmm force field
class IMPATOMEXPORT CharmmParameters : public ForceFieldParameters {
public:

  /** construction with Charmm parameters file
      for addition of bonds topology file is enough,
      for the rest both files are needed
   */
  CharmmParameters(const String& topology_file_name,
                   const String& par_file_name = std::string());
  IMP_REF_COUNTED_DESTRUCTOR(CharmmParameters);
private:

  // read non-bonded parameters for VdW computation
  void read_VdW_params(std::ifstream& input_file);
  // read topology file
  void read_topology_file(std::ifstream& input_file);

  ResidueType parse_residue_line(const String& line);
  void parse_atom_line(const String& line, const ResidueType& curr_res_type);
  void parse_bond_line(const String& line, const ResidueType& curr_res_type);

};

IMPATOM_END_NAMESPACE

#endif /* IMPATOM_CHARMM_PARAMETERS_H */
