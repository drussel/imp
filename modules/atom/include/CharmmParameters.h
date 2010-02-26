/**
 * \file atom/CharmmParameters.h \brief access to Charmm force field parameters
 *
 * Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */
#ifndef IMPATOM_CHARMM_PARAMETERS_H
#define IMPATOM_CHARMM_PARAMETERS_H

#include "ForceFieldParameters.h"
#include "charmm_topology.h"
#include "macros.h"

#include <string>
#include <fstream>

// swig is being dumb
IMP_BEGIN_NAMESPACE
class VersionInfo;
IMP_END_NAMESPACE

IMPATOM_BEGIN_NAMESPACE

struct CHARMMBondParameters {
  double force_constant;
  double mean;
};

//! Charmm force field
class IMPATOMEXPORT CharmmParameters : public ForceFieldParameters {
  std::map<std::string, CHARMMIdealResidueTopology> residue_topologies_;
  std::map<std::string, CHARMMPatch> patches_;
  std::map<std::pair<std::string, std::string>,
           CHARMMBondParameters> bond_parameters_;

public:

  /** construction with Charmm parameters file
      for addition of bonds topology file is enough,
      for the rest both files are needed
   */
  CharmmParameters(const String& topology_file_name,
                   const String& par_file_name = std::string());

  void add_patch(CHARMMPatch &patch) {
    patches_.insert(std::make_pair(patch.get_type(), patch));
  }

  void add_residue_topology(CHARMMIdealResidueTopology &res) {
    residue_topologies_.insert(std::make_pair(res.get_type(), res));
  }

  CHARMMPatch &get_patch(std::string name) {
    std::map<std::string, CHARMMPatch>::iterator it = patches_.find(name);
    if (it != patches_.end()) {
      return it->second;
    } else {
      IMP_THROW("Patch " << name << " does not exist", ValueException);
    }
  }

#ifndef SWIG
  const CHARMMPatch &get_patch(std::string name) const {
    std::map<std::string, CHARMMPatch>::const_iterator it = patches_.find(name);
    if (it != patches_.end()) {
      return it->second;
    } else {
      IMP_THROW("Patch " << name << " does not exist", ValueException);
    }
  }
#endif

  CHARMMIdealResidueTopology &get_residue_topology(std::string name) {
    std::map<std::string, CHARMMIdealResidueTopology>::iterator it
              = residue_topologies_.find(name);
    if (it != residue_topologies_.end()) {
      return it->second;
    } else {
      IMP_THROW("Residue " << name << " does not exist", ValueException);
    }
  }

#ifndef SWIG
  const CHARMMIdealResidueTopology &get_residue_topology(std::string name) const
  {
    std::map<std::string, CHARMMIdealResidueTopology>::const_iterator it
              = residue_topologies_.find(name);
    if (it != residue_topologies_.end()) {
      return it->second;
    } else {
      IMP_THROW("Residue " << name << " does not exist", ValueException);
    }
  }
#endif

  CHARMMTopology *make_topology(Hierarchy hierarchy) const;

  const CHARMMBondParameters *get_bond_parameters(std::string type1,
                                                  std::string type2) const {
    std::pair<std::string, std::string> types = std::make_pair(type1, type2);
    std::pair<std::string, std::string> rtypes = std::make_pair(type2, type1);
    if (bond_parameters_.find(types) != bond_parameters_.end()) {
      return &bond_parameters_.find(types)->second;
    } else if (bond_parameters_.find(rtypes) != bond_parameters_.end()) {
      return &bond_parameters_.find(rtypes)->second;
    } else {
      return NULL;
    }
  }

  IMP_FORCE_FIELD_PARAMETERS(CharmmParameters);
private:

  virtual String get_force_field_atom_type(Atom atom) const;

  void read_parameter_file(std::ifstream& input_file);
  // read topology file
  void read_topology_file(std::ifstream& input_file);

  ResidueType parse_residue_line(const String& line);
  void parse_atom_line(const String& line, ResidueType curr_res_type,
                       CHARMMResidueTopologyBase &residue);
  void parse_bond_line(const String& line, ResidueType curr_res_type,
                       CHARMMResidueTopologyBase &residue);

  void parse_nonbonded_parameters_line(String line);
  void parse_bonds_parameters_line(String line);
};

IMPATOM_END_NAMESPACE

#endif /* IMPATOM_CHARMM_PARAMETERS_H */
