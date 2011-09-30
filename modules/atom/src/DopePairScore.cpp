/**
 *  \file DopePairScore.cpp
 *  \brief Dope score
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/atom/DopePairScore.h>
#include <IMP/atom/Atom.h>
#include <IMP/atom/Residue.h>

IMPATOM_BEGIN_NAMESPACE

namespace {

  namespace data {

#define DOPE_TYPE(NameKey) const DopeType \
    Dope##NameKey(DopeType::add_key(#NameKey));
#define DOPE_TYPE_ALIAS(Name, Key) const DopeType Dope##Name\
  (DopeType::add_alias(DopeType(#Key), #Name));

    DOPE_TYPE(ALA_N)
    DOPE_TYPE(ALA_CA)
    DOPE_TYPE(ALA_C)
    DOPE_TYPE(ALA_O)
    DOPE_TYPE(ALA_CB)
    DOPE_TYPE(ARG_N)
    DOPE_TYPE(ARG_CA)
    DOPE_TYPE(ARG_C)
    DOPE_TYPE(ARG_O)
    DOPE_TYPE(ARG_CB)
    DOPE_TYPE(ARG_CG)
    DOPE_TYPE(ARG_CD)
    DOPE_TYPE(ARG_NE)
    DOPE_TYPE(ARG_CZ)
    DOPE_TYPE(ARG_NH)
    DOPE_TYPE_ALIAS(ARG_NH1, ARG_NH)
    DOPE_TYPE_ALIAS(ARG_NH2, ARG_NH)
    DOPE_TYPE(ASN_N)
    DOPE_TYPE(ASN_CA)
    DOPE_TYPE(ASN_C)
    DOPE_TYPE(ASN_O)
    DOPE_TYPE(ASN_CB)
    DOPE_TYPE(ASN_CG)
    DOPE_TYPE(ASN_OD1)
    DOPE_TYPE(ASN_ND2)
    DOPE_TYPE(ASP_N)
    DOPE_TYPE(ASP_CA)
    DOPE_TYPE(ASP_C)
    DOPE_TYPE(ASP_O)
    DOPE_TYPE(ASP_CB)
    DOPE_TYPE(ASP_CG)
    DOPE_TYPE(ASP_OD)
    DOPE_TYPE_ALIAS(ASP_OD1, ASP_OD)
    DOPE_TYPE_ALIAS(ASP_OD2, ASP_OD)
    DOPE_TYPE(CYS_N)
    DOPE_TYPE_ALIAS(CSS_N,CYS_N)
    DOPE_TYPE(CYS_CA)
    DOPE_TYPE_ALIAS(CSS_CA,CYS_CA)
    DOPE_TYPE(CYS_C)
    DOPE_TYPE_ALIAS(CSS_C,CYS_C)
    DOPE_TYPE(CYS_O)
    DOPE_TYPE_ALIAS(CSS_O,CYS_O)
    DOPE_TYPE(CYS_CB)
    DOPE_TYPE_ALIAS(CSS_CB,CYS_CB)
    DOPE_TYPE(CYS_SG)
    DOPE_TYPE_ALIAS(CSS_SG,CYS_SG)
    DOPE_TYPE(GLN_N)
    DOPE_TYPE(GLN_CA)
    DOPE_TYPE(GLN_C)
    DOPE_TYPE(GLN_O)
    DOPE_TYPE(GLN_CB)
    DOPE_TYPE(GLN_CG)
    DOPE_TYPE(GLN_CD)
    DOPE_TYPE(GLN_OE1)
    DOPE_TYPE(GLN_NE2)
    DOPE_TYPE(GLU_N)
    DOPE_TYPE(GLU_CA)
    DOPE_TYPE(GLU_C)
    DOPE_TYPE(GLU_O)
    DOPE_TYPE(GLU_CB)
    DOPE_TYPE(GLU_CG)
    DOPE_TYPE(GLU_CD)
    DOPE_TYPE(GLU_OE)
    DOPE_TYPE_ALIAS(GLU_OE1, GLU_OE)
    DOPE_TYPE_ALIAS(GLU_OE2, GLU_OE)
    DOPE_TYPE(GLY_N)
    DOPE_TYPE(GLY_CA)
    DOPE_TYPE(GLY_C)
    DOPE_TYPE(GLY_O)
    DOPE_TYPE(HIS_N)
    DOPE_TYPE_ALIAS(HSD_N, HIS_N)
    DOPE_TYPE(HIS_CA)
    DOPE_TYPE_ALIAS(HSD_CA, HIS_CA)
    DOPE_TYPE(HIS_C)
    DOPE_TYPE_ALIAS(HSD_C, HIS_C)
    DOPE_TYPE(HIS_O)
    DOPE_TYPE_ALIAS(HSD_O, HIS_O)
    DOPE_TYPE(HIS_CB)
    DOPE_TYPE_ALIAS(HSD_CB, HIS_CB)
    DOPE_TYPE(HIS_CG)
    DOPE_TYPE_ALIAS(HSD_CG, HIS_CG)
    DOPE_TYPE(HIS_ND1)
    DOPE_TYPE_ALIAS(HSD_ND1, HIS_ND1)
    DOPE_TYPE(HIS_CD2)
    DOPE_TYPE_ALIAS(HSD_CD2, HIS_CD2)
    DOPE_TYPE(HIS_CE1)
    DOPE_TYPE_ALIAS(HSD_CE1, HIS_CE1)
    DOPE_TYPE(HIS_NE2)
    DOPE_TYPE_ALIAS(HSD_NE2, HIS_NE2)
    DOPE_TYPE(ILE_N)
    DOPE_TYPE(ILE_CA)
    DOPE_TYPE(ILE_C)
    DOPE_TYPE(ILE_O)
    DOPE_TYPE(ILE_CB)
    DOPE_TYPE(ILE_CG1)
    DOPE_TYPE(ILE_CG2)
    DOPE_TYPE(ILE_CD1)
    DOPE_TYPE(LEU_N)
    DOPE_TYPE(LEU_CA)
    DOPE_TYPE(LEU_C)
    DOPE_TYPE(LEU_O)
    DOPE_TYPE(LEU_CB)
    DOPE_TYPE(LEU_CG)
    DOPE_TYPE(LEU_CD)
    DOPE_TYPE_ALIAS(LEU_CD1, LEU_CD)
    DOPE_TYPE_ALIAS(LEU_CD2, LEU_CD)
    DOPE_TYPE(LYS_N)
    DOPE_TYPE(LYS_CA)
    DOPE_TYPE(LYS_C)
    DOPE_TYPE(LYS_O)
    DOPE_TYPE(LYS_CB)
    DOPE_TYPE(LYS_CG)
    DOPE_TYPE(LYS_CD)
    DOPE_TYPE(LYS_CE)
    DOPE_TYPE(LYS_NZ)
    DOPE_TYPE(MET_N)
    DOPE_TYPE(MET_CA)
    DOPE_TYPE(MET_C)
    DOPE_TYPE(MET_O)
    DOPE_TYPE(MET_CB)
    DOPE_TYPE(MET_CG)
    DOPE_TYPE(MET_SD)
    DOPE_TYPE(MET_CE)
    DOPE_TYPE(PHE_N)
    DOPE_TYPE(PHE_CA)
    DOPE_TYPE(PHE_C)
    DOPE_TYPE(PHE_O)
    DOPE_TYPE(PHE_CB)
    DOPE_TYPE(PHE_CG)
    DOPE_TYPE(PHE_CD)
    DOPE_TYPE_ALIAS(PHE_CD1, PHE_CD)
    DOPE_TYPE_ALIAS(PHE_CD2, PHE_CD)
    DOPE_TYPE(PHE_CE)
    DOPE_TYPE_ALIAS(PHE_CE1, PHE_CE)
    DOPE_TYPE_ALIAS(PHE_CE2, PHE_CE)
    DOPE_TYPE(PHE_CZ)
    DOPE_TYPE(PRO_N)
    DOPE_TYPE(PRO_CA)
    DOPE_TYPE(PRO_C)
    DOPE_TYPE(PRO_O)
    DOPE_TYPE(PRO_CB)
    DOPE_TYPE(PRO_CG)
    DOPE_TYPE(PRO_CD)
    DOPE_TYPE(SER_N)
    DOPE_TYPE(SER_CA)
    DOPE_TYPE(SER_C)
    DOPE_TYPE(SER_O)
    DOPE_TYPE(SER_CB)
    DOPE_TYPE(SER_OG)
    DOPE_TYPE(THR_N)
    DOPE_TYPE(THR_CA)
    DOPE_TYPE(THR_C)
    DOPE_TYPE(THR_O)
    DOPE_TYPE(THR_CB)
    DOPE_TYPE(THR_OG1)
    DOPE_TYPE(THR_CG2)
    DOPE_TYPE(TRP_N)
    DOPE_TYPE(TRP_CA)
    DOPE_TYPE(TRP_C)
    DOPE_TYPE(TRP_O)
    DOPE_TYPE(TRP_CB)
    DOPE_TYPE(TRP_CG)
    DOPE_TYPE(TRP_CD1)
    DOPE_TYPE(TRP_CD2)
    DOPE_TYPE(TRP_NE1)
    DOPE_TYPE(TRP_CE2)
    DOPE_TYPE(TRP_CE3)
    DOPE_TYPE(TRP_CZ2)
    DOPE_TYPE(TRP_CZ3)
    DOPE_TYPE(TRP_CH2)
    DOPE_TYPE(TYR_N)
    DOPE_TYPE(TYR_CA)
    DOPE_TYPE(TYR_C)
    DOPE_TYPE(TYR_O)
    DOPE_TYPE(TYR_CB)
    DOPE_TYPE(TYR_CG)
    DOPE_TYPE(TYR_CD)
    DOPE_TYPE_ALIAS(TYR_CD1, TYR_CD)
    DOPE_TYPE_ALIAS(TYR_CD2, TYR_CD)
    DOPE_TYPE(TYR_CE)
    DOPE_TYPE_ALIAS(TYR_CE1, TYR_CE)
    DOPE_TYPE_ALIAS(TYR_CE2, TYR_CE)
    DOPE_TYPE(TYR_CZ)
    DOPE_TYPE(TYR_OH)
    DOPE_TYPE(VAL_N)
    DOPE_TYPE(VAL_CA)
    DOPE_TYPE(VAL_C)
    DOPE_TYPE(VAL_O)
    DOPE_TYPE(VAL_CB)
    DOPE_TYPE(VAL_CG)
    DOPE_TYPE_ALIAS(VAL_CG1, VAL_CG)
    DOPE_TYPE_ALIAS(VAL_CG2, VAL_CG)
}

  IntKey get_dope_type_key() {
    static const IntKey ik("dope atom type");
    return ik;
  }
}


DopePairScore::DopePairScore(double threshold):
  P(get_dope_type_key(), threshold, get_data_path("dope_score.lib")){
  }

DopePairScore::DopePairScore(double threshold, base::TextInput file):
  P(get_dope_type_key(), threshold, file){
  }

namespace {
void add_dope_score_data(Atom atom) {
  int type;
  Residue rd= get_residue(atom);
  std::string atom_string=atom.get_atom_type().get_string();
  std::string residue_string= rd.get_residue_type().get_string();
  std::string score_type = residue_string + '_' + atom_string;
  if (!DopeType::get_key_exists(score_type)) {
    type=-1;
  } else {
    //std::cout << "Type for " << atom << " is "
    //  << DopeType(score_type) << std::endl;
    type= DopeType(score_type).get_index();
  }
  if (type==-1 && atom.get_element() != H) {
    IMP_LOG(TERSE, "Failed to find type for "
            << atom << " " << rd << std::endl);
  }
  if (atom->has_attribute(get_dope_type_key())) {
    IMP_USAGE_CHECK(atom->get_value(get_dope_type_key()) == type,
                    "Atom " << atom << " already has dope score type "
                    << "but it is not correct. Got "
                    << atom->get_value(get_dope_type_key())
                    << " expected " << type);
  } else {
    atom->add_attribute(get_dope_type_key(), type);
  }
}
}

void add_dope_score_data(Hierarchy h) {
  Hierarchies atoms= get_by_type(h, ATOM_TYPE);
  for (unsigned int i= 0; i< atoms.size(); ++i) {
    add_dope_score_data(Atom(atoms[i]));
  }
}


IMPATOM_END_NAMESPACE
