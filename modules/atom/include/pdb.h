/**
 *  \file atom/pdb.h
 *  \brief Functions to read pdbs
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */
#ifndef IMPATOM_PDB_H
#define IMPATOM_PDB_H

#include "atom_config.h"
#include "Hierarchy.h"
#include "Atom.h"
#include "element.h"
#include "internal/pdb.h"
#include <IMP/file.h>
#include <IMP/Model.h>
#include <IMP/Particle.h>

IMPATOM_BEGIN_NAMESPACE


//! Select which atoms to read from a PDB file
/** Selector is a general purpose class used to select records from a PDB
    file. Using descendants of this class one may implement arbitrary
    selection functions with operator() and pass them to PDB reading functions
    for object selection. Simple selectors can be used to build more complicated
    ones. Inheritence means "AND" unless otherwise noted (that is, the
    CAlphaPDBSelector takes all non-alternate C-alphas since it inherits from
    NonAlternativePDBSelector).

    PDBSelectors are designed to be temporary objects and should never be
    stored.
    \see read_pdb
*/
class IMPATOMEXPORT PDBSelector {
 public:
  //! Return true if the line should be processed
  virtual bool operator()(const std::string& pdb_line) const=0;
  virtual ~PDBSelector();
};

//! Select all ATOM and HETATM records which are not alternatives
class NonAlternativePDBSelector : public PDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    return ((internal::atom_alt_loc_indicator(pdb_line) == ' ') ||
            (internal::atom_alt_loc_indicator(pdb_line) == 'A'));
  }
};

//! Select all CA ATOM records
class CAlphaPDBSelector : public NonAlternativePDBSelector {
 public:
  bool operator() (const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    const std::string type = internal::atom_type(pdb_line);
    return (type[1] == 'C' && type[2] == 'A' && type[3] == ' ');
  }
};

//! Select all CB ATOM records
class CBetaPDBSelector: public NonAlternativePDBSelector {
 public:
  bool operator() (const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    const std::string type = internal::atom_type(pdb_line);
    return (type[1] == 'C' && type[2] == 'B' && type[3] == ' ');
  }
};

//! Select all C (not CA or CB) ATOM records
class CPDBSelector: public NonAlternativePDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    const std::string type = internal::atom_type(pdb_line);
    return (type[1] == 'C' && type[2] == ' ' && type[3] == ' ');
  }
};

//! Select all N ATOM records
class NPDBSelector: public NonAlternativePDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    const std::string type = internal::atom_type(pdb_line);
    return (type[1] == 'N' && type[2] == ' ' && type[3] == ' ');
  }
};

//! Defines a selector that will pick every ATOM and HETATM record
class AllPDBSelector : public PDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const { return true; }
};

//! Select all ATOM and HETATMrecords with the given chain ids
class ChainPDBSelector : public NonAlternativePDBSelector {
 public:
  //! The chain id can be any character in chains
  ChainPDBSelector(const std::string &chains): chains_(chains) {}
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    for(int i=0; i < (int)chains_.length(); i++) {
      if(internal::atom_chain_id(pdb_line) == chains_[i])
        return true;
    }
    return false;
  }
 private:
  std::string chains_;
};

//! Select all non-water ATOM and HETATMrecords
class WaterPDBSelector : public NonAlternativePDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    const std::string res_name = internal::atom_residue_name(pdb_line);
    return ((res_name[0]=='H' && res_name[1] =='O' && res_name[2]=='H') ||
            (res_name[0]=='D' && res_name[1] =='O' && res_name[2]=='D'));
  }
};

//! Select all hydrogen ATOM and HETATM records
class HydrogenPDBSelector : public NonAlternativePDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    return ((pdb_line[internal::atom_element_field_]=='H'
             && pdb_line[internal::atom_element_field_+1]==' ')
            || (pdb_line[internal::atom_element_field_]==' '
             && pdb_line[internal::atom_element_field_+1]=='H')
            || (pdb_line[internal::atom_element_field_]==' '
                && pdb_line[internal::atom_element_field_+1]==' '
                && pdb_line[0]== 'A'
                && (pdb_line[internal::atom_type_field_] =='H'
                    || pdb_line[internal::atom_type_field_+1] == 'H')));
  }
};

//! Select non water and non hydrogen atoms
class NonWaterNonHydrogenPDBSelector : public NonAlternativePDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    WaterPDBSelector w;
    HydrogenPDBSelector h;
    return (! w(pdb_line) && ! h(pdb_line));
  }
};

//! Select all non-water non-alternative ATOM and HETATM records
class NonWaterPDBSelector : public NonAlternativePDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    WaterPDBSelector w;
    return( ! w(pdb_line));
  }
};

//! Select all P ATOM records
class PPDBSelector : public NonAlternativePDBSelector {
 public:
  bool operator()(const std::string& pdb_line) const {
    if (!NonAlternativePDBSelector::operator()(pdb_line)) return false;
    const std::string type = internal::atom_type(pdb_line);
    return (type[1] == 'P' && type[2] == ' ');
  }
};

#if 0

//! Only select lines liked by both selectors
/** The two passed selectors are not deleted by this selector. */
class AndPDBSelector: public NonAlternativePDBSelector {
  const PDBSelector &a, &b;
public:
  AndPDBSelector(const PDBSelector &a,
                 const PDBSelector &b): a_(a), b_(b) {
  }
  bool operator()(const std::string& pdb_line) const {
    return a_(pdb_line)&& b_(pdb_line);
  }
};
#endif

/** @name PDB Reading

   The read PDB methods produce a hierarchy that looks as follows:
    - One Atom per ATOM or HETATM record in the PDB.
    - All Atom particles have a parent which is a Residue.
    - All Residue particles have a parent which is a Chain.

    Waters are currently dropped if they are ATOM records. This can be fixed.

    The read_pdb() functions should successfully parse all valid pdb files. It
    can produce warnings on files which are not valid. It will attempt to read
    such files, but all bets are off.

    When reading PDBs, PDBSelector objects can be used to choose to only process
    certain record types. See the class documentation for more information.
    When no PDB selector is supplied for reading, the
    NonWaterPDBSelector is used.

    Set the IMP::LogLevel to IMP::VERBOSE to see details of parse errors.
*/
//!@{

/** Read a all the molecules in the first model of the
    pdb file.

    \relatesalso Hierarchy
 */
IMPATOMEXPORT Hierarchy read_pdb(TextInput in,
                                 Model* model);

/** \relatesalso Hierarchy
 */
IMPATOMEXPORT Hierarchy
read_pdb(TextInput in,
         Model* model,
         const PDBSelector& selector,
         bool select_first_model = true);



/** \relatesalso Hierarchy
 */
IMPATOMEXPORT Hierarchies read_multimodel_pdb(TextInput in,
                                              Model *model,
                                              const PDBSelector& selector);
/** @} */

/** @name PDB Writing

    The methods to write a PDBs expects a Hierarchy that looks as follows:
    - all leaves are Atom particles
    - all Atom particles have Residue particles as parents

    All Residue particles that have a Chain particle as an ancestor
    are considered part of a protein, DNA or RNA, ones without are
    considered heterogens.

    The functions produce files that are not valid PDB files,
    eg only ATOM/HETATM lines are printed for all Atom particles
    in the hierarchy. Complain if your favorite program can't read them and
    we might fix it.
*/
//!@{

/** \relatesalso Hierarchy
*/
IMPATOMEXPORT void write_pdb(Hierarchy mhd,
                             TextOutput out);
/** \relatesalso Hierarchy
*/
IMPATOMEXPORT void write_pdb(const Hierarchies &mhd,
                             TextOutput out);

/** \relatesalso Hierarchy
*/
IMPATOMEXPORT void write_multimodel_pdb(
                        const Hierarchies& mhd, TextOutput out);

/** @} */


#ifndef IMP_DOXYGEN

/**
   This function returns a string in PDB ATOM format
*/
IMPATOMEXPORT std::string pdb_string(const algebra::VectorD<3>& v,
                                     int index = -1,
                                     AtomType at = AT_C,
                                     ResidueType rt = atom::ALA,
                                     char chain = ' ',
                                     int res_index = 1,
                                     char res_icode = ' ',
                                     Element e = C);

/**
   This function returns a connectivity string in PDB format
  /note The CONECT records specify connectivity between atoms for which
      coordinates are supplied. The connectivity is described using
      the atom serial number as found in the entry.
  /note http://www.bmsc.washington.edu/CrystaLinks/man/pdb/guide2.2_frame.html
*/
IMPATOMEXPORT std::string conect_record_string(int,int);
#endif





IMPATOM_END_NAMESPACE

#endif /* IMPATOM_PDB_H */
