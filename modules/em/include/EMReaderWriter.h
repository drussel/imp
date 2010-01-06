/**
 *  \file EMReaderWriter.h
 *  \brief Classes to read or write density files in EM format.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPEM_EM_READER_WRITER_H
#define IMPEM_EM_READER_WRITER_H

#include "config.h"
#include "MapReaderWriter.h"
#include "DensityHeader.h"
#include "def.h"

IMPEM_BEGIN_NAMESPACE

class IMPEMEXPORT EMHeader
{
public:
#ifndef SWIG
  class EMHeaderParse
  {
  public:
    static const int COMMENT_SIZE               = 80;
    static const int VOLTAGE_OFFSET             = 0;
    static const int CS_OFFSET                  = 1;
    static const int APERTURE_OFFSET            = 2;
    static const int MAGNIFICATION_OFFSET       = 3;
    static const int POSTMAGNIFICATION_OFFSET   = 4;
    static const int EXPOSURETIME_OFFSET        = 5;
    static const int OBJECTPIXELSIZE_OFFSET     = 6;
    static const int MICROSCOPE_OFFSET          = 7;
    static const int PIXELSIZE_OFFSET           = 8;
    static const int CCDAREA_OFFSET             = 9;
    static const int DEFOCUS_OFFSET             = 10;
    static const int ASTIGAMATISM_OFFSET        = 11;
    static const int ASTIGAMATISMANGLE_OFFSET   = 12;
    static const int FOCUSINCREMENT_OFFSET      = 13;
    static const int COUNTSPERELECTRON_OFFSET   = 14;
    static const int INTENSITY_OFFSET           = 15;
    static const int ENERGYSLITWIDTH_OFFSET     = 16;
    static const int ENERGYOFFSET_OFFSET        = 17;
    static const int TILTANGLE_OFFSET           = 18;
    static const int TILTAXIS_OFFSET            = 19;
    static const int MARKERX_OFFSET             = 20;
    static const int MARKERY_OFFSET             = 21;
    static const int LSWAP_OFFSET               = 22;

    void InitEMHeader(EMHeader &header) {
      float scale = 1000.0;
      header.magic                  = (int)magic;
      header.type                   = (int)type;
      header.nx                     = nx;
      header.ny                     = ny;
      header.nz                     = nz;
      strncpy(header.comment, comment,COMMENT_SIZE);
      header.comment[COMMENT_SIZE]  = '\0';
      header.voltage                = (float)emdata[VOLTAGE_OFFSET];
      header.Cs                     = (float)emdata[CS_OFFSET]/scale;
      header.Aperture               = (float)emdata[APERTURE_OFFSET];
      header.Magnification          = (float)emdata[MAGNIFICATION_OFFSET];
      header.Postmagnification      = (float)emdata[POSTMAGNIFICATION_OFFSET]
                                      /scale;
      header.Exposuretime           = (float)emdata[EXPOSURETIME_OFFSET];
      header.Objectpixelsize        = (float)emdata[OBJECTPIXELSIZE_OFFSET]
                                      /scale;
      header.Microscope             = (float)emdata[MICROSCOPE_OFFSET];
      header.Pixelsize              = (float)emdata[PIXELSIZE_OFFSET]/scale;
      header.CCDArea                = (float)emdata[CCDAREA_OFFSET]/scale;
      header.Defocus                = (float)emdata[DEFOCUS_OFFSET];
      header.Astigmatism            = (float)emdata[ASTIGAMATISM_OFFSET]/scale;
      header.AstigmatismAngle       = (float)emdata[ASTIGAMATISMANGLE_OFFSET];
      header.FocusIncrement         = (float)emdata[FOCUSINCREMENT_OFFSET]
                                      /scale;
      header.CountsPerElectron      = (float)emdata[COUNTSPERELECTRON_OFFSET]
                                      /scale;
      header.Intensity              = (float)emdata[INTENSITY_OFFSET]/scale;
      header.EnergySlitwidth        = (float)emdata[ENERGYSLITWIDTH_OFFSET];
      header.EnergyOffset           = (float)emdata[ENERGYOFFSET_OFFSET];
      header.Tiltangle              = (float)emdata[TILTANGLE_OFFSET]/scale;
      header.Tiltaxis               = (float)emdata[TILTAXIS_OFFSET]/scale;
      header.MarkerX                = (float)emdata[MARKERX_OFFSET];
      header.MarkerY                = (float)emdata[MARKERY_OFFSET];
      header.lswap                  = emdata[LSWAP_OFFSET];

    }
    void Init(const EMHeader &header) {
      magic    = (unsigned char)header.magic;
      type     = (unsigned char)header.type;
      nx       = header.nx;
      ny       = header.ny;
      nz       = header.nz;
      strncpy(comment,header.comment, COMMENT_SIZE);
      float scale=1000.0;
      emdata[VOLTAGE_OFFSET]              = (int)header.voltage;
      emdata[CS_OFFSET]                   = (int)(header.Cs*scale);
      emdata[APERTURE_OFFSET]             = (int)header.Aperture;
      emdata[MAGNIFICATION_OFFSET]        = (int)header.Magnification;
      emdata[POSTMAGNIFICATION_OFFSET]    = (int)(header.Postmagnification
                                                  * scale);
      emdata[EXPOSURETIME_OFFSET]         = (int)header.Exposuretime ;
      emdata[OBJECTPIXELSIZE_OFFSET]      = (int)(header.Objectpixelsize
                                                  * scale);
      emdata[MICROSCOPE_OFFSET]           = (int)header.Microscope;
      emdata[PIXELSIZE_OFFSET]            = (int)(header.Pixelsize * scale);
      emdata[CCDAREA_OFFSET]              = (int)(header.CCDArea * scale);
      emdata[DEFOCUS_OFFSET]              = (int)header.Defocus;
      emdata[ASTIGAMATISM_OFFSET]         = (int)(header.Astigmatism   * scale);
      emdata[ASTIGAMATISMANGLE_OFFSET]    = (int)header.AstigmatismAngle;
      emdata[FOCUSINCREMENT_OFFSET]       = (int)(header.FocusIncrement
                                                  * scale);
      emdata[COUNTSPERELECTRON_OFFSET]    = (int)(header.FocusIncrement
                                                  * scale);
      emdata[INTENSITY_OFFSET]            = (int)(header.Intensity * scale);
      emdata[ENERGYSLITWIDTH_OFFSET]      = (int)header.EnergySlitwidth;
      emdata[ENERGYOFFSET_OFFSET]         = (int)header.EnergyOffset ;
      emdata[TILTANGLE_OFFSET]            = (int)(header.Tiltangle *scale);
      emdata[TILTAXIS_OFFSET]             = (int)(header.Tiltaxis * scale);
      emdata[MARKERX_OFFSET]              = (int)header.MarkerX;
      emdata[MARKERY_OFFSET]              = (int)header.MarkerY;
      emdata[LSWAP_OFFSET]                = header.lswap;

    }
    //actual information in the EM header
    unsigned char magic;
    unsigned char dummy[2];
    unsigned char type;
    int nx,ny,nz;
    char comment[80];
    int emdata[40];
    char dummy_long[256];

  };
#endif // SWIG



  EMHeader(){}
  EMHeader(const DensityHeader &header);
  void GenerateCommonHeader(DensityHeader &header);


  //// Outputs coordinates delimited by single space.
  friend std::ostream& operator<<(std::ostream& s, const EMHeader &h) {
    return s <<"size : " << h.nx << "  " << h.ny << "  " << h.nz
      << std::endl << "magic: " << h.magic << std::endl <<
      "type:  " << h.type <<  std::endl <<
      " comment : " << h.comment <<  std::endl <<
      " voltage : " << h.voltage <<  std::endl <<
      "cs: " << h.Cs << std::endl <<
      "aperture: " << h.Aperture<< std::endl <<
      "Magnification " << h.Magnification<< std::endl <<
      " Postmagnification "<< h.Postmagnification << std::endl <<
      " Exposuretime  " << h.Exposuretime << std::endl <<
      "Objectpixelsize  " << h.Objectpixelsize << std::endl <<
      "Microscope  " << h.Microscope << std::endl <<
      "pixel-size:  " << h.Pixelsize << std::endl;
  }


  //attributes
  int magic;
  int type;
  int nx,ny,nz;
  char comment[81];
  float voltage;
  float Cs;
  float Aperture;
  float Magnification;
  float Postmagnification;
  float Exposuretime;
  float Objectpixelsize;
  float Microscope;
  float Pixelsize;
  float CCDArea;
  float Defocus;
  float Astigmatism;
  float AstigmatismAngle;
  float FocusIncrement;
  float CountsPerElectron;
  float Intensity;
  float EnergySlitwidth;
  float EnergyOffset;
  float Tiltangle;
  float Tiltaxis;
  float MarkerX;
  float MarkerY;
  int lswap;
};


class IMPEMEXPORT EMReaderWriter : public MapReaderWriter
{
public:
 //! Reads a density file in EM format and stores the information
 /**
  \param[in] filename name of the file to read
  \param[in] data pointer to the data
  \param[in] header DensityHeader() to store the header information
  \exception IOException in case that the filename was not found
  */
  void Read(const char *filename, float **data, DensityHeader &header);
  //! Writes a density file in EM format with the header information
  /**
     \param[in] filename name of the file to read
     \param[in] data pointer to the data
     \param[in] header DensityHeader() with the header information
  */
  void Write(const char* filename, const float *data,
             const DensityHeader &header);
protected:
  //! Reads the header
  //! Reads the data
  /**
  \param file ifstream of the file to read
  \param header EMHeader to store the header information
  */
  void ReadHeader(std::ifstream &file, EMHeader &header);
  //! Reads the data
  /**
  \param file ifstream of the file to read
  \param data pointer to store the data
  \param header EMHeader() to store the header information
  \exception IOException if the data allocation had failed
  \exception  IOException is the requested data type is not implemented
  */
  void ReadData(std::ifstream &file, float **data, const EMHeader &header);
  //! Writes the header
  /**
  \param[in] file ofstream of the file to write
  \param[in] header EMHeader() with the header information
  \exception IOException if the writing operation had failed.
  */
  void WriteHeader(std::ostream& s, const EMHeader &header);
};

IMPEM_END_NAMESPACE

#endif  /* IMPEM_EM_READER_WRITER_H */
