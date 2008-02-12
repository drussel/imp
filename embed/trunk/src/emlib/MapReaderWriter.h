#ifndef _MAPREADERWRITER_H
#define _MAPREADERWRITER_H

/*
  CLASS
  MapReader

  KEYWORDS

  AUTHORS
  Keren Lasker (mailto: kerenl@salilab.org)

  OVERVIEW TEXT
  an abstract class for reading a map


  CHANGES LOG
  Sep 27 2007, Keren: change the class into a stateless one.


*/



#include "EM_config.h"
#include "DensityHeader.h"
#include <iostream>
#include <fstream>
using namespace std;
class EMDLLEXPORT MapReaderWriter {
public:
//   friend  ostream& operator<<(ostream& s, const MapReaderWriter &m) { // write the map
//     m.Write(s);
//   }
//   friend istream& operator>>(istream& s, MapReaderWriter &m) { //read map
//     m.Read(s);
//   }
 
  virtual int Read(const char *filename, float **data, DensityHeader &header){return 0;}
  virtual void Write(const char *filename, const float *data, const DensityHeader &header){}
  virtual ~MapReaderWriter() {}

};


#endif //_MAPREADERWRITER_H
