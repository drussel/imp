/**
 *  \file geometry.h
 *  \brief XXXXXXXXXXXXXX
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPDISPLAY_GEOMETRY_H
#define IMPDISPLAY_GEOMETRY_H

#include "config.h"
#include "Color.h"
#include "macros.h"
#include "internal/version_info.h"
#include <IMP/PairContainer.h>
#include <IMP/SingletonContainer.h>
#include <IMP/RefCountedObject.h>
#include <IMP/algebra/Vector3D.h>

IMPDISPLAY_BEGIN_NAMESPACE

//! Compute a geometric description from a particle
/** Extract geometry from a particle. This is an abstract
    base class.

    A given particle is turned into a geometric object with a
    given dimension and size. An object of dimension 0 is a sphere
    (or a point if the radius is 0), and object of dimension 1
    is a cylinder or segment, and an object with higher dimension
    is a polygon (and must have thickness 0). A dimension of -1
    means do nothing.
 */
class IMPDISPLAYEXPORT Geometry: public RefCountedObject
{
  Color default_color_;
  std::string name_;
public:
  Geometry();

  virtual ~Geometry();

  //! Return the dimension of the object
  virtual unsigned int get_dimension() const=0;

  //! Return the ith vertex of the object
  virtual algebra::Vector3D get_vertex(unsigned int i) const=0;

  //! Return the number of vertices of the object
  virtual unsigned int get_number_of_vertices() const=0;

  //! Return the thickness of the object
  virtual Float get_size() const {
    return 0;
  }

  //! Return a name for the object (or the empty string)
  std::string get_name() const {
    return name_;
  }

  //! Return the color of the object
  virtual Color get_color() const {
    return default_color_;
  }

  //! Set the default color
  void set_color(Color c) {
    default_color_= c;
  }

  //! Set the name
  void set_name(std::string c) {
    name_= c;
  }

  //! Write information about the object
  virtual void show(std::ostream &out= std::cout) const=0;

  //! get the version info
  virtual VersionInfo get_version_info() const =0;
};

IMP_OUTPUT_OPERATOR(Geometry);

//! Should be ref counted but swig objects
typedef std::vector<Geometry* > Geometries;

//! Produce some geometry from a particle
class IMPDISPLAYEXPORT CompoundGeometry: public RefCountedObject {
 public:
  CompoundGeometry();
  virtual ~CompoundGeometry();
  //! Write information about the object
  virtual void show(std::ostream &out= std::cout) const{};

  //! get the version info
  virtual VersionInfo get_version_info() const {return internal::version_info;}

  //! Return a list of geometry objects
  virtual Geometries get_geometry() const =0;
};
IMP_OUTPUT_OPERATOR(CompoundGeometry);


//! Should be ref counted but swig objects
typedef std::vector<CompoundGeometry* > CompoundGeometries;


IMPDISPLAY_END_NAMESPACE

#endif  /* IMPDISPLAY_GEOMETRY_H */
