/**
 *  \file IMP/display/python_only.h   \brief Build dependency graphs on models.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPDISPLAY_PYTHON_ONLY_H
#define IMPDISPLAY_PYTHON_ONLY_H

#include "display_config.h"

IMP_BEGIN_NAMESPACE

#ifdef IMP_DOXYGEN
/** This writer displays things to a
    \external{http://pivy.coin3d.org/,Pivy} window.

    At the moment it support SphereGeometry and CylinderGeometry.

    This class is under development.
*/
class PivyWriter:public Writer {
 public:
  PivyWriter();
  /** Show the window and start the interactive event loop.
      Normal python execution will resume when the window
      is closed.
  */
  void show();
};
#endif

IMP_END_NAMESPACE

#endif  /* IMPDISPLAY_PYTHON_ONLY_H */