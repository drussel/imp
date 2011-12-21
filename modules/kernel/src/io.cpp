/**
 *  \file file.cpp
 *  \brief Get directories used by IMP.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/io.h>
#include <IMP/compatibility/map.h>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>

#define IMP_CHECK_MODEL_PARTICLES(m)                            \
  for (Model::ParticleIterator pit= m->particles_begin();       \
       pit != m->particles_end(); ++pit) {                      \
    IMP::check_particle(*pit);                                  \
  }                                                             \

IMP_BEGIN_NAMESPACE

// not yet exposed
void check_particle(Particle*p);

namespace {
  void write_particles_to_buffer(const ParticlesTemp &particles,
                          const FloatKeys &keys,
                          char *buf, unsigned int size) {
    IMP_USAGE_CHECK(size>= particles.size()*keys.size()*sizeof(double),
                    "Not enough space: " << size << " vs "
                    << particles.size()*keys.size()*sizeof(double));
    boost::iostreams::stream<boost::iostreams::array_sink>  in(buf, size);
    for (unsigned int i=0; i< particles.size(); ++i) {
      for (unsigned int j=0; j< keys.size(); ++j) {
        double value=0;
        if (particles[i]->has_attribute(keys[j])) {
          value=particles[i]->get_value(keys[j]);
        }
        in.write(reinterpret_cast<char*>(&value), sizeof(double));
        if (!in) {
          IMP_THROW("Error reading writing to buffer", IOException);
        }
      }
    }
  }
  void read_particles_from_buffer( const char *buffer, unsigned int size,
                          const ParticlesTemp &particles,
                          const FloatKeys &keys) {
    IMP_USAGE_CHECK(size== particles.size()*keys.size()*sizeof(double),
                    "Not enough data to read: " << size
                    << " vs " << particles.size()*keys.size()*sizeof(double));
    boost::iostreams::stream<boost::iostreams::array_source>  in(buffer, size);
    for (unsigned int i=0; i< particles.size(); ++i) {
      for (unsigned int j=0; j< keys.size(); ++j) {
        double value;
        in.read(reinterpret_cast<char*>(&value), sizeof(double));
        if (!in) {
          IMP_THROW("Error reading from buffer", IOException);
        }
        if (particles[i]->has_attribute(keys[j])) {
          particles[i]->set_value(keys[j], value);
        }
      }
    }
  }
}


IMPEXPORT vector<char>
write_particles_to_buffer(const ParticlesTemp &particles,
                          const FloatKeys &keys) {
  if (particles.empty() || keys.empty()) {
    return vector<char>();
  }
  unsigned int size= particles.size()*keys.size()*sizeof(double);
  vector<char> ret(size);
  write_particles_to_buffer(particles, keys, &ret.front(), size);
  return ret;
}
IMPEXPORT void read_particles_from_buffer( const vector<char> &buffer,
                                  const ParticlesTemp &particles,
                                  const FloatKeys &keys) {
  if (particles.empty() || keys.empty()) {
    return;
  }
  read_particles_from_buffer(&buffer.front(),
                             buffer.size()*sizeof(double), particles, keys);
  IMP_CHECK_MODEL_PARTICLES(particles[0]->get_model());
}

IMP_END_NAMESPACE
