/**
 *  \file file.cpp
 *  \brief Get directories used by IMP.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/io.h>
#include <IMP/internal/particle_save.h>
#include <IMP/compatibility/map.h>
#ifdef IMP_KERNEL_USE_NETCDFCPP
#include <netcdfcpp.h>
#endif
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

#ifdef IMP_USE_DEPRECATED


void write_particles(const ParticlesTemp &particles,
                     TextOutput out) {
  compatibility::map<Particle*, unsigned int> to;
  for (unsigned int i=0; i< particles.size(); ++i) {
    to[particles[i]]=i;
  }
  compatibility::map<unsigned int, internal::ParticleData> data;
  for (compatibility::map<Particle*, unsigned int>::const_iterator
         it= to.begin();
       it != to.end(); ++it) {
    data[it->second]= internal::ParticleData(it->first);
  }
  for (compatibility::map<unsigned int,
         internal::ParticleData>::const_iterator it= data.begin();
       it != data.end(); ++it) {
    out.get_stream() << "particle: " << it->first << std::endl;
    it->second.write_yaml(out, to);
  }
}

void write_particles(const ParticlesTemp &particles,
                     const FloatKeys &keys,
                     TextOutput out) {
  compatibility::map<Particle*, unsigned int> to;
  for (unsigned int i=0; i< particles.size(); ++i) {
    to[particles[i]]=i;
  }
  compatibility::map<unsigned int, internal::ParticleData> data;
  for (compatibility::map<Particle*, unsigned int>::const_iterator
         it= to.begin();
       it != to.end(); ++it) {
    data[it->second]= internal::ParticleData(it->first, keys);
  }
  for (compatibility::map<unsigned int,
         internal::ParticleData>::const_iterator it= data.begin();
       it != data.end(); ++it) {
    out.get_stream() << "particle: " << it->first << std::endl;
    it->second.write_yaml(out, to);
  }
}

void read_particles(TextInput in,
                    const ParticlesTemp &particles,
                    Model *) {
  internal::LineStream ls(in);
  compatibility::map<unsigned int, Particle *> from;
  for (unsigned int i=0; i< particles.size(); ++i) {
    from[i]= particles[i];
  }
  compatibility::map<Particle*, internal::ParticleData> data;
  do {
    internal::LineStream::LinePair lp= ls.get_line();
    if (lp.first.empty()) break;
    if (lp.first== "particle") {
      std::istringstream iss(lp.second);
      int i=-1;
      iss >> i;
      if (i<0) {
        IMP_THROW("Error reading from line "
                  << lp.first << ": " << lp.second,
                  IOException);
      }
      if (from.find(i) == from.end()) {
        IMP_THROW("Can't find particle " << i << " in map",
                  IOException);
      }
      data[from.find(i)->second]=internal::ParticleData();
      ls.push_indent();
      data[from.find(i)->second].read_yaml(ls, from);
      ls.pop_indent();
    } else {
      IMP_LOG(TERSE,
              "Found " << lp.first << " when looking for particle"
              << std::endl);
    }
  } while (true);
  IMP_LOG(TERSE, "Read " << data.size() << " particles"
          << std::endl);
  if (data.size() != from.size()) {
    IMP_THROW("Did not read all particles. Expected "
              << from.size() << " got " << data.size(),
              IOException);
  }
  for (compatibility::map<Particle*, internal::ParticleData>::const_iterator
         it= data.begin(); it != data.end(); ++it) {
    it->second.apply(it->first);
  }
  IMP_CHECK_MODEL_PARTICLES(particles[0]->get_model());
}


void read_particles(TextInput in,
                    const ParticlesTemp &particles,
                    const FloatKeys &keys) {
  internal::LineStream ls(in);
  compatibility::map<unsigned int, Particle *> from;
  for (unsigned int i=0; i< particles.size(); ++i) {
    from[i]= particles[i];
  }
  compatibility::map<Particle*, internal::ParticleData> data;
  do {
    internal::LineStream::LinePair lp= ls.get_line();
    if (lp.first.empty()) break;
    if (lp.first== "particle") {
      std::istringstream iss(lp.second);
      int i=-1;
      iss >> i;
      if (i<0) {
        IMP_THROW("Error reading from line "
                  << lp.first << ": " << lp.second,
                  IOException);
      }
      if (from.find(i) == from.end()) {
        IMP_THROW("Can't find particle " << i << " in map",
                  IOException);
      }
      data[from.find(i)->second]=internal::ParticleData();
      ls.push_indent();
      data[from.find(i)->second].read_yaml(ls, from);
      ls.pop_indent();
    } else {
      IMP_LOG(TERSE,
              "Found " << lp.first << " when looking for particle"
              << std::endl);
    }
  } while (true);
  IMP_LOG(TERSE, "Read " << data.size() << " particles"
          << std::endl);
  if (data.size() != from.size()) {
    IMP_THROW("Did not read all particles. Expected "
              << from.size() << " got " << data.size(),
              IOException);
  }
  for (compatibility::map<Particle*, internal::ParticleData>::const_iterator
         it= data.begin(); it != data.end(); ++it) {
    it->second.apply(it->first, keys);
  }
  IMP_CHECK_MODEL_PARTICLES(particles[0]->get_model());
}




#ifdef IMP_KERNEL_USE_NETCDFCPP

void write_particles_binary(const ParticlesTemp &particles,
                            const FloatKeys &keys,
                            std::string filename,
                            bool append) {
  IMP_DEPRECATED(write_particles_binary, IMP.hdf5);
  NcFile::FileMode mode;
  // replace on 0 also
  if (append) {
    mode=NcFile::Write;
  } else {
    mode=NcFile::Replace;
  }
  NcFile f(filename.c_str(), mode /*,NULL, 0, NcFile::Netcdf4*/);
  if (!f.is_valid()) {
    IMP_THROW("Unable to open file " << filename << " for writing",
              IOException);
  }
  const NcDim* dims[2];
  if (append && f.num_vars() > 0) {
    dims[0]= f.get_dim("particles");
    dims[1]= f.get_dim("values");
    if (static_cast<unsigned int>(dims[0]->size()) != particles.size()) {
      IMP_THROW("Number of particles (" << particles.size()
                << ") does not match expected (" << dims[0]->size()
                << ")", IOException);
    }
    if (static_cast<unsigned int>(dims[1]->size()) != keys.size()) {
      IMP_THROW("Number of keys (" << keys.size()
                << ") does not match expected (" << dims[1]->size()
                << ")", IOException);
    }
  } else {
    dims[0]= f.add_dim("particles", particles.size());
    dims[1]= f.add_dim("values", keys.size());
  }
  NcVar *cur=NULL;
  if (append) {
    std::ostringstream oss;
    oss << f.num_vars();
    cur = f.add_var(oss.str().c_str(), ncDouble, 2, dims);
  } else {
    cur = f.add_var("0", ncDouble, 2, dims);
  }
  boost::scoped_array<double> values(new double[particles.size()*keys.size()]);
  write_particles_to_buffer(particles, keys,
                            reinterpret_cast<char*>(values.get()),
                            particles.size()*keys.size()*sizeof(double));
  cur->put(values.get(), particles.size(), keys.size());
}

void read_particles_binary(NcFile &f,
                       const ParticlesTemp &particles,
                       const FloatKeys &keys,
                       int var_index) {
  IMP_DEPRECATED(read_particles_binary, IMP.hdf5);
  if (var_index >= f.num_vars()) {
    IMP_THROW("Illegal component of file requested " << var_index
              << ">=" << f.num_vars(), IOException);
  }
  NcVar *data=f.get_var(var_index);
  boost::scoped_array<double> values(new double[particles.size()*keys.size()]);
  data->get(values.get(), particles.size(), keys.size());
  read_particles_from_buffer(reinterpret_cast<char*>(values.get()),
                             particles.size()*keys.size()*sizeof(double),
                             particles, keys);
  IMP_CHECK_MODEL_PARTICLES(particles[0]->get_model());
}

void read_particles_binary(std::string filename,
                       const ParticlesTemp &particles,
                       const FloatKeys &keys,
                       int frame) {
  IMP_DEPRECATED(read_particles_binary, IMP.hdf5);
  NcFile f(filename.c_str(), NcFile::ReadOnly
           /*,NULL, 0, NcFile::Netcdf4*/);
  if (!f.is_valid()) {
    IMP_THROW("Unable to open file " << filename << " for reading",
              IOException);
  }
  int index=-1;
  if (frame>=0) {
    std::ostringstream oss;
    oss << frame;
    for ( int i=0; i< f.num_vars(); ++i) {
      NcVar *v= f.get_var(i);
      if (std::string(v->name())== oss.str()) {
        index=i;
        break;
      }
    }
    if (index==-1) {
      std::string vars;
      for ( int i=0; i< f.num_vars(); ++i) {
        NcVar *v= f.get_var(i);
        vars= vars+" "+std::string(v->name());
      }
      IMP_THROW("Unable to find frame \"" << oss.str()
                << "\" found frames are " << vars, IOException);
    }
  } else {
    index=0;
  }
  read_particles_binary(f, particles, keys, index);
  IMP_CHECK_MODEL_PARTICLES(particles[0]->get_model());
}
#endif


#endif // IMP_USE_DEPRECATED



IMPEXPORT std::vector<char>
write_particles_to_buffer(const ParticlesTemp &particles,
                          const FloatKeys &keys) {
  if (particles.empty() || keys.empty()) {
    return std::vector<char>();
  }
  unsigned int size= particles.size()*keys.size()*sizeof(double);
  std::vector<char> ret(size);
  write_particles_to_buffer(particles, keys, &ret.front(), size);
  return ret;
}
IMPEXPORT void read_particles_from_buffer( const std::vector<char> &buffer,
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
