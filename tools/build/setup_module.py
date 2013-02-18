#!/usr/bin/python

"""
Set up a module by
- checking that all required modules and dependencies are found
- creating the config header and .cpp and version check .py
- linking .py files from the bin and benchmarks directories into the build dir

If the module cannot be configured, the script exits with an error.
"""

import sys
from optparse import OptionParser
import os.path
import tools
import glob

header_template="""
/*
 * \file %(filename)s
 * \brief Provide macros to mark functions and classes as exported
 *        from a DLL/.so, and to set up namespaces
 *
 * When building the module, %(cppprefix)s_EXPORTS should be defined, and when
 * using the module externally, it should not be. Classes and functions
 * declared in the module's headers should then be marked with
 * %(cppprefix)sEXPORT if they are intended to be part of the API and
 * they are not defined entirely in a header.
 *
 * The Windows build environment requires applications to mark exports in
 * this way; we use the same markings to set the visibility of ELF symbols
 * if we have compiler support.
 *
 * All code in this module should live in the IMP::base namespace.
 * This is simply achieved by wrapping things with the
 * %(cppprefix)s_BEGIN_NAMESPACE and %(cppprefix)s_END_NAMESPACE macros.
 * There are similar macros for module code that is designed to be for
 * internal use only.
 *
 * This header is auto-generated by sconstools/module/_config_h.py;
 * it should not be edited manually.
 *
 * Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#ifndef %(cppprefix)s_CONFIG_H
#define %(cppprefix)s_CONFIG_H

#include <IMP/base/base_config.h>
#include <string>


#ifdef _MSC_VER

#ifdef %(cppprefix)s_EXPORTS
#define %(cppprefix)sEXPORT __declspec(dllexport)
#else // EXPORTS
#define %(cppprefix)sEXPORT __declspec(dllimport)
#endif // EXPORTS

#else // _MSC_VER

#ifdef GCC_VISIBILITY
#define %(cppprefix)sEXPORT __attribute__ ((visibility("default")))
#else // GCC_VISIBILITY
#define %(cppprefix)sEXPORT
#endif // GCC_VISIBILITY
#endif // _MSC_VER

#if defined(_MSC_VER) && !defined(SWIG)
#ifdef %(cppprefix)s_EXPORTS

#define %(cppprefix)s_EXPORT_TEMPLATE(name)                 template class __declspec(dllexport) name

#else //EXPORTS

#define %(cppprefix)s_EXPORT_TEMPLATE(name)                 template class __declspec(dllimport) name

#endif // EXPORTS

#else // MSC and SWIG
#define %(cppprefix)s_EXPORT_TEMPLATE(name) IMP_REQUIRE_SEMICOLON_NAMESPACE

#endif // MSC and SWIG

#define %(cppprefix)s_BEGIN_NAMESPACE \\
%(namespacebegin)s

#define %(cppprefix)s_END_NAMESPACE \\
%(namespaceend)s

#define %(cppprefix)s_BEGIN_INTERNAL_NAMESPACE \\
%(cppprefix)s_BEGIN_NAMESPACE namespace internal {


#define %(cppprefix)s_END_INTERNAL_NAMESPACE \\
} %(cppprefix)s_END_NAMESPACE

%(cppdefines)s

//  functions are defined explicitly for swig

%(cppprefix)s_BEGIN_NAMESPACE
/** \name Standard module methods
  All \imp modules have a set of standard methods to help get information
  about the module and about file associated with the modules.
  @{
  */
#if !defined(SWIG)
%(cppprefix)sEXPORT std::string get_module_version();
#endif

#if !defined(SWIG)
// swig will whine about duplicate definitions of function
inline std::string get_module_name() {
   return "IMP::%(name)s";
}
#endif

%(cppprefix)s_END_NAMESPACE

#if !defined(SWIG) && !defined(IMP_DOXYGEN) && %(is_not_base)s

#include <IMP/base/Showable.h>
#include <IMP/base/hash.h>

%(cppprefix)s_BEGIN_NAMESPACE
using ::IMP::base::Showable;
using ::IMP::base::operator<<;
using ::IMP::base::hash_value;
%(cppprefix)s_END_NAMESPACE
%(cppprefix)s_BEGIN_INTERNAL_NAMESPACE
using ::IMP::base::Showable;
using ::IMP::base::operator<<;
using ::IMP::base::hash_value;
%(cppprefix)s_END_INTERNAL_NAMESPACE

#endif //!defined(SWIG) && !defined(IMP_DOXYGEN) && %(is_not_base)s

#if !defined(SWIG)

%(cppprefix)s_BEGIN_NAMESPACE

//! Return the full path to installed data
/** Each module has its own data directory, so be sure to use
    the version of this function in the correct module. To read
    the data file "data_library" that was placed in the \c data
    directory of module "mymodule", do something like
    \code
    std::ifstream in(IMP::mymodule::get_data_path("data_library"));
    \endcode
    This will ensure that the code works when \imp is installed or
    used via the \c tools/imppy.sh script.
*/
%(cppprefix)sEXPORT std::string get_data_path(std::string file_name);

//! Return the path to installed example data for this module
/** Each module has its own example directory, so be sure to use
    the version of this function in the correct module.  For example
    to read the file \c example_protein.pdb located in the
    \c examples directory of the IMP::atom module, do
    \code
    IMP::atom::read_pdb(IMP::atom::get_example_path("example_protein.pdb", model));
    \endcode
    This will ensure that the code works when \imp is installed or
    used via the \c tools/imppy.sh script.
*/
%(cppprefix)sEXPORT std::string get_example_path(std::string file_name);
/** @} */


%(cppprefix)s_END_NAMESPACE

#endif // SWIG

#include <IMP/base/compiler_macros.h>

#endif  /* %(cppprefix)s_CONFIG_H */
"""

cpp_template="""/**
 *  \\file config.cpp
 *  \\brief base module version information.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include <%(filename)s>
#include <IMP/base/internal/directories.h>

%(cppprefix)s_BEGIN_NAMESPACE

std::string get_module_version() {
    return std::string("%(version)s");
}
std::string get_data_path(std::string file_name) {
  return IMP::base::internal::get_data_path("%(name)s", file_name);
}
std::string get_example_path(std::string file_name)  {
  return IMP::base::internal::get_example_path("%(name)s", file_name);
}

%(cppprefix)s_END_NAMESPACE
"""

parser = OptionParser()
parser.add_option("-D", "--defines", dest="defines", default="",
                  help="Colon separated list of defines.")
parser.add_option("-n", "--name",
                  dest="name", help="The name of the module.")
parser.add_option("-s", "--source",
                  dest="source", help="The root for IMP source.")
parser.add_option("-d", "--datapath",
                  dest="datapath", default="", help="An extra IMP datapath.")

def get_version(options):
    module_version= os.path.join(options.source, "modules", options.name, "VERSION")
    if os.path.exists(module_version):
        return open(module_version, "r").read().split("\n")[0]
    else:
        imp_version= os.path.join(options.source, "VERSION")
        if os.path.exists(imp_version):
            return open(imp_version, "r").read().split("\n")[0]
        else:
            return "develop"

def add_list_to_defines(cppdefines, data, sym, val, names):
    names.sort()
    for n in names:
        nn= n.replace(".", "_").upper()
        cppdefines.append("#define IMP_%s_%s_%s"%(data["name"].upper(), sym, nn))
        cppdefines.append("#define IMP_%s_HAS_%s %d"%(data["name"].upper(), nn, val))

def make_header(options):
    dir= os.path.join("include", "IMP", options.name)
    file=os.path.join(dir, "%s_config.h"%options.name)
    try:
        os.makedirs(dir)
    except:
        #exists
        pass

    data={}
    data["name"]= options.name
    data["filename"]="IMP/%s/%s_config.h"%(options.name, options.name)
    data["cppprefix"]="IMP%s"%options.name.upper().replace("_", "")
    data["namespacebegin"]="namespace IMP { namespace %s {"%options.name
    data["namespaceend"]="} }"
    if data["name"] !="base":
        data["is_not_base"]=1
    else:
        data["is_not_base"]=0

    cppdefines=[]
    if options.defines != "":
        for define in tools.split(options.defines):
            parts= define.split("=")
            if len(parts) ==2:
                cppdefines.append("#define %s %s"%(parts[0], parts[1]))
            else:
                cppdefines.append("#define %s"%parts[0])

    required_modules=""
    lib_only_required_modules=""
    required_dependencies=""
    optional_dependencies=""
    exec open(os.path.join(options.source, "modules", data["name"], "description"), "r").read()

    info= tools.get_module_info(data["name"], options.datapath)

    optional_modules=[x for x in info["modules"] if x not in tools.split(required_modules) and x != ""]
    unfound_modules=[x for x in info["unfound_modules"] if x != ""]
    optional_dependencies=[x for x in info["dependencies"] if x not in tools.split(required_dependencies) and x != ""]
    unfound_dependencies=[x for x in info["unfound_dependencies"] if x != ""]
    add_list_to_defines(cppdefines, data, "USE", 1,
                        ["imp_"+x for x in optional_modules])
    add_list_to_defines(cppdefines, data, "NO", 0,
                        ["imp_"+x for x in unfound_modules])
    add_list_to_defines(cppdefines, data, "USE", 1, optional_dependencies)
    add_list_to_defines(cppdefines, data, "NO", 0, info["unfound_dependencies"])
    data["cppdefines"]="\n".join(cppdefines)
    tools.rewrite(file, header_template%data)

def make_cpp(options):
    dir= os.path.join("src", options.name)
    file=os.path.join(dir, "config.cpp")
    try:
        os.makedirs(dir)
    except:
        # exists
        pass
    data={}
    data["filename"]="IMP/%s/%s_config.h"%(options.name, options.name)
    data["cppprefix"]="IMP%s"%options.name.upper().replace("_", "")
    data["name"]= options.name
    data["version"]= get_version(options)
    tools.rewrite(file, cpp_template%data)

def make_version_check(options):
    dir= os.path.join("lib", "IMP", options.name)
    tools.mkdir(dir, clean=False)
    outf= os.path.join(dir, "_version_check.py")
    template="""def check_version(myversion):
  def _check_one(name, expected, found):
    if expected != found:
      raise RuntimeError('Expected version '+expected+' but got '+ found \
           +' when loading module '+name\
            +'. Please make sure IMP is properly built and installed and that matching python and C++ libraries are used.')
  _check_one('%s', '%s', myversion)
  """
    tools.rewrite(outf, template%(options.name, get_version(options)))

def write_no_ok(module):
    new_order= [x for x in tools.get_sorted_order() if x != module]
    tools.set_sorted_order(new_order)
    tools.rewrite(os.path.join("data", "build_info", "IMP."+module), "ok=False\n")

def write_ok(module, modules, unfound_modules, dependencies, unfound_dependencies,
             swig_includes, swig_wrapper_includes):
    print "yes"
    config=["ok=True"]
    if len(modules) > 0:
        config.append("modules = \"" + ":".join(modules)+"\"")
    if len(unfound_modules) > 0:
        config.append("unfound_modules = \""+ ":".join(unfound_modules)+"\"")
    if len(dependencies) > 0:
        config.append("dependencies = \"" + ":".join(dependencies)+"\"")
    if len(unfound_dependencies) > 0:
        config.append("unfound_dependencies = \"" + ":".join(unfound_dependencies)+"\"")
    if len(swig_includes) > 0:
        config.append("swig_includes = \"" + ":".join(swig_includes)+"\"")
    if len(swig_wrapper_includes) > 0:
        config.append("swig_wrapper_includes = \"" + ":".join(swig_wrapper_includes)+"\"")
    tools.rewrite(os.path.join("data", "build_info", "IMP."+module), "\n".join(config))

def setup_module(module, source, datapath):
    print "Configuring module", module, "...",
    data= tools.get_module_description(source, module, datapath)
    for d in data["required_dependencies"]:
        if not tools.get_dependency_info(d, datapath)["ok"]:
            print d, "not found"
            write_no_ok(module)
            return False
    dependencies = data["required_dependencies"]
    unfound_dependencies = []
    for d in data["optional_dependencies"]:
        if tools.get_dependency_info(d, datapath)["ok"]:
            dependencies.append(d)
        else:
            unfound_dependencies.append(d)
    for d in data["required_modules"]:
        if not tools.get_module_info(d, datapath)["ok"]:
            print "IMP."+d, "not found"
            write_no_ok(module)
            return False
    modules= data["required_modules"]
    unfound_modules = []
    for d in data["optional_modules"]:
        if tools.get_module_info(d, datapath)["ok"]:
            modules.append(d)
        else:
            unfound_modules.append(d)
    all_modules=tools.get_dependent_modules(modules, datapath)
    swig_includes=[os.path.split(x)[1] for x
                   in tools.get_glob([os.path.join(source, "modules", module,
                                                   "pyext", "include", "*.i")])]\
                 + ["IMP/"+module+"/"+os.path.split(x)[1] for x
                            in tools.get_glob([os.path.join("include", "IMP", module, "*_macros.h")])]
    swig_wrapper_includes= ["IMP/"+module+"/internal/"+os.path.split(x)[1] for x
                   in tools.get_glob([os.path.join(source, "modules", module, "include", "internal", "swig*.h")])]
    tools.mkdir(os.path.join("src", module))
    tools.mkdir(os.path.join("src", module+"_swig"))
    write_ok(module, all_modules,
             unfound_modules, tools.get_dependent_dependencies(all_modules, dependencies,datapath),
             unfound_dependencies, swig_includes, swig_wrapper_includes)
    return True

def link_bin(options):
    path = os.path.join("module_bin", options.name)
    tools.mkdir(path, clean=False)
    for old in tools.get_glob([os.path.join(path, "*.py")]):
        os.unlink(old)
    tools.link_dir(os.path.join(options.source, "modules", options.name, "bin"), path, clean=False, match=["*.py"])

def link_benchmark(options):
    path = os.path.join("benchmark", options.name)
    tools.mkdir(path, clean=False)
    for old in tools.get_glob([os.path.join(path, "*.py")]):
        os.unlink(old)
    tools.link_dir(os.path.join(options.source, "modules", options.name, "benchmark"), path, clean=False, match=["*.py"])

def main():
    (options, args) = parser.parse_args()
    disabled= tools.split(open("data/build_info/disabled", "r").read(), "\n")
    if options.name in disabled:
        print options.name, "is disabled"
        write_no_ok(options.name)
        tools.rmdir(os.path.join("module_bin", options.name))
        tools.rmdir(os.path.join("benchmark", options.name))
        exit(1)
    if setup_module(options.name, options.source, options.datapath):
        make_header(options)
        make_cpp(options)
        make_version_check(options)
        link_bin(options)
        link_benchmark(options)
        exit(0)
    else:
        tools.rmdir(os.path.join("module_bin", options.name))
        tools.rmdir(os.path.join("benchmark", options.name))
        exit(1)

if __name__ == '__main__':
    main()
