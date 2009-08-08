"""Tools and Builders for IMP modules. See `IMPModule` for more information."""

import os.path
import pyscanner
import examples
import test
import swig


from SCons.Script import Builder, File, Action, Glob, Return, Alias, Dir
import hierarchy

#def module_depends(env, target, source):
#    env.Depends(target, [env.Alias(env['IMP_MODULE']+"-"+source)])

def file_compare(a, b):
    pa= a.abspath
    pb= b.abspath
    return cmp(pa,pb)

def module_glob(patterns):
    ret=[]
    for x in patterns:
        ret= ret+Glob(x)
    ret.sort()#cmp= file_compare)
    return ret

def module_requires(env, target, source):
    env.Requires(target, [env.Alias(env['IMP_MODULE']+"-"+source)])

def module_alias(env, target, source, is_default=False):
    a=env.Alias(env['IMP_MODULE']+"-"+target, [source])
    if is_default:
        env.Default(a)

def add_to_global_alias(env, target, source):
    env.Alias(env.Alias(target), [env.Alias(env['IMP_MODULE']+'-'+source)])


def add_to_module_alias(env, target, source):
    env.Alias(env.Alias(env['IMP_MODULE']+'-'+target),
              [env.Alias(env['IMP_MODULE']+'-'+source)])


#def module_deps_depends(env, target, source, dependencies):
#    env.Depends(target,
#              [env.Alias(x+'-'+source) for x in dependencies])

def module_deps_requires(env, target, source, dependencies):
    env.Requires(target,
              [env.Alias(x+'-'+source) for x in dependencies])


def do_mac_name_thing(env, source, target):
    targetdir= os.path.split(target[0].abspath)[0]
    sourcedir= os.path.split(source[0].abspath)[0]
    #print targetdir
    #print sourcedir
    env.Execute("install_name_tool -id %s %s"% (target[0].abspath, target[0].abspath))
    env.Execute("install_name_tool -change %s %s %s"%(os.path.join(sourcedir, 'libimp.dylib'),
                                                      os.path.join(targetdir, 'libimp.dylib'),
                                                      target[0].abspath))
    for m in env['IMP_MODULES_ALL']:
        oname=os.path.join(sourcedir, "libimp_"+m+".dylib")
        nname=os.path.join(targetdir, "libimp_"+m+".dylib")
        #print oname
        #print nname
        env.Execute("install_name_tool -change %s %s %s"%(oname,
                                                          nname,
                                                          target[0].abspath))
def postprocess_lib(env, target):
    """ for now assume that all libs go in the same place"""
    if env['PLATFORM'] == 'darwin':
        dir= os.path.split(target[0].abspath)[0]
        env.AddPostAction(target, do_mac_name_thing)


def make_vars(env):
    """Make a map which can be used for all string substitutions"""
    module = env['IMP_MODULE']
    module_include_path = env['IMP_MODULE_INCLUDE_PATH']
    module_src_path = env['IMP_MODULE_SRC_PATH']
    module_preproc = env['IMP_MODULE_PREPROC']
    module_namespace = env['IMP_MODULE_NAMESPACE']
    module_suffix = env['IMP_MODULE_SUFFIX']
    version = env['IMP_MODULE_VERSION']#source[1].get_contents()
    nicename= env['IMP_MODULE_NICENAME']
    author = nicename+" development team"
    vars={'module_include_path':module_include_path,
          'module_src_path':module_src_path, 'module':module,
          'PREPROC':module_preproc, 'author':author, 'version':version,
          'namespace':module_namespace,
          'module_suffix':module_suffix,
          'module_nicename':nicename}
    return vars


def action_config(target, source, env):
    """The IMPModuleConfig Builder generates a configuration header file
       used to mark classes and functions for export and to define namespaces,
       and a corresponding SWIG interface, e.g.
       env.IMPModuleConfig(('config.h', 'foo_config.i'), env.Value('foo'))
       generates a configuration header and interface for the 'foo' module."""
    vars= make_vars(env)
    h = file(target[0].abspath, 'w')
    i = file(target[1].abspath, 'w')
    vars['filename']=os.path.basename(target[0].abspath)
    print >> h, """/*
 * \\file %(filename)s
 * \\brief Provide macros to mark functions and classes as exported
 *        from a DLL/.so, and to set up namespaces
 *
 * When building the module, %(PREPROC)s_EXPORTS should be defined, and when
 * using the module externally, it should not be. Classes and functions
 * defined in the module's headers should then be marked with
 * %(PREPROC)sEXPORT if they are intended to be part of the API, or with
 * %(PREPROC)sLOCAL if they are not (the latter is the default).
 *
 * The Windows build environment requires applications to mark exports in
 * this way; we use the same markings to set the visibility of ELF symbols
 * if we have compiler support.
 *
 * All code in this module should live in the %(namespace)s namespace.
 * This is simply achieved by wrapping things with the
 * %(PREPROC)s_BEGIN_NAMESPACE and %(PREPROC)s_END_NAMESPACE macros.
 * There are similar macros for module code that is designed to be for
 * internal use only.
 *
 * This header is auto-generated by tools/imp-module.py; it should not be
 * edited manually.
 *
 * Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef %(PREPROC)s_CONFIG_H
#define %(PREPROC)s_CONFIG_H

#ifdef _MSC_VER
#ifdef %(PREPROC)s_EXPORTS
#define %(PREPROC)sEXPORT __declspec(dllexport)
#else
#define %(PREPROC)sEXPORT __declspec(dllimport)
#endif
#define %(PREPROC)sLOCAL
#else
#ifdef GCC_VISIBILITY
#define %(PREPROC)sEXPORT __attribute__ ((visibility("default")))
#define %(PREPROC)sLOCAL __attribute__ ((visibility("hidden")))
#else
#define %(PREPROC)sEXPORT
#define %(PREPROC)sLOCAL
#endif
#endif
""" % vars

    print >> i, """/* Ignore shared object exports macros */
#define %(PREPROC)sEXPORT
#define %(PREPROC)sLOCAL
""" % vars

    for out in (i, h):
        print >> out, "#define %(PREPROC)s_BEGIN_NAMESPACE \\"%vars
        for comp in vars['namespace'].split("::"):
            print >> out, "namespace %s {\\" %comp
        print >> out
        print >> out, "#define %(PREPROC)s_END_NAMESPACE \\"%vars
        for comp in vars['namespace'].split("::"):
            print >> out, "} /* namespace %s */ \\" %comp
        print >> out
        print >> out, """#define %(PREPROC)s_BEGIN_INTERNAL_NAMESPACE \\
%(PREPROC)s_BEGIN_NAMESPACE \\
namespace internal {
""" %vars
        print >> out
        print >> out, """#define %(PREPROC)s_END_INTERNAL_NAMESPACE \\
} /* namespace internal */ \\
%(PREPROC)s_END_NAMESPACE
""" %vars

    print >> h, """

#include <%(module_include_path)s/internal/version_info.h>
#include <IMP/internal/config.h>

#endif  /* %(PREPROC)s_CONFIG_H */""" % vars


def action_version_info(target, source, env):
    """The IMPModuleVersionInfo Builder generates a source file and header to
       return version information, e.g.
       env.IMPModuleVersionInfo(('src/internal/version_info.cpp',
                                 'include/internal/version_info.h'),
                                (env.Value('foo'), env.Value('Me'),
                                 env.Value('1.0')))
       generates version information for the 'foo' module."""
    vars= make_vars(env)

    cpp = file(target[0].abspath, 'w')
    h = file(target[1].abspath, 'w')

    for (f, ext) in ((cpp, 'cpp'), (h, 'h')):
        vars['ext']=ext
        print >> f, """/**
 *  \\file %(module_include_path)s/internal/version_info.%(ext)s
 *  \\brief %(module)s module version information.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */
""" % vars

    print >> h, """#ifndef %(PREPROC)s_INTERNAL_VERSION_INFO_H
#define %(PREPROC)s_INTERNAL_VERSION_INFO_H

#include "../config.h"

""" % vars

    print >> cpp, """#include <%(module_include_path)s/internal/version_info.h>
#include <IMP/VersionInfo.h>
""" \
                  % vars

    print >> h, """namespace IMP {
class VersionInfo;
}
""" %vars

    for f in (h, cpp):
        print >> f, "%(PREPROC)s_BEGIN_INTERNAL_NAMESPACE\n" % vars

    print >> h, """//! Version and authorship of the %(module)s module.
extern %(PREPROC)sEXPORT VersionInfo version_info;""" \
        % vars

    print >> cpp, 'VersionInfo version_info("%(author)s", "%(version)s");' \
              %vars

    for f in (h, cpp):
        print >> f, "\n%(PREPROC)s_END_INTERNAL_NAMESPACE" % vars

    print >> h, "\n#endif  /* %(PREPROC)s_INTERNAL_VERSION_INFO_H */" % vars

def action_link_test(target, source, env):
    """The IMPModuleLinkTesto Builder generates a source file. By linking in two
    of these, any functions which are defined in headers but not declared inline are detected"""
    cpp = file(target[0].abspath, 'w')
    vars= make_vars(env)
    vars['fname']=os.path.split(target[0].abspath)[1]
    print >> cpp, """/**
 *  \\file %(module_include_path)s/internal/%(fname)s
 *  \\brief Test linking for non-inlined functions.
 *
 *  This file is auto-generated, do not edit.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */
""" % vars

    print >> cpp, """
#ifndef NDEBUG
#include "%(module_include_path)s.h"
#endif""" % vars


def IMPModuleLib(envi, files):
    """Build, and optionally also install, an IMP module's C++
       shared library. This is only available from within an environment
       created by `IMPSharedLibraryEnvironment`."""
    from scons_tools import get_sharedlib_environment
    vars= make_vars(envi)
    env = get_sharedlib_environment(envi, '%(PREPROC)s_EXPORTS' % vars,
                                    cplusplus=True)
    module = env['IMP_MODULE']
    module_suffix = env['IMP_MODULE_SUFFIX']
    vars= make_vars(env)
    files =files+['internal/link_0.cpp', 'internal/link_1.cpp', 'internal/version_info.cpp']
    if env.get('static', False) and env['CC'] == 'gcc':
        build = env.StaticLibrary('#/build/lib/imp%s' % module_suffix,
                                      list(files))
    else:
        build = env.SharedLibrary('#/build/lib/imp%s' % module_suffix,
                                  list(files) )
        postprocess_lib(env, build)
    install = env.Install(env.GetInstallDirectory('libdir'), build)
    postprocess_lib(env, install)
    module_requires(env, build, 'include')
    module_requires(env, build, 'data')
    module_alias(env, 'lib', build, True)
    add_to_global_alias(env, 'all', 'lib')
    module_alias(env, 'install-lib', install)
    add_to_module_alias(env, 'install', 'install-lib')
    module_deps_requires(env, build, 'include', env['IMP_REQUIRED_MODULES'])
    module_deps_requires(env, build, 'lib', env['IMP_REQUIRED_MODULES'])
    module_deps_requires(env, install, 'install-lib', env['IMP_REQUIRED_MODULES'])


def IMPModuleInclude(env, files):
    """Install the given header files, plus any auto-generated files for this
       IMP module."""
    vars=make_vars(env)
    includedir = env.GetInstallDirectory('includedir')
    files=files+['config.h', 'internal/version_info.h']
    install = hierarchy.InstallHierarchy(env, includedir+"/"+vars['module_include_path'],
                                         list(files))
    build=hierarchy.InstallHierarchy(env, "#/build/include/"+vars['module_include_path'],
                                     list(files), True)
    module_alias(env, 'include', build)
    add_to_global_alias(env, 'all', 'include')
    module_alias(env, 'install-include', install)
    add_to_module_alias(env, 'install', 'install-include')

def IMPModuleData(env, files):
    """Install the given data files for this IMP module."""
    vars=make_vars(env)
    datadir = env.GetInstallDirectory('datadir')
    install = hierarchy.InstallDataHierarchy(env, datadir+"/"+vars['module_include_path'], files, False)
    build = hierarchy.InstallDataHierarchy(env, "#/build/data/"+vars['module_include_path'], files, True)
    module_alias(env, 'data', build)
    add_to_global_alias(env, 'all', 'data')
    module_alias(env, 'install-data', install)
    add_to_module_alias(env, 'install', 'install-data')


def IMPModuleExamples(env, files, required_modules=[]):
    vars=make_vars(env)
    #for f in files:
    #    print f.abspath
    (build, install, test)= examples.handle_example_dir(env, Dir("."), vars['module'], vars['module_include_path'], files)
    module_alias(env, 'examples', build)
    add_to_global_alias(env, 'all', 'examples')
    module_alias(env, 'install-examples', install)
    add_to_global_alias(env, 'doc-install', 'install-examples')
    module_alias(env, 'test-examples', test)
    add_to_global_alias(env, 'test', 'test-examples')
    module_requires(env, test, 'python')
    add_to_global_alias(env, 'doc', 'examples')
    module_deps_requires(env, test, 'python', required_modules)

def IMPModuleBin(envi, files, required_modules=[], extra_libs=[], install=True):
    from scons_tools import get_bin_environment
    env= get_bin_environment(envi)
    vars=make_vars(env)
    env.Prepend(LIBS=(['imp%(module_suffix)s' % vars]+["imp_"+x for x in required_modules]))
    env.Append(LIBS=extra_libs);
    build=[]
    install_list=[]
    bindir = env.GetInstallDirectory('bindir')
    for f in files:
        prog= env.Program(f)
        cb= env.Install("#/build/bin", prog)
        ci= env.Install(bindir, prog)
        build.append(cb)
        if install:
            install_list.append(ci)
        build.append(prog)
    module_alias(env, 'bin', build, True)
    add_to_global_alias(env, 'all', 'bin')
    if install:
        module_alias(env, 'install-bin', install_list)
        add_to_module_alias(env, 'install', 'install-bin')
    module_requires(env, build, 'include')
    module_requires(env, build, 'lib')
    module_requires(env, build, 'data')
    module_deps_requires(env, build, 'lib', required_modules)


def IMPModulePython(env):
    """Build and install an IMP module's Python extension and the associated
       wrapper file from a SWIG interface file. This is only available from
       within an environment created by `IMPPythonExtensionEnvironment`."""
    from scons_tools import get_pyext_environment
    module = env['IMP_MODULE']
    module_suffix= env['IMP_MODULE_SUFFIX']
    vars=make_vars(env)
    build=[]
    install=[]
    if env['IMP_MODULE_CPP']:
        penv = get_pyext_environment(env, module.upper(), cplusplus=True)
        if penv['CC'] != 'w32cc':
            penv['LIBS']=[]
        penv.Prepend(LIBS=['imp%s' % module_suffix])
        #penv.Append(CPPPATH=[Dir('#').abspath])
        #penv.Append(SWIGFLAGS='-python -c++ -naturalvar')
        interfaces= module_glob(["*.i"])
        for i in interfaces:
            cb= env.LinkInstallAs("#/build/swig/"+str(i), i)
            build.append(cb)

        swig_interface=File(module+".i")
        globlist=["#/build/include/%(module_include_path)s/*.h"%vars]\
            + ["#/module/"+x+"/pyext/*.i" for x in env['IMP_REQUIRED_MODULES']]\
            + ["*.i"]
        #print globlist
        deps= module_glob(globlist)
        penv._IMPSWIG(target=['wrap.cc',
                              'wrap.h'],
                      source=["%(module)s.i"%vars]+deps)
        build.append(penv._IMPPatchSWIG(target=['patched_wrap.cc'],
              source=['wrap.cc']))
        build.append(penv._IMPPatchSWIG(target=['patched_wrap.h'],
              source=['wrap.h']))
        buildlib = penv.LoadableModule('#/build/lib/_IMP%s' % module_suffix,
                                       "patched_wrap.cc"%vars)
        # Place the generated Python wrapper in lib directory:
        gen_pymod = File('IMP%s.py' % module_suffix.replace("_","."))
        env.Depends(gen_pymod, buildlib)
        buildinit = penv.LinkInstallAs('#/build/lib/%s/__init__.py'
                                      % vars['module_include_path'],
                                      gen_pymod)
        installinit = penv.InstallAs(penv.GetInstallDirectory('pythondir',
                                                            vars['module_include_path'],
                                                            '__init__.py'),
                                    gen_pymod)
        installlib = penv.Install(penv.GetInstallDirectory('pyextdir'), buildlib)
        postprocess_lib(penv, buildlib)
        build.append(buildlib)
        build.append(buildinit)
        install.append(installinit)
        install.append(installlib)
        build.append(swig_interface)
    files = module_glob(['src/*.py'])
    #print [x.path for x in Glob("*")]
    #print Dir("src").path
    #print [x.path for x in Glob("src/*")]
    #print [x.path for x in Glob("src/*.py")]
    for f in files:
        #print f
        nm= os.path.split(f.path)[1]
        #print ('#/build/lib/%s/'+nm) % vars['module_include_path']
        build.append(env.LinkInstallAs(('#/build/lib/%s/'+nm) % vars['module_include_path'],
                                       f))
        install.append(env.InstallAs(env.GetInstallDirectory('pythondir',
                                                             vars['module_include_path'],
                                                             nm),f))
    # Install the Python extension and module:
    #buildlib = env.Install("#/build/lib", pyext)
    module_alias(env, 'python', build, True)
    add_to_global_alias(env, 'all', 'python')
    module_alias(env, 'install-python', install)
    add_to_module_alias(env, 'install', 'install-python')
    if env['IMP_MODULE_CPP']:
        module_requires(env, build, 'include')
        module_requires(env, build, 'lib')
        module_requires(env, build, 'config')
        module_deps_requires(env, build, 'python',env['IMP_REQUIRED_MODULES'] )
        module_deps_requires(env, build, 'include', env['IMP_REQUIRED_MODULES'])
        module_deps_requires(env, install, 'install-lib', env['IMP_REQUIRED_MODULES'])

def IMPModuleGetExamples(env):
    vars= make_vars(env)
    files=module_glob(["*.py", "*/*.py","*.readme","*/*.readme"])
    return files

def IMPModuleGetHeaders(env):
    vars = make_vars(env)
    raw_files=module_glob(["*.h", "*/*.h"])
    files=[]
    for f in raw_files:
        s= str(f)
        #print s
        fname= os.path.split(s)[1]
        if fname.startswith("."):
            continue
        if s== "internal/version_info.h":
            continue
        if s=="config.h":
            continue
        files.append(f)
    return files

def IMPModuleGetSources(env):
    vars = make_vars(env)
    raw_files=module_glob(["*.cpp", "*/*.cpp"])
    files=[]
    for f in raw_files:
        s= str(f)
        #print s
        fname= os.path.split(s)[1]
        if fname.startswith("."):
            continue
        if s== "internal/link_0.cpp":
            continue
        if s== "internal/link_1.cpp":
            continue
        if s=="internal/version_info.cpp":
            continue
        files.append(f)
    return files

def IMPModuleGetData(env):
    vars = make_vars(env)
    raw_files=module_glob(["*"])
    files=[]
    for f in [os.path.split(str(x))[1] for x in raw_files]:
        if str(f).endswith("SConscript"):
            continue
        if str(f).endswith(".old"):
            continue
        if str(f).startswith("."):
            continue
        files.append(f)
    return files

def IMPModuleGetBins(env):
    vars = make_vars(env)
    raw_files= module_glob(["*.cpp"])
    return raw_files

def IMPModuleGetDocs(env):
    files=module_glob(["*.dox.in", "*.dox", "*.pdf"])
    return files


def IMPModuleDoc(env, files):
    vars= make_vars(env)
    build=[]
    #install=[]
    docdir=env['docdir']+"/"+vars['module_include_path']
    for f in files:
        if str(f).endswith(".dox"):
            pass
        else:
            build.append(env.Install(f, "#/doc/html/"+vars['module']))
            #install.append(env.Install(f, docdir))
    module_alias(env, 'doc', build)
    add_to_global_alias(env, 'all', 'doc')
    #module_alias(env, 'install-doc', install)
    #add_to_module_alias(env, 'install', 'install-doc')



#   files= ["#/bin/imppy.sh", "#/tools/run_all_tests.py"]+\
#        [x.abspath for x in Glob("test_*.py")+ Glob("*/test_*.py")]

def IMPModuleTest(env, required_modules=[], **keys):
    """Pseudo-builder to run tests for an IMP module. The single target is
       generally a simple output file, e.g. 'test.passed', while the single
       source is a Python script to run (usually run-all-tests.py).
       Right now, the assumption is made that run-all-tests.py executes
       all files called test_*.py in the current directory and subdirectories.
       If the TEST_ENVSCRIPT construction variable is set, it is a shell
       script to run to set up the environment to run the test script.
       A convenience alias for the tests is added, and they are always run."""
    files= ["#/bin/imppy.sh", "#/tools/run-all-tests.py"]+\
        [x.abspath for x in module_glob(["test_*.py", "*/test_*.py"])]
    #print files
    test = env._IMPModuleTest("test.passed", files, **keys)
    env.AlwaysBuild("test.passed")
    module_alias(env, 'test', test)
    add_to_global_alias(env, 'test', 'test')
    module_requires(env, test, 'python')
    module_deps_requires(env, test, 'python', required_modules)
    module_deps_requires(env, test, 'python', env['IMP_REQUIRED_MODULES'])

def invalidate(env, fail_action):
    """'Break' an environment, so that any builds with it use the fail_action
       function (which should be an Action which terminates the build)"""
    for var in ('SHLINKCOM', 'CCCOM', 'CXXCOM', 'SHCCCOM', 'SHCXXCOM',
                'SWIGCOM'):
        env[var] = fail_action
    #env.Append(BUILDERS={'_IMPModuleTest': Builder(action=fail_action)})
    env['VALIDATED'] = False

def validate(env):
    """Confirm that a module's environment is OK for builds."""
    module = env['IMP_MODULE']
    env['VALIDATED'] = True

def IMPModuleBuild(env, version, required_modules=[],
                   optional_dependencies=[]):
    print "Configuring module " + env['IMP_MODULE'],

    if not env['IMP_MODULE_CPP']:
        print " (python only)",
    if required_modules is not None and len(required_modules) != 0:
        print "(requires " +", ".join(required_modules) + ")",
    print

    # Check required modules and add kernel
    if required_modules is not None:
        env.Prepend(LIBS=['imp'])
        env.Prepend(LIBS=['imp_'+x for x in required_modules])
        for x in required_modules:
            if x.startswith("imp_"):
                print "Required modules should have the name of the module (eg 'algebra'), not the name of the library."
                print required_modules
                raise ValueError(x)
            if x=='kernel':
                print "You do not need to list the kernel as a required module"
                print required_modules
                raise ValueError(x)
        required_modules.append('kernel')
        env['IMP_REQUIRED_MODULES']= required_modules
    else:
        env['IMP_REQUIRED_MODULES']= []
    for d in optional_dependencies:
        if d== "CGAL":
            if env['CGAL_LIBS']:
                env.Prepend(LIBS=env['CGAL_LIBS'])
        else:
            raise ValueError("Do not understand optional dependency: " +d)


    env['IMP_MODULE_VERSION'] = version
    vars=make_vars(env)
    env.validate()
    env.SConscript('doc/SConscript', exports='env')
    env.SConscript('examples/SConscript', exports='env')
    env.SConscript('data/SConscript', exports='env')

    if env['IMP_MODULE_CPP']:
        env.SConscript('include/SConscript', exports='env')
        env.SConscript('src/SConscript', exports='env')
        env.SConscript('bin/SConscript', exports='env')
    if env.get('python', True):
        env.SConscript('pyext/SConscript', exports='env')
        env.SConscript('test/SConscript', exports='env')

    add_to_global_alias(env, 'install', 'install')



def IMPModuleSetup(env, module, cpp=True, module_suffix=None,
                   module_include_path=None, module_src_path=None, module_preproc=None,
                   module_namespace=None, module_nicename=None):
    """Set up an IMP module. The module's SConscript gets its own
       customized environment ('env') in which the following pseudo-builders
       or methods are available: IMPPython, IMPModuleTest, validate
       and invalidate. If `cpp` is True, necessary C++ headers are also
       automatically generated, and these additional methods are available:
       IMPSharedLibraryEnvironment, IMPPythonExtensionEnvironment, IMPHeaders,
       IMPData.
       Either validate or invalidate must be called in the module's top-level
       SConscript before setting up any builders, to indicate whether the
       module's necessary dependencies have been met.
    """
    if module_suffix is None:
        module_suffix="_"+module
    if module_src_path is None:
        module_src_path="modules/"+module
    if module_include_path is None:
        module_include_path="IMP/"+module
    if module_preproc is None:
        module_preproc="IMP"+module.upper()
    if module_namespace is None:
        module_namespace="IMP::"+module
    if module_nicename is None:
        module_nicename= "IMP."+module
    #print module_suffix
    #print module_src_path
    #print module_include_path
    #print module_preproc
    #print module_namespace
    env['IMP_MODULES_ALL'].append(module)
    env = env.Clone()
    config = Builder(action=action_config)
    version_info = Builder(action=action_version_info)
    link_test = Builder(action=action_link_test)
    env.Append(BUILDERS = {'IMPModuleConfig': config,
                           'IMPModuleVersionInfo': version_info,
                           'IMPModuleLinkTest': link_test})

    env['IMP_MODULE'] = module
    env['IMP_MODULE_SUFFIX'] = module_suffix
    env['IMP_MODULE_INCLUDE_PATH'] = module_include_path
    env['IMP_MODULE_SRC_PATH'] = module_src_path
    env['IMP_MODULE_PREPROC'] = module_preproc
    env['IMP_MODULE_NAMESPACE'] = module_namespace
    env['IMP_MODULE_NICENAME'] = module_nicename
    env['IMP_MODULE_VERSION'] = "SVN"
    env['IMP_MODULE_AUTHOR'] = "A. Biologist"
    env['IMP_MODULE_CPP']= cpp
    env.Prepend(CPPPATH=['#/build/include'])
    env.Prepend(LIBPATH=['#/build/lib'])
    vars=make_vars(env)
    if cpp:
        build_config=[]
        # Generate version information
        build_config.append(
            env.IMPModuleVersionInfo(
                 ('%s/src/internal/version_info.cpp' % (module),
                  '%s/include/internal/version_info.h' % (module)),
                 ()))
        # Generate config header and SWIG equivalent
        build_config.append(env.IMPModuleConfig(target=['%s/include/config.h' % module,
                                                      '%s/pyext/%s_config.i' \
                                                          % (module, module)],
                                                source=[]))
        build_config.append(env.IMPModuleLinkTest(target=['#/%(module_src_path)s/src/internal/link_0.cpp'%vars],
                                                source=[]))
        build_config.append(env.IMPModuleLinkTest(target=['#/%(module_src_path)s/src/internal/link_1.cpp'%vars],
                                                source=[]))
        env.AddMethod(IMPModuleLib)
        env.AddMethod(IMPModuleInclude)
        module_alias(env, 'config', build_config)
    env.AddMethod(IMPModuleData)
    env.AddMethod(IMPModulePython)
    env.AddMethod(IMPModuleTest)
    env.AddMethod(IMPModuleBuild)
    env.AddMethod(IMPModuleGetHeaders)
    env.AddMethod(IMPModuleGetExamples)
    env.AddMethod(IMPModuleGetData)
    env.AddMethod(IMPModuleGetSources)
    env.AddMethod(IMPModuleGetBins)
    env.AddMethod(IMPModuleBin)
    env.AddMethod(IMPModuleDoc)
    env.AddMethod(IMPModuleExamples)
    env.AddMethod(IMPModuleGetDocs)
    env.Append(BUILDERS={'_IMPModuleTest': test.UnitTest})
    env.Append(BUILDERS={'_IMPColorizePython': examples.ColorizePython})
    env.Append(BUILDERS={'_IMPExamplesDox': examples.MakeDox})
    env.Append(BUILDERS={'_IMPSWIG': swig.SwigIt})
    env.Append(BUILDERS={'_IMPPatchSWIG': swig.PatchSwig})
    env.AddMethod(validate)
    env.AddMethod(invalidate)
    env['TEST_ENVSCRIPT'] = None
    env['VALIDATED'] = None
    env.SConscript('%s/SConscript' % module, exports='env')

def generate(env):
    """Add builders and construction variables for the IMP module tool."""
    env['IMP_MODULES_ALL'] = []
    env.AddMethod(IMPModuleSetup)

def exists(env):
    """Right now no external programs are needed"""
    return True
