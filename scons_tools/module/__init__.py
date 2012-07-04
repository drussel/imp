"""Tools and Builders for IMP modules. See `IMPModule` for more information."""

import os.path
import sys
import scons_tools.pyscanner
import _swig
import _header
import _version_info
import _link_test
import _standards
import _version_h
import _config_h
import _all_cpp
import scons_tools.bug_fixes
import scons_tools.run
import scons_tools.dependency
import scons_tools.doc
import scons_tools.environment
import scons_tools.examples
import scons_tools.install
import scons_tools.utility
import scons_tools.paths as stp
import scons_tools.bins as stb

from SCons.Script import Builder, File, Action, Glob, Return, Dir, Move, Copy, Scanner
from SCons.Scanner import C as CScanner

def _get_module_name(env):
    return scons_tools.environment.get_current_name(env)


def _get_module_full_name(env):
    name= _get_module_name(env)
    if name=="kernel":
        name="IMP"
    else:
        name="IMP."+name
    return name


def _get_module_path(env):
    module= _get_module_name(env)
    name=module
    if name=="kernel":
        name=""
    elif name=="RMF":
        name="librmf"
    return name

def _get_module_data(env, module=None):
    if not module:
        module= _get_module_name(env)
    return scons_tools.data.get(env).modules[module]

def _get_module_has_data(env):
    #print _get_module_data(env).data
    return _get_module_data(env).data

def _get_module_alias(env):
    return _get_module_data(env).alias

def _get_found_modules(env, modules):
    ret=[]
    for m in modules:
        if _get_module_data(env, m).ok:
            ret.append(m)
    return ret

def _get_module_modules(env):
    return _get_module_data(env).modules

def _get_module_unfound_modules(env):
    return _get_module_data(env).unfound_modules


def _get_module_version(env):
    return _get_module_data(env).version


def _get_module_unfound_dependencies(env):
    return _get_module_data(env).unfound_dependencies

def _get_module_dependencies(env):
    return _get_module_data(env).dependencies

def _get_module_direct_dependencies(env):
    return _get_module_data(env).direct_dependencies


def _set_module_links(env, links):
    env['IMP_MDLE_LINKS']=links
def _get_module_links(env):
    return env['IMP_MDLE_LINKS']


def _get_module_variables(env):
    """Make a map which can be used for all string substitutions"""
    return env['IMP_MODULE_VARS']


def IMPModuleLib(envi, files):
    """Build, and optionally also install, an IMP module's C++
       shared library. This is only available from within an environment
       created by `IMPSharedLibraryEnvironment`."""
    if envi["IMP_PASS"] != "BUILD":
        return
    vars= _get_module_variables(envi)
    module = _get_module_name(envi)
    module_libname =_get_module_variables(envi)['module_libname']
    version= _get_module_version(envi)
    data= scons_tools.data.get(envi).modules[_get_module_name(envi)]
    prefix=vars['module_libname']
    if prefix=="imp":
        prefix="imp_kernel"
    config= envi.IMPModuleConfigCPP(target=[stp.get_build_source_file(envi,"config.cpp", module)],
                                    source=[envi.Value(version),
                                            envi.Value(envi.subst(envi['datadir'])),
                                            envi.Value(envi.subst(os.path.join(envi['docdir'], "examples")))])
    #env.AlwaysBuild(version)
    build=[]
    if envi['percppcompilation']=="yes"\
           or module in envi['percppcompilation'].split(":"):
        allf=files+config
        if envi['build']=="debug" and envi['linktest']:
            link0=envi.IMPModuleLinkTest(target=[stp.get_build_source_file(envi,
                                                                           'link_0.cpp',
                                                                           module)],
                                          source=[])
            link1=envi.IMPModuleLinkTest(target=[stp.get_build_source_file(envi,
                                                                           'link_1.cpp',
                                                                            module)],
                                          source=[])
            allf= allf+link0+link1
    else:
        allf= [_all_cpp.get(envi, list(files))]+config
        if envi['build']=="debug" and envi['linktest']:
            link1=envi.IMPModuleLinkTest(target=[stp.get_build_source_file(envi,
                                                                           'link.cpp',
                                                                           module)],
                                          source=[])
            allf= allf+link1
    if envi['IMP_BUILD_STATIC']:
        env= scons_tools.environment.get_staticlib_environment(envi)
        sl= env.StaticLibrary('#/build/lib/%s' % module_libname,
                              allf)
        scons_tools.data.get(env).add_to_alias(_get_module_alias(env), sl[0])
    if envi['IMP_BUILD_DYNAMIC']:
        env = scons_tools.environment.get_sharedlib_environment(envi, '%(EXPORT)s_EXPORTS' % vars,
                                    cplusplus=True)
        sl=env.SharedLibrary('#/build/lib/%s' % module_libname,
                                       allf )
        scons_tools.data.get(env).add_to_alias(_get_module_alias(env), sl[0])
        scons_tools.utility.postprocess_lib(env, sl)


def IMPModuleInclude(env, files):
    """Install the given header files, plus any auto-generated files for this
       IMP module."""
    if env["IMP_PASS"] != "BUILD":
        return
    vars=_get_module_variables(env)
    module= _get_module_name(env)
    moduleinclude= vars['module_include_path']
    # Generate config header and SWIG equivalent
    version=env.IMPModuleVersionH(target\
                                      =[File("#/build/include/"+moduleinclude\
                                                 +"/"+module+"_version.h")],
                               source=[env.Value(_get_module_version(env))])
    data= scons_tools.data.get(env)
    deps= _get_module_dependencies(env)
    signature=_get_module_unfound_dependencies(env)\
        + deps\
        + [data.dependencies[x].version for x in deps]\
        + _get_found_modules(env, _get_module_modules(env))\
        + [_get_module_has_data(env)]
    config=env.IMPModuleConfigH(target\
                                    =[File("#/build/include/"+moduleinclude\
                                               +"/"+module+"_config.h")],
                               source=[env.Value(env['IMP_MODULE_CONFIG']),
                                       env.Value(signature)])
    scons_tools.install.install_hierarchy_in_build(env,files,
                                                   "#/build/include/"\
                                                       + vars['module_include_path'])
    header=_header.build_header(env,
                                "#/build/include/"+vars['module_include_path']+".h",
                                list(files))

def IMPModuleData(env, files):
    """Install the given data files for this IMP module."""
    if env["IMP_PASS"] != "BUILD":
        return
    dta=scons_tools.data.get(env).modules[_get_module_name(env)]
    module=_get_module_name(env)
    dta.data=True
    build =scons_tools.install.install_hierarchy_in_build(env, files,
                                                          "#/build/data/"+module)
    scons_tools.data.get(env).add_to_alias(_get_module_alias(env), build)


def IMPModuleExamples(env, example_files, data_files):
    example_files= [File(x) for x in example_files]
    if env["IMP_PASS"] == "BUILD":
        seen=[]
        for e in example_files:
            if str(e).endswith(".py"):
                for o in example_files:
                    if str(o) == scons_tools.utility.get_without_extension(str(e))+".readme":
                        seen.append(o)
                        break
                    scons_tools.examples.add_python_example(env, e, o.get_contents())
        links=[]
        fexample_files=[x for x in example_files if x not in seen]
        split= scons_tools.utility.get_split_into_directories(fexample_files)
        for k in split.keys():
            if len(k)>0:
                name =_get_module_name(env)+ " examples: "+k
                pre=k+'/'
            else:
                name =_get_module_name(env)+ " examples"
                pre=""
            name=scons_tools.utility.get_display_from_name(name)
            l= scons_tools.examples.add_page(env, name,
                                             [pre+x for x in split[k]])
            links.append(l)
        _set_module_links(env, links)

    if env["IMP_PASS"]=="RUN":
        module= _get_module_name(env)
        build=scons_tools.install.install_hierarchy_in_build(env,
                                                             example_files+data_files,
                                                             "#/build/doc/examples/"+module)
        scons_tools.data.get(env).add_to_alias(_get_module_alias(env), build)
        test_files = stp.get_matching_source(env, ['test_examples.py'])
        runable=[x for x in example_files + test_files
                 if str(x).endswith(".py") \
                     and str(x).find("fragment")==-1]
        if len(runable)>0:
            tests = scons_tools.test.add_tests(env,
                                               source=runable,
                                               type='example')
            for t in tests:
                env.Depends(t, build)


def IMPModuleBin(env, files):
    if env["IMP_PASS"] != "BUILD":
        return
    prgs=stb.handle_bins(env, files,
                         stp.get_build_bin_dir(env, _get_module_name(env)),
                         extra_modules=[_get_module_name(env)])
    scons_tools.data.get(env).add_to_alias(_get_module_alias(env), prgs)

def IMPModuleBenchmark(env, files):
    if env["IMP_PASS"] != "BUILD":
        return
    prgs, bmarks=stb.handle_benchmarks(env, files,
                                       stp.get_build_benchmark_dir(env, _get_module_name(env)),
                                       extra_modules=[_get_module_name(env)])
    scons_tools.data.get(env).add_to_alias(_get_module_alias(env)+"-benchmarks", bmarks)
    scons_tools.data.get(env).add_to_alias(_get_module_alias(env), prgs)

def IMPModulePython(env, swigfiles=[], pythonfiles=[]):
    """Build and install an IMP module's Python extension and the associated
       wrapper file from a SWIG interface file. This is only available from
       within an environment created by `IMPPythonExtensionEnvironment`."""
    if env["IMP_PASS"] != "BUILD":
        return
    module =_get_module_name(env)
    vars=_get_module_variables(env)
    data=scons_tools.data.get(env)
    moduledata=data.modules[_get_module_name(env)]
    alldata= scons_tools.data.get(env).modules
    penv = scons_tools.environment.get_pyext_environment(env, module.upper(),
                                                         cplusplus=True,
                                                         extra_modules=[module])
    #penv.Decider('timestamp-match')
    versions=[]
    vc= _swig.VersionCheck(penv,
                           target=[File("#/build/lib/"\
                                            +vars['module_include_path']\
                                            +"/_version_check.py")],
                           source=[env.Value(_get_module_version(env))]+versions)
    data.add_to_alias(_get_module_alias(env),vc[0])
    prefix=vars['module_pylibname'][1:]
    if prefix=="IMP":
        prefix="IMP_kernel"
    swigfile= \
       penv.IMPModuleSWIGPreface(target=[File("#/build/swig/"+prefix+".i")],
                                 source=[File("swig.i-in"),
                                         env.Value(_get_module_modules(env)),
                                         env.Value(" ".join(_get_module_dependencies(env))),
                                  env.Value(" ".join(_get_module_unfound_dependencies(env))),
                                         env.Value(_get_module_has_data(env))])
    for i in swigfiles:
        if str(i).endswith('.i'):
            scons_tools.install.install_in_build(env,i, "#/build/swig/"+str(i))
    produced=File("#/build/lib/"+vars['module_include_path']+"/__init__.py")
    version=_get_module_version(penv)
    cppin=stp.get_build_source_file(penv,
                                    "wrap.cpp-in", module)
    hin=stp.get_build_source_file(penv,
                                  "wrap.h-in", module)
    swigr=penv.IMPModuleSWIG(target=[produced,
                                     cppin, hin],
                             source=[swigfile])
    #print "Moving", produced.path, "to", dest.path
    cppf=stp.get_build_source_file(penv,
                                   "wrap.cpp", module)
    hf=stp.get_build_source_file(penv,
                                 "wrap.h", module)
    patched=penv.IMPModulePatchSWIG(target=[cppf],
                                    source=[cppin])
    hpatched=penv.IMPModulePatchSWIG(target=[hf],
                                     source=[hin])
    penv.Requires(patched, hpatched)
    lpenv= scons_tools.bug_fixes.clone_env(penv)
    buildlib = lpenv.LoadableModule("#/build/lib/"+vars["module_pylibname"],
                                    patched) #SCANNERS=scanners
    data.add_to_alias(_get_module_alias(env), buildlib[0])
    if "kernel" in _get_module_dependencies(env):
        # all python support needs kernel, silly design to put it in the base
        # namespace/python module
        env.Requires(data.get_alias(module),
                          data.get_alias("kernel"))
    scons_tools.utility.postprocess_lib(penv, buildlib)
    b= scons_tools.install.install_hierarchy_in_build(env,
                                                      pythonfiles,
                                                          "#/build/lib/"+vars['module_include_path'], prefix="src")
    for bc in b:
        data.add_to_alias(_get_module_alias(env), bc)

def IMPModuleGetExamples(env):
    rms= stp.get_matching_source(env, ["*.readme", "*.py"])
    # evil, this should put put somewhere in build
    return [x for x in rms if not x.path.endswith("test_examples.py")]

def IMPModuleGetExampleData(env):
    ret=  stp.get_matching_source(env, ["*.pdb", "*.mrc", "*.dat",
                                        "*.xml", "*.em", "*.rmf",
                                        "*.hdf5", "*.mol2", "*data/*"])
    return ret

def IMPModuleGetPythonTests(env):
    return stp.get_matching_source(env, ["test_*.py", "*/test_*.py"])
def IMPModuleGetCPPTests(env):
    return stp.get_matching_source(env, ["test_*.cpp", "*/test_*.cpp"])
def IMPModuleGetExpensivePythonTests(env):
    return stp.get_matching_source(env, ["expensive_test_*.py",
                                         "*/expensive_test_*.py"])
def IMPModuleGetExpensiveCPPTests(env):
    return stp.get_matching_source(env, ["expensive_test_*.cpp",
                                         "*/expensive_test_*.cpp"])


def IMPModuleGetHeaders(env):
    files=stp.get_matching_source(env, ["*.h", "internal/*.h"])
    return files

def IMPModuleGetSwigFiles(env):
    vars=_get_module_variables(env)
    prefix=vars['module_pylibname'][1:]
    if prefix=="IMP":
        prefix="IMP_kernel"
    files=stp.get_matching_source(env, [prefix+".*.i"])
    return files

def IMPModuleGetPython(env):
    files=scons_tools.utility.get_matching(["src/*.py", "src/*/*.py",
                                            "src/*/*/*.py", "src/*/*/*/*.py",
                                            "src/*/*/*/*/*.py"])
    return files

def IMPModuleGetSources(env):
    files=stp.get_matching_source(env,["*.cpp", "internal/*.cpp"])
    return files

def IMPModuleGetData(env):
    raw_files=scons_tools.utility.get_matching_recursive(["*"])
    files=[]
    for i,f in enumerate([os.path.split(str(x))[1] for x in raw_files]):
        if str(f).endswith("SConscript"):
            continue
        if str(f).endswith(".old"):
            continue
        if str(f).startswith("."):
            continue
        if str(f).endswith("~"):
            continue
        files.append(raw_files[i])
    return files

def IMPModuleGetBins(env):
    return stp.get_matching_source(env, ["*.cpp", "*.py"])

def IMPModuleGetBenchmarks(env):
    return stp.get_matching_source(env, ["*.cpp", "*.py"])

def IMPModuleGetDocs(env):
    files=stp.get_matching_source(env, ["*.dox", "*.pdf", "*.dot", "*.png"])
    return files


def IMPModuleDoc(env, files, authors,
                 brief, overview,
                 publications=None,
                 license="standard"):
    if env["IMP_PASS"] != "BUILD":
        return
    docdir=env['docdir']+"/"+_get_module_variables(env)['module_include_path']
    links= _get_module_links(env)
    if overview.find('\r') != -1:
        raise RuntimeError("\\r is not allowed in overview fields as it generally should be \\\\r")
    if len(links) > 0:
        overview+= '\n\nExamples:\n'
        for l in links:
            overview+=' - ' +l +'\n'
        # defined in doc so all examples have been seen
        overview += ' - \\ref '+_get_module_name(env)\
                    +'_all_example_index "All examples using IMP.'+_get_module_name(env)+'"\n'
    scons_tools.doc.add_doc_page(env,
                                 "\\namespace "\
                                 +_get_module_variables(env)['namespace']\
                                 +'\n\\brief '+brief,
                                 authors,_get_module_version(env),
                                 brief, overview, publications, license)
    scons_tools.doc.add_doc_files(env, files)


#   files= ["#/bin/imppy.sh", "#/tools/run_all_tests.py"]+\
#        [x.abspath for x in Glob("test_*.py")+ Glob("*/test_*.py")]

def IMPModuleTest(env, python_tests=[], cpp_tests=[],
                  expensive_python_tests=[],
                  expensive_cpp_tests=[],
                  plural_exceptions=[], show_exceptions=[],
                  function_name_exceptions=[],
                  value_object_exceptions=[],
                  class_name_exceptions=[],
                  spelling_exceptions=[],
                  check_standards=True):
    """Pseudo-builder to run tests for an IMP module. The single target is
       generally a simple output file, e.g. 'test.passed', while the single
       source is a Python script to run (usually run-all-tests.py).
       Right now, the assumption is made that run-abll-tests.py executes
       all files called test_*.py in the current directory and subdirectories."""
    if env["IMP_PASS"] != "RUN":
        return
    files= [x.abspath for x in python_tests]
    expensive_files= [x.abspath for x in expensive_python_tests]
    module=_get_module_name(env)
    if len(cpp_tests)>0:
        #print "found cpp tests", " ".join([str(x) for x in cpp_tests])
        prgs= stb.handle_bins(env, cpp_tests,
                              stp.get_build_test_dir(env, module),
                              extra_modules=[module])
        #print [x[0].abspath for x in prgs]
        cpptest= env.IMPModuleCPPTest(target=File("#/build/test/%s_cpp_test_programs.py"%module),
                                       source= prgs)
        files.append(cpptest)
    if len(expensive_cpp_tests)>0:
        #print "found cpp tests", " ".join([str(x) for x in cpp_tests])
        prgs= _make_programs(env, Dir("#/build/test"), expensive_cpp_tests, prefix=module)
        #print [x[0].abspath for x in prgs]
        cpptest= env.IMPModuleCPPTest(target="%s_expensive_cpp_test_programs.py"%module,
                                       source= prgs)
        expensive_files.append(cpptest)
    if check_standards:
        standards=_standards.add(env, plural_exceptions=plural_exceptions,
                                 show_exceptions=show_exceptions,
                                 function_name_exceptions=function_name_exceptions,
                                 value_object_exceptions=value_object_exceptions,
                                 class_name_exceptions=class_name_exceptions,
                                 spelling_exceptions=spelling_exceptions)
        #found=False
        #for f in files:
        #    if str(f).endswith("test_standards.py"):
        #        found=f
        #if found:
        #    files.remove(found)
        files.append(standards)
    tests = scons_tools.test.add_tests(env, source=files,
                                       expensive_source=expensive_files,
                                       type='module unit test')

def IMPModuleBuild(env, version=None, required_modules=[],
                   lib_only_required_modules=[],
                   optional_modules=[],
                   lib_only_optional_modules=[],
                   optional_dependencies=[], config_macros=[],
                   module=None, module_libname=None,
                   module_pylibname=None,
                   module_include_path=None, module_preproc=None,
                   module_namespace=None, module_nicename=None,
                   required_dependencies=[],
                   alias_name=None,
                   cxxflags=[], cppdefines=[], cpppath=[], python_docs=False,
                   local_module=False,
                   standards=True):
    if env.GetOption('help'):
        return
    dta= scons_tools.data.get(env)
    if module is None:
        module=Dir('.').abspath.split('/')[-1]
        if module=="local":
            module=Dir('.').abspath.split('/')[-2]+"_local"
    if not module_libname and (module != module.lower() or module.find("-") != -1):
        scons_tools.utility.report_error("Module names can only have lower case characters and numbers")
    if module_libname is None:
        module_libname="imp_"+module
    if module_pylibname is None:
        module_pylibname="_IMP_"+module
    if module_include_path is None:
        module_include_path="IMP/"+module
    if module_namespace is None:
        module_namespace="IMP::"+module
    if module_preproc is None:
        module_preproc=module_namespace.replace("::","_").upper()
    if module_nicename is None:
        module_nicename= "IMP."+module
    if alias_name is None:
        alias_name=module
    if python_docs:
        env.Append(IMP_PYTHON_DOCS=[module])
    optm=optional_modules+lib_only_optional_modules
    optd=optional_dependencies
    reqd=required_dependencies
    reqm=required_modules+lib_only_required_modules
    all_sconscripts=stp.get_sconscripts(env, ['data', 'examples'])
    nenv = scons_tools.utility.configure_module(env,
                                                module, alias_name,
                                                module_libname,
                                                version,
                                                required_modules=reqm,
                                                optional_dependencies=optd,
                                                optional_modules=optm,
                                                required_dependencies= reqd)
    if not nenv:
        return
    preclone=env

    env = nenv
    vars={'module_include_path':module_include_path,
          'module':module,
          'PREPROC':module_preproc,
          'EXPORT':module_preproc.replace("_", ""),
          'namespace':module_namespace,
          'module_libname':module_libname,
          'module_pylibname':module_pylibname,
          'module_nicename':module_nicename,
          'module_alias':alias_name}
    env['IMP_MODULE_VARS']=vars


    build_config=[]
    if cxxflags:
        env.Replace(CXXFLAGS=cxxflags)
    if cppdefines:
        env.Append(CPPDEFINES=cppdefines)
    if cpppath:
        env.Append(CPPPATH=cpppath)
    #if len(found_optional_modules + found_optional_dependencies)>0:
    #    print "  (using " +", ".join(found_optional_modules + found_optional_dependencies) +")"
    real_config_macros=config_macros[:]

    #print "config", module, real_config_macros
    env['IMP_MODULE_CONFIG']=real_config_macros
    for s in all_sconscripts:
        env.SConscript(s, exports='env')

    if env['IMP_PASS']=="BUILD":
        dta.add_to_alias("all", _get_module_alias(env))
        # needed for data
        for m in _get_module_modules(env):
            env.Requires(dta.get_alias(_get_module_alias(env)),
                         dta.get_alias(dta.modules[m].alias))

        if standards:
            root=Dir(".").abspath
            if env.get('repository', None):
                old=Dir("#").abspath
            #print old, root, env['repository']
                root=root.replace(old, Dir(Dir("#").abspath+"/"+env['repository']).abspath)
                scons_tools.standards.add(env, [root+"/"+x for x in ["include/*.h",
                                                                     "include/internal/*.h",
                                                                     "src/*.cpp",
                                                                     "src/internal/*.cpp",
                                                                     "test/*.py",
                                                                     "bin/*.cpp"]])
    return env
