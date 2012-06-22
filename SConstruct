# include IMP build utility functions:

import scons_tools
import scons_tools.dependency
import scons_tools.dependency.boost
import scons_tools.dependency.swig
import scons_tools.dependency.endian
import scons_tools.dependency.gcc
import scons_tools.dependency.pkgconfig
import scons_tools.application
import scons_tools.test
import scons_tools.config_py
import scons_tools.build_summary
import scons_tools.variables
import scons_tools.imppy
import scons_tools.html_coverage
import scons_tools.python_coverage
import sys
import atexit
from SCons import Script

# We need scons 0.98 or later
EnsureSConsVersion(0, 98)

# Set up build environment:
vars = Variables(files=[File('#/config.py').abspath])
scons_tools.variables.add_common_variables(vars, "imp")
env = scons_tools.environment.get_base_environment(variables=vars,
                              tools=["default", "swig", "dot", "doxygen", "cpp"],
                              toolpath=["scons_tools/tools"])
env['IMP_VERSION']=open(scons_tools.utility.get_source_path(env, "VERSION"), "r").read()
env['IMP_VARIABLES']=vars
env['IMP_CONFIGURATION']=[]

Export('env')

if env.get('repository', None) is not None:
    Repository(env['repository'])


if not env.GetOption('help'):
    if not env.GetOption('clean'):
        if not env.get('COMPILER_OK', None):
            Exit("""
No working compiler found. Please make sure that g++ or another
compiler recognized by scons can be found in your path and that all
the passed compiler options (cxxflags, linkflags) are correct.
""")

    scons_tools.dependency.pkgconfig.configure_check(env)
    scons_tools.dependency.add_external_library(env, "Boost", None,
                                                "boost/version.hpp", versionheader="boost/version.hpp",
                                                versioncpp=["BOOST_VERSION"])
    scons_tools.dependency.boost.find_lib_version(env)
    if not env.GetOption('clean'):
        if env.get('html_coverage', 'no') != 'no':
            scons_tools.html_coverage.register(env)

        if not scons_tools.data.get(env).dependencies['Boost'].ok or scons_tools.data.get(env).dependencies['Boost'].version < 103300:
            scons_tools.utility.report_error(env, """
Boost version is required to build IMP, but it could not be found on your system.

In particular, if you have Boost installed in a non-standard location, please use the 'includepath' option to add this location to the search path.  For example, a Mac using Boost installed with MacPorts will have the Boost headers in /opt/local/include, so edit (or create) config.py and add the line

includepath='/opt/local/include'

You can see the produced config.log for more information as to why boost failed to be found.
""")



if not env.GetOption('help'):
    # various flags depending on compiler versions and things
    scons_tools.dependency.swig.configure_check(env)
    scons_tools.dependency.endian.configure_check(env)
    scons_tools.dependency.gcc.configure_check_visibility(env)
    scons_tools.dependency.gcc.configure_check_hash(env)
    # Make these objects available to SConscript files:

# placed here so that the result is universally visible since it
# is special cased for benchmarks
scons_tools.dependency.add_external_library(env, "tcmalloc",
                                            ["tcmalloc"],
                                            # garbage to avoid rename issues
                                            "vector",
                                            enabled=False)


first=["modules", "applications", "biological_systems"]
last=["doc"]
reordered=[]
reordered_last=[]
all= [str(x) for x in Glob("*/SConscript")]
for f in first:
    e= f+"/SConscript"
    if e in all:
        reordered.append(e)
        all.remove(e)
for f in last:
    e= f+"/SConscript"
    if e in all:
        reordered_last.append(e)
        all.remove(e)
reordered.extend(all)
for f in reordered:
    SConscript("#/"+f)


if not env.GetOption('help'):
    # This must be after the other SConscipt calls so that it knows about all the generated files
    scons_tools.doc.add_overview_pages(env)
    imppy= scons_tools.imppy.add(env, "tools/imppy.sh")
    sitecust = scons_tools.python_coverage.setup(env)
    env.Depends(imppy, sitecust)
    env.Alias(env.Alias('all'), imppy)

    env.Alias(env.Alias('test'), [env.Alias('examples-test')])

    Clean('all', ['build'])
    Clean('all', Glob('scons_tools/*.pyc')\
              + Glob('tools/*.pyc'))

    env.Default(env.Alias('all'))


    unknown = vars.UnknownVariables()
    # Older versions of scons have a bug with command line arguments
    # that are added late, so remove those we know about from this list
    for dep, data in scons_tools.data.get(env).dependencies.items():
        for var in data.variables:
            unknown.pop(var, None)
    if unknown:
        print >> sys.stderr, "\n\nUnknown variables: ", " ".join(unknown.keys())
        print >> sys.stderr, "Use 'scons -h' to get a list of the accepted variables."
        Exit(1)
    for f in reordered_last:
        SConscript("#/"+f)
    scons_tools.build_summary.setup(env)
    config_py=scons_tools.config_py.add(env)
    senv= scons_tools.environment.get_named_environment(env, "scons", [], [])
    scons_tools.install.install(senv, "datadir/scons", "SConstruct")
    scons_tools.install.install_hierarchy(senv, "datadir/scons/scons_tools",
                                          "scons_tools",
                                          Glob("scons_tools/*.py")+
                                          Glob("scons_tools/*/*.py"))

else:
    tenv= Environment(variables=vars)
    Help("""
    Available command-line options:
    (These can also be specified in regular Python syntax by creating a file
    called 'config.py' in this directory.)
    """)
    Help(vars.GenerateHelpText(tenv))

    Help("""
    Type: 'scons' to build the IMP kernel and all configured modules (i.e. those
                  with no unmet dependencies);
          'scons test' to run unit tests for the kernel and all configured modules;
          'scons install' to install the kernel and all configured modules;
          'scons doc{-install}' to build (and optionally install) doc

    Other useful targets:
          '[kernel,modulename]-test' to test all modules, the kernel, or a particular module
          '[kernel, modulename]-test-examples' to test the examples for a particular module or the kernel
          'all' to build and test everything (and clean up everything in conjunction with '-c')

    Infrequently changing settings can be stored in a 'config.py' file in the build directory. An example is provided in tools/example-config.py.
    """)
