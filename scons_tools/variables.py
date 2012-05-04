"""Utility functions used by all IMP modules"""

import os.path
import re
import sys
from SCons.Script import *
import dependency.compilation
import module
import dependency
import platform
import utility

import SCons


def _propagate_variables(env):
    """enforce dependencies between variables"""
    env['builddir']="#/build"
    env['IMP_BUILD_STATIC']= env['static']
    env['IMP_BUILD_DYNAMIC']= env['dynamic']
    if env['python'] != "no" and not env.get('PYTHON', None):
        env['IMP_PROVIDE_PYTHON']= True
        if env['python'] =="auto":
            env['PYTHON']=env['python']
        else:
            env['PYTHON']=env['python']
    else:
        env['IMP_PROVIDE_PYTHON']=False
        if env['wine']:
            env['PYTHON']="w32python"
        else:
            env['PYTHON']="python"
    env['IMP_PROVIDE_PYTHON']= env['python']
    env['IMP_USE_PLATFORM_FLAGS']= env['platformflags']
    env['IMP_USE_RPATH']= env['rpath']
    if env['pythonsosuffix'] != 'default':
        env['IMP_PYTHON_SO']=env['pythonsosuffix']
    elif env['IMP_PROVIDE_PYTHON'] and not env['IMP_USE_PLATFORM_FLAGS']:
        print >> sys.stderr, "Do not know suffix for python lib, please provide pythonsosuffix"
        env.Exit(1)
    if env['wine']:
        env['IMP_BUILD_STATIC']=False
        env['IMP_PYTHON_SO']='.pyd'

    if env['PLATFORM']!= 'posix' and env['PLATFORM'] != 'darwin' and env['IMP_USE_RPATH']:
        env['IMP_USE_RPATH']=False
        print >> sys.stderr, "WARNING rpath not supported on platform "+ env['PLATFORM']

    if not env['IMP_BUILD_DYNAMIC']:
        env['IMP_PROVIDE_PYTHON']=False
    if not env['IMP_BUILD_DYNAMIC'] and not env['IMP_BUILD_STATIC']:
        print >> sys.stderr, "One of dynamic or static libraries must be supported."
        env.Exit(1)
    if env.get('pythonpath', None):
        env['PYTHONPATH'] = env['pythonpath']
    else:
        env['PYTHONPATH']=''
    if env.get('cxxcompiler', None):
        env['CXX']=env['cxxcompiler']
    if env.get('ar', None):
        env['AR']= env['ar']
    if env.get('ranlib', None):
        env['RANLIB']= env['ranlib']
    if env.get("swigprogram", None):
        env['SWIG']= env["swigprogram"]
    if env.get('cxxflags', None):
        env.Append(CXXFLAGS = env['cxxflags'].split())
    else:
        env.Append(CXXFLAGS=[])

    for t in ['includepath', 'libpath', 'datapath','pythonpath', 'swigpath', 'ldlibpath']:
        r=utility.get_abspaths(env, t, env.get(t, ""))
        env[t]=os.path.pathsep.join(r)

    if env.get('pythoncxxflags', None):
        env.Append(IMP_PYTHON_CXXFLAGS = env['pythoncxxflags'].split())
    elif env.get('cxxflags', None):
        env.Append(IMP_PYTHON_CXXFLAGS = env['cxxflags'].split())
    else:
        env.Append(IMP_PYTHON_CXXFLAGS=[])
    if env.get('bincxxflags', None):
        env.Append(IMP_BIN_CXXFLAGS = env['bincxxflags'].split())
    elif env.get('cxxflags', None):
        env.Append(IMP_BIN_CXXFLAGS = env['cxxflags'].split())
    else:
        env.Append(IMP_BIN_CXXFLAGS=[])

    if env.get('linkflags', None):
        env.Append(IMP_LINKFLAGS=env['linkflags'].split())
    else:
        env.Append(IMP_LINKFLAGS=[])
    if env.get('pythonlinkflags', None):
        env.Append(IMP_PYTHON_LINKFLAGS=env['pythonlinkflags'].split())
    else:
        env.Append(IMP_PYTHON_LINKFLAGS=[])

    if env.get('shliblinkflags', None):
        env.Append(IMP_SHLIB_LINKFLAGS=env['shliblinkflags'].split())
    else:
        env.Append(IMP_SHLIB_LINKFLAGS=[])

    if env.get('arliblinkflags', None):
        env.Append(IMP_ARLIB_LINKFLAGS=env['arliblinkflags'].split())
    else:
        env.Append(IMP_ARLIB_LINKFLAGS=[])


    if env.get('binlinkflags', None):
        env.Append(IMP_BIN_LINKFLAGS=env['binlinkflags'].split())
    else:
        env.Append(IMP_BIN_LINKFLAGS=[])

    if env.get('includepath') is not None:
        env.Prepend(CPPPATH=utility.get_env_paths(env, 'includepath'))
    else:
        env.Append(CPPPATH=[])

    if env.get('libpath') is not None:
        env.Prepend(LIBPATH=utility.get_env_paths(env, 'libpath'))
    else:
        env.Append(LIBPATH=[])
    if env.get('libs') is not None:
        env.Append(LIBS=utility.get_env_paths(env, 'libs'))
    else:
        env.Append(LIBS=[])

    if env.get('ldlibpath') is not None and env.get('ldlibpath') != '':
        env['ENV']['LD_LIBRARY_PATH'] = env['ldlibpath']

    if env.get('environment') is not None:
        for pair in env.get('environment').split(','):
            if pair != "":
                if pair.find("=") != -1:
                    (name, value)= pair.split("=")
                    env['ENV'][name]=value
                else:
                    env['ENV'][pair]=""


def add_common_variables(vars, package):
    """Add common variables to an SCons Variables object."""
    libenum=["yes", "no", "auto"]
    libdir = '${prefix}/lib'
    if hasattr(os, 'uname') and sys.platform == 'linux2' \
       and os.uname()[-1] == 'x86_64':
        # Install in /usr/lib64 rather than /usr/lib on x86_64 Linux boxes
        libdir += '64'
    vars.Add(PathVariable('cxxcompiler', 'The C++ compiler to use (eg g++).', None,
                          PathVariable.PathAccept))
    vars.Add(PathVariable('ar', "The command to make a static library.", None,
                          PathVariable.PathAccept))
    vars.Add(PathVariable('ranlib', "The command to make an index of a static library.", None,
                          PathVariable.PathAccept))
    vars.Add(PathVariable('swigprogram', 'The path to the swig command.', None,
                          PathVariable.PathAccept))
    vars.Add(PathVariable('prefix', 'Top-level installation directory', '/usr',
                          PathVariable.PathAccept))
    vars.Add(PathVariable('datadir', 'Data file installation directory',
                          '${prefix}/share/%s'%package,
                          PathVariable.PathAccept))
    vars.Add(PathVariable('bindir', 'Executable installation directory',
                          '${prefix}/bin', PathVariable.PathAccept))
    vars.Add(PathVariable('libdir', 'Shared library installation directory',
                          libdir, PathVariable.PathAccept))
    vars.Add(PathVariable('includedir', 'Include file installation directory',
                          '${prefix}/include', PathVariable.PathAccept))
    vars.Add(PathVariable('pythondir', 'Python module installation directory',
                          '${libdir}/python%d.%d/site-packages' \
                          % sys.version_info[0:2], PathVariable.PathAccept))
    vars.Add(PathVariable('pyextdir',
                          'Python extension module installation directory',
                          '${pythondir}', PathVariable.PathAccept))
    vars.Add(PathVariable('docdir', 'Documentation installation directory',
                          '${prefix}/share/doc/%s' % package,
                          PathVariable.PathAccept))
    # Note that destdir should not affect any compiled-in paths; see
    # http://www.gnu.org/prep/standards/html_node/DESTDIR.html
    vars.Add(PathVariable('destdir',
                          'String to prepend to every installed filename',
                          '', PathVariable.PathAccept))
    vars.Add(PackageVariable('pythoninclude',
                             'Directory holding Python include files ' + \
                             '(if unspecified, distutils location is used)',
                             'no'))
    vars.Add(PackageVariable('modeller',
                             "Set to 'yes' to use the MODELLER package, "
                             "or 'no' to not use it (and to disable modules "
                             "such as IMP.modeller that use it). 'yes' will "
                             "only find MODELLER if it is in the system Python "
                             "path (e.g. Windows, Mac .dmg or Linux .rpm "
                             "binary installs); if you installed the .tar.gz "
                             "version, or have a copy of the source code, set "
                             "this variable to the top-level MODELLER "
                             "directory.", 'no'))
    vars.Add(BoolVariable('wine',
                          'Build using MS Windows tools via Wine emulation',
                          False))
    #vars.Add(BoolVariable('timetests',
    #                      'Print running time for each test',
    #                      False))
    vars.Add(BoolVariable('versionchecks',
                          "By default IMP checks the versions of its dependencies a"
                          "runtime. This is useful to ensure that there are no link "
                          "problems and to avoid very difficult to track down errors. "
                          "In some scenarios, these cannot be implemented properly and "
                          "need to be disabled. If you don't have a good reason, leave "
                          "this on.", True))
    vars.Add(EnumVariable('build',
                          "Set to 'release' for a normal build," \
                          +" 'debug' to disable optimization," \
                          +" or 'fast' to disable most runtime checks," \
                          +" or 'compile' to compile as quickly as possible,"\
                          +" but keep debugging information",
                          "release", ['release', 'debug', 'fast', 'compile']))
    vars.Add(EnumVariable('endian',
                          "The endianness of the platform. \"auto\" will determine it automatically.",
                          "auto", ['auto', 'big', 'little']))
    vars.Add(BoolVariable('linksysv',
                          'Link with old-style SysV, not GNU hash, for ' + \
                          'binary compatibility', False))
    vars.Add('includepath', 'Include search path ' + \
             '(e.g. "/usr/local/include:/opt/local/include")', None)
    vars.Add('swigpath', 'Swig search path ' + \
             '(e.g. "/usr/local/share/swig")', None)
    vars.Add('libpath', 'Library search path ' + \
             '(e.g. "/usr/local/lib:/opt/local/lib")', None)
    vars.Add('libs', 'Extra libs to add to link commands ' + \
             '(e.g. "efence:pthread")', None)
    vars.Add(BoolVariable('rpath',
                          'Add any entries from libpath to library search ' + \
                          'path (rpath) on Linux systems', True))
    vars.Add('ldlibpath', 'Add to the runtime library search path ' +\
             '(LD_LIBRARY_PATH on linux-like systems) for various ' + \
             'build tools and the test cases', None)
    vars.Add('cxxflags', 'C++ flags for all C++ builds (e.g. "-fno-rounding:-DFOOBAR"). See pythoncxxflags.',
             None)
    vars.Add('pythoncxxflags', 'C++ flags for building the python libraries (e.g. "-fno-rounding:-DFOOBAR")',
             None)
    vars.Add('bincxxflags', 'C++ flags for building executables libraries (e.g. "-fno-rounding:-DFOOBAR")',
             None)
    vars.Add(EnumVariable('boost_autolink',
                          'Whether to use Boost autolinking to find Boost '
                          'dynamic or static libraries on supported platforms',
                          'disable', ['disable', 'static', 'dynamic'],
                          ignorecase=1))

    vars.Add('linkflags', 'Link flags for all linking (e.g. "-lefence"). See pythonlinkflags, arliblinkflags, shliblinkflags.', None)
    vars.Add('environment', "Add entries to the environment in which tools are run. The variable should be a comma separated list of name=value pairs.", "")
    vars.Add('pythonlinkflags', 'Link flags for linking python libraries (e.g. "-lefence")', "")
    vars.Add('arliblinkflags', 'Link flags for linking static libraries (e.g. "-lefence")', "")
    vars.Add('shliblinkflags', 'Link flags for linking shared libraries (e.g. "-lefence")', "")
    vars.Add('binlinkflags', 'Link flags for linking executables (e.g. "-lefence")', "")
    vars.Add('pkgconfig', 'Whether to use pkg_config ', "auto")
    vars.Add('path', 'Extra executable path ' + \
             '(e.g. "/opt/local/bin/") to search for build tools', None)
    vars.Add('precommand',
             'A command to be run to wrap program invocations.' + \
             'For example, "valgrind --db-attach=yes --suppressions=valgrind-python.supp"', "")
    vars.Add('pythonpath', 'Extra python path ' + \
             '(e.g. "/opt/local/lib/python-2.5/") to use for tests', None)
    vars.Add('boostversion', 'The version of boost. If this is not none, the passed version is used and checks are not done. The version should look like "104200" for Boost "1.42".', None)
    vars.Add('boostlibsuffix', 'The suffix to add onto the boost library names.', 'auto')
    vars.Add(BoolVariable('platformflags',
                          'If true, add any compiler and linker arguments that might be needed/desired. If false, only used passed flags (eg only the values in "cxxflags", "linkflags" etc).',
                          True))
    vars.Add(BoolVariable('deprecated',
                          'Build deprecated classes and functions', False))
    vars.Add('percppcompilation',
                          'By default, all the .cpp files in a module are merged before building, greatly accelerating the process. This can be turned off globally by setting this variable to "yes" or per module by setting it to a colon separated list of module names, eg "em2d:kernel".', "no")
    vars.Add('pythonsosuffix', 'The suffix for the python libraries.', 'default')
    vars.Add('dot',
             'Use dot from graphviz to lay out graphs in the documentation if available. This produces prettier graphs, but is slow.',
                          "auto")
    # Set to False for stable builds
    svn_build = True
    vars.Add(BoolVariable('svn',
                          'True if this build is from an svn version of IMP. If so, SVN version info is added to the provided version number.',
                          svn_build))
    vars.Add('python', 'The path to python or "no" if python should not be used.', "python")
    vars.Add(BoolVariable('local', 'Whether to build local modules, applications and biological systems that are not part of the IMP distribution', True))
    vars.Add(BoolVariable('linktest', 'Test for header defined functions which are not inline', True))
    vars.Add(PathVariable('repository', 'Where to find the source code to build. This is only needed if building in a different directory than the source.', None, PathVariable.PathAccept)) #PathIsDir
    vars.Add(BoolVariable('static', 'Whether to build static libraries.', False))
    vars.Add(BoolVariable('dynamic', 'Whether to build dynamic libraries (needed for python support).', True))
    vars.Add(BoolVariable('precompiledheader', 'Whether to use a precompiled header for swig libraries ', False))
    vars.Add('disabledmodules', 'A colon-separated list of modules to disable.', '')
    vars.Add('datapath', "The path to the data of an ininstalled IMP you want to use.", None)
    vars.Add(BoolVariable('pretty', "Whether to write cleaner output when building.", True))
    vars.Add(BoolVariable('color', "Whether to write color output output when building.", True))
    vars.Add(EnumVariable('cppcoverage',
                      "Whether to report on C++ code coverage of tests. "
                      '"no" will do no reporting; '
                      '"lines" will list the lines of code that were missed; '
                      '"annotate" will make annotated copies of the code.'
                      " It is strongly recommended to combine this with "
                      "build='debug'. ",
                      'no', ['no', 'lines', 'annotate']))
    vars.Add(EnumVariable('pycoverage',
                      "Whether to report on Python code coverage of tests."
                      '"no" will do no reporting; '
                      '"lines" will list the lines of code that were missed; '
                      '"annotate" will make annotated copies of the code. '
                      'This requires a recent version of the Python coverage '
                      'module installed on your system.',
                      'no', ['no', 'lines', 'annotate']))
    vars.Add(EnumVariable('html_coverage',
                          'Whether to output a coverage report '
                          'in HTML format. Requires cppcoverage set and the '
                          'lcov package for C output, and pycoverage set '
                          'for Python output). "single" will output '
                          'a single report that covers all modules or '
                          'applications that were tested with this scons '
                          'invocation; "separate" will generate a separate '
                          'report for each module or application.',
                          'no', ['no', 'single', 'separate']))
    #vars.Add(BoolVariable('noexternaldependencies', 'Do not check files in the provided includepath and libpath for changes.', False))


def update(env, variables):
    variables.Update(env)
    _propagate_variables(env)
