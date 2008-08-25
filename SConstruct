# Include IMP build utility functions:
from tools import *
from tools import boost, cgal

# We need scons 0.98 or later
EnsureSConsVersion(0, 98)

# Set up build environment:
opts = Options('config.py', ARGUMENTS)
add_common_options(opts, "imp")
opts.Add(PackageOption('embed', 'Location of the EMBED package', 'no'))
opts.Add(PackageOption('cgal', 'Location of the CGAL package', True))
env = MyEnvironment(options=opts, require_modeller=False,
                    tools=["default", "doxygen", "docbook", "imp_module"],
                    toolpath=["tools"])
boost.configure_check(env, '1.30')
cgal.configure_check(env)
Help("""
Available command-line options:
(These can also be specified in regular Python syntax by creating a file
called 'config.py' in this directory.)
""")
Help(opts.GenerateHelpText(env))

Help("""
Type: 'scons' to build the IMP kernel;
      'scons test' to run the kernel unit tests;
      'scons examples' to run the kernel examples;
      'scons install' to install the kernel.
""")

# Make these objects available to SConscript files:
Export('env', 'get_pyext_environment', 'get_sharedlib_environment',
       'invalidate_environment')

# Check code for coding standards:
standards = env.Command("standards", "SConstruct", "tools/check-standards.py")
env.AlwaysBuild(standards)

# Subdirectories to build:
bin = SConscript('bin/SConscript')
Export('bin')
(src, pyext) = SConscript('kernel/SConscript')
SConscript('em/SConscript')

# Uncomment when DOMINO code compiles with latest kernel:
#SConscript('domino/SConscript')

# bin script first requires kernel libraries to be built:
env.Depends(bin, [src, pyext])

# Build the binaries by default:
env.Default(bin)
