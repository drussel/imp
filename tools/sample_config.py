import os
import os.path

## for convenience
extra_roots=[os.path.join(os.environ['HOME'],"fs")]

# if you use macporst
extra_roots.append("/opt/local/")

## Where to install IMP
prefix="/tmp/imp"


pythonpath=""
includepath=""
libpath=""
swigpath=""
path=""
## Add my local environment to that used by scons and tools/imppy.sh
try:
    pythonpath=os.environ['PYTHONPATH']
except:
    pass
try:
    ldlibpath=os.environ['LD_LIBRARY_PATH']
except:
    pass

def add(old, root, dir):
    import os.path
    return ":".join([old,os.path.join(root, dir)])

for r in extra_roots:
    includepath=add(includepath, r, "include")
    libpath=add(includepath, r, "lib")
    swigpath=add(includepath, r, "share/imp/swig")
    path=add(includepath, r, "bin")

# uncomment if you are building an isolated module/application
#imp_build="/Users/drussel/src/IMP/imp/debug
#includepath=add(includepath, imp_build, "include")
#libpath=add(includepath, imp_build, "lib")
#swigpath=add(includepath, imp_build, "swig")
#pythonpath=add(pythonpath, imp_build, "lib")



## The build more can be one of 'debug', 'release' or 'fast'.
## It is recommended that you use a 'debug' build for testing and development
## and a 'fast' build for running the final code.
build='debug'

## It is recommented that you build in a different directory than the sources
## to do this, have your config.py and a link to the SConscruct files and scons_tools
## directories in the build directory and set 'repository' to a path to the source
## directory.
repository="path_to_source"

# Run all tests with valgrind
#precommand="valgrind --gen-suppressions=yes --db-attach=yes --undef-value-errors=no --suppressions=../svn/tools/valgrind-python.supp"

## Get modeller's location from an environment variable
#modeller=os.environ['MODINSTALLSVN']
