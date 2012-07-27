from SCons.Script import Glob, Dir, File, Builder, Action, Exit, Scanner, Move
import SCons
import os
import sys
import re
import scons_tools.dependency
import scons_tools.module
import scons_tools.data

# standard include files
base_includes= ["IMP_base.macros.i",
                "IMP_base.exceptions.i",
                "IMP_base.directors.i",
                "IMP_base.types.i",
                "IMP_base.refcount.i",
                "IMP_base.streams.i",
                "IMP_base.streams_kernel.i"]
kernel_includes= ["IMP_kernel.macros.i"]


def _null_scanner(node, env, path):
    #print "null scanning", node.abspath
    return []

# 1. Workaround for SWIG bug #1863647: Ensure that the PySwigIterator class
#    (SwigPyIterator in 1.3.38 or later) is renamed with a module-specific
#    prefix, to avoid collisions when using multiple modules
# 2. If module names contain '.' characters, SWIG emits these into the CPP
#    macros used in the director header. Work around this by replacing them
#    with '_'. A longer term fix is not to call our modules "IMP.foo" but
#    to say %module(package=IMP) foo but this doesn't work in SWIG stable
#    as of 1.3.36 (Python imports incorrectly come out as 'import foo'
#    rather than 'import IMP.foo'). See also IMP bug #41 at
#    https://salilab.org/imp/bugs/show_bug.cgi?id=41
def _action_patch_swig_wrap(target, source, env):
    lines = file(source[0].path, 'r').readlines()
    vars= scons_tools.module._get_module_variables(env)
    repl1 = '"swig::%s_PySwigIterator *"' % vars['PREPROC']
    repl2 = '"swig::%s_SwigPyIterator *"' % vars['PREPROC']
    orig = 'SWIG_IMP.%s_WRAP_H_' % scons_tools.module._get_module_name(env)
    repl = 'SWIG_IMP_%s_WRAP_H_' % scons_tools.module._get_module_name(env)
    fh= file(target[0].path, 'w')
    for line in lines:
        line = line.replace('"swig::PySwigIterator *"', repl1)
        line = line.replace('"swig::SwigPyIterator *"', repl2)
        line = line.replace(orig, repl)
        line = line.replace("wrap.h-in", "wrap.h")
        # for some reason swig has issues with directors and VersionInfo
        # when %extend is used
        line = line.replace(" VersionInfo ", " IMP::VersionInfo ")
        line = line.replace("(VersionInfo ", "(IMP::VersionInfo ")
        line = line.replace("<VersionInfo ", "<IMP::VersionInfo ")
        line = line.replace("<:", "< :") # swig generates bad C++ code
        fh.write(line.replace('"swig::SwigPyIterator *"', repl2))
    fh.close()

def _print_patch_swig_wrap(target, source, env):
    print "Patching swig file "+str(target[0])

PatchSwig = Builder(action=Action(_action_patch_swig_wrap,
                                _print_patch_swig_wrap))


def _action_swig_file(target, source, env):
    vars= scons_tools.module._get_module_variables(env)
    dta= scons_tools.data.get(env)
    deps= scons_tools.module._get_module_modules(env)
    deps.reverse()
    #print "dependencies are " +str(deps)
    warning="// WARNING Generated file, do not edit, edit the swig.i-in instead."
    preface=[warning,"""

%%module(directors="1") "%s"
%%feature("autodoc", 1);
// turn off the warning as it mostly triggers on methods (and lots of them)
%%warnfilter(321);

%%{
/* SWIG generates long class names with wrappers that use certain Boost classes,
   longer than the 255 character name length for MSVC. This shouldn't affect
   the code, but does result in a lot of warning output, so disable this warning
   for clarity. */
#ifdef _MSC_VER
#pragma warning( disable: 4503 )
#endif

#include <boost/version.hpp>
#if BOOST_VERSION > 103600
#if BOOST_VERSION > 103800
#include <boost/exception/all.hpp>
#else
#include <boost/exception.hpp>
#endif
#endif

#include <boost/type_traits/is_convertible.hpp>
#include <boost/utility/enable_if.hpp>
#include <exception>
"""%vars['module_include_path'].replace("/", ".")]
    dta= scons_tools.data.get(env)
    for d in deps+[vars['module']]:
        # kind of evil
        if d== "kernel":
            preface.append('#include "IMP.h"')
            preface.append('#include "IMP/internal/swig.h"')
            preface.append('#include "IMP/internal/swig_helpers.h"')
            preface.append('#include "IMP/kernel_config.h"')
        elif d=="base":
            preface.append('#include "IMP/base.h"')
            preface.append('#include "IMP/base/internal/swig.h"')
            preface.append('#include "IMP/base/internal/swig_helpers.h"')
            preface.append('#include "IMP/base/base_config.h"')
        else:
            ln= dta.modules[d].libname
            nm=ln.replace("imp", "IMP").replace("IMP_", "IMP/")
            preface.append("#include \"%s.h\""% nm)
            preface.append("#include \"%s/%s_config.h\""% (nm, d))
    preface.append("""%}
%implicitconv;
%include "std_vector.i"
%include "std_string.i"
%include "std_pair.i"
""")
    for d in deps+[vars['module']]:
        #print d
        if d== "base":
            for i in base_includes:
                preface.append('%%include "%s"'%i)
            preface.append('%include "IMP/base/base_config.h"')
            preface.append('%import "IMP_base.i"')
        elif d== "kernel":
            preface.append('%include "IMP/kernel_config.h"')
            preface.append('%import "IMP_kernel.i"')
            for i in kernel_includes:
                preface.append('%%include "%s"'%i)
        elif d=="RMF":
            preface.append('%include "RMF/RMF_config.h"')
            preface.append('%import "RMF.i"')
        else:
            ln= dta.modules[d].libname
            sn= ln.replace("imp", "IMP")
            nm=ln.replace("imp", "IMP").replace("IMP_", "IMP/")
            preface.append('%%include "%s/%s_config.h"'%(nm, d))
            preface.append('%%import "%s.i"'%sn)
    preface.append("""
%pythoncode %{
_value_types=[]
_object_types=[]
_raii_types=[]
_plural_types=[]
%}

""")
    preface.append("""
%%include "typemaps.i"
"""%vars)
    preface.append(warning)

    # special case the kernel to make sure that VersionInfo is wrapped
    # before get_version_info() is wrapped.
    preface.append("""%{
#ifdef NDEBUG
#error "The python wrappers must not be built with NDEBUG"
#endif
%}
""")
    dta= scons_tools.data.get(env)
    preface.append(warning)

    preface.append(warning)
    preface.append(open(source[0].abspath, "r").read())
    preface.append(warning)
    for ns in vars['namespace'].split("::"):
        preface.append("namespace "+ns + " {")
    preface.append("""
const std::string get_module_version();
""")
    if source[4].get_contents()=="True":
        preface.append("""
        std::string get_example_path(std::string fname);
        std::string get_data_path(std::string fname);
""")
    for ns in vars['namespace'].split("::"):
        preface.append("}")
    preface.append(warning)
    deps= source[2].get_contents().split(" ")
    udeps= source[3].get_contents().split(" ")
    preface.append("%pythoncode {")
    for d in deps:
        # avoid issues with dumb encoding as strings
        if d != "":
            nm=scons_tools.dependency.get_dependency_string(d).lower()
            preface.append("has_"+nm +"=True")
    for d in udeps:
        # avoid issues with dumb encoding as strings
        if d != "":
            nm=scons_tools.dependency.get_dependency_string(d).lower()
            preface.append("has_"+nm+"=False")
    preface.append("}")
    preface.append("""
%pythoncode %{
import _version_check
_version_check.check_version(get_module_version())
%}""")
    open(target[0].abspath, "w").write("\n".join(preface))

def _print_swig_file(target, source, env):
    print "Generating swig interface "+str(target[0]) + " from " +str(source[0])

SwigPreface = Builder(action=Action(_action_swig_file,
                                    _print_swig_file))


def _action_simple_swig(target, source, env):
    vars= scons_tools.module._get_module_variables(env)
    cppflags= ""
    for x in env.get('CPPFLAGS', []):
        if x.startswith("-I") or x.startswith("-D"):
            cppflags= cppflags+" " + x
    sv= env['SWIGVERSION'].split(".")
    if sv[0]=="1" and sv[1] == "3" and int(sv[2])<34:
        warnings=[]
    else:
        warnings=["-Wextra"]

    command = [env['SWIG'], "-castmode -interface", "%(module_pylibname)s",
               "-DPySwigIterator=%(PREPROC)s_PySwigIterator",
               "-DSwigPyIterator=%(PREPROC)s_SwigPyIterator",
               "-python", "-c++", "-naturalvar",
               "-fvirtual"]+warnings
    # if building a module by itself, we need to find swig headers
    command+= ["-I"+x for x in
               scons_tools.utility.get_env_paths(env, 'includepath')]\

    # Signal whether we are building the kernel
    if scons_tools.module._get_module_name(env) == 'kernel':
        command.append('-DIMP_SWIG_KERNEL')
    #print base
    command=command+["-o",target[1].abspath, "-oh",target[2].abspath]
    ussp=env.get('swigpath', "")
    command=command+[" -I"+Dir("#/build/swig").path]\
        + ["-I"+Dir("#/build/include").abspath]\
        + ["-I"+str(x) for x in
           scons_tools.utility.get_env_paths(env, 'swigpath')]
    command.append("-DIMP_SWIG")
    command.append(source[0].abspath)
    final_command=" ".join(command) %vars
    ret= env.Execute(final_command)

    modulename = vars['module']
    # Prevent collision between RMF and rmf on case-insensitive filesystems
    if modulename == 'RMF':
        modulename = 'librmf'
    oname=File("#/build/src/"+modulename+"/"\
                   +vars['module_include_path'].replace("/", ".")+".py")
    #print oname.path, "moving to", target[0].path
    # scons build in Move produces an error with no explaination
    ret= env.Execute("mv "+oname.abspath+" "+target[0].abspath)
    return ret

def _print_simple_swig(target, source, env):
    print "Running swig on file "+str(source[0].path)

def _action_version_check(target, source, env):
    def get_module(name):
        dta= scons_tools.data.get(env)
        ln= dta.modules[name].libname
        inm= ln.replace("imp_", "imp.").replace("imp", "IMP")
        return inm
    out= open(target[0].abspath, "w")
    print >> out, "def check_version(myversion):"
    print >> out, "  def _check_one(name, expected, found):"
    print >> out, "    if expected != found:"
    print >> out, "      raise RuntimeError('Expected version '+expected+' but got '+ found \\"
    print >> out, "           +' when loading module '+name\\"
    print >> out, "            +'. Please make sure IMP is properly built and installed and that matching python and C++ libraries are used.\\n')"
    myversion= source[0].get_contents()
    print >> out, "  _check_one('"+scons_tools.module._get_module_name(env)+\
          "', '"+myversion+"', myversion)"
    for i in range(1,len(source), 2):
        mn= source[i].get_contents()
        ver= source[i+1].get_contents()
        print >> out, "  import "+get_module(mn)
        print >> out, "  _check_one('"+mn+\
          "', '"+ver+"', "+get_module(mn)+".get_module_version())"

def _print_version_check(target, source, env):
    print "Building version check "+target[0].abspath

VersionCheck = Builder(action=Action(_action_version_check,
                                _print_version_check))


def swig_scanner(node, env, path):
    import re
    contents= node.get_contents()
    # scons recurses with the same scanner, rather than the right one
    # print "Scanning "+str(node)
    dta= scons_tools.data.get(env)
    if str(node).endswith(".h"):
        # we don't care about recursive .hs for running swig
        return []
    else :
        oldret=[]
        ret=[]
        for x in re.findall('\n%include\s"([^"]*.h)"', contents):
            if x.startswith("IMP/"):
                xc= x[4:]
                if xc.find("/") != -1:
                    module= xc[0:xc.find("/")]
                else:
                    module="kernel"
                if module=="internal":
                    module="kernel"
                if not dta.modules[module].external:
                    ret.extend(["#/build/include/"+x])
            if x.startswith("RMF/") and not dta.modules["RMF"].external:
                ret.extend([File("#/build/include/"+x)])

        for x in re.findall('\n%include\s"IMP_([^"]*).i"', contents)\
                +re.findall('\n%import\s"IMP_([^"]*).i"', contents):
            mn= x.split(".")[0]
            if not dta.modules[mn].external:
                ret.append("#/build/swig/IMP_"+x+".i")
        if not dta.modules["RMF"].external:
            if re.search('\n%include\s"RMF.i"', contents)\
                    or re.search('\n%import\s"RMF.i"', contents):
                ret.append("#/build/swig/RMF.i")
            for x in re.findall('\n%include\s"RMF.([^"]*).i"', contents)\
                    +re.findall('\n%import\s"RMF.([^"]*).i"', contents):
                ret.append("#/build/swig/RMF."+x+".i")
        retset=set(ret)
        ret=list(retset)
        ret.sort()
    return ret

def inswig_scanner(node, env, path):
    if str(node).endswith(".i") or str(node).endswith(".h"):
        return swig_scanner(node, env, path)
    ret= swig_scanner(node, env, path)
    for i in base_includes:
        if not dta.modules[i].external:
            f= "#/build/swig/"+i
            ret.append(f)
    for m in scons_tools.module._get_module_python_modules(env):
        if not dta.modules[m].external:
            ret.append("#/modules/"+m+"/pyext/swig.i-in")
    return ret.sorted()

scanner= Scanner(function=swig_scanner, skeys=['.i'], name="IMPSWIG", recursive=True)
# scons likes to call the scanner on nodes which do not exist (making it tricky to parse their contents
# so we have to walk higher up in the tree
inscanner= Scanner(function=inswig_scanner, skeys=['.i-in'], name="IMPINSWIG", recursive=True)

def get_swig_action(env):
    comstr="%sRunning swig on %s$SOURCE%s"%(env['IMP_COLORS']['purple'],
                                            env['IMP_COLORS']['end'],
                                            env['IMP_COLORS']['end'])
    return Builder(action=Action(_action_simple_swig,
                                 _print_simple_swig,
                                 comstr=comstr),
                   source_scanner= scanner,
                   comstr=comstr)
