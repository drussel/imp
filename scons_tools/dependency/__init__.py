import scons_tools.utility
import scons_tools.data
import scons_tools.paths
import SCons
import os
from SCons.Script import File, Action, Dir

def _search_for_deps(context, libname, extra_libs, headers, body, possible_deps):
    for i in range(0,len(possible_deps)+1):
        lc= extra_libs+possible_deps[0:i]
        #print "Trying "+ str(i) +" with " +str(lc)
        olibs= context.env.get('LIBS', [])
        if libname is not None:
            context.env.Append(LIBS=[libname]+lc)
        else:
            context.env.Append(LIBS=lc)
            #print context.env["LIBPATH"]
            #print context.env["CPPPATH"]
        #print context.env['LINKFLAGS']
        #print "checking", libname, lc
        ret=context.sconf.CheckLibWithHeader(libname, header=headers, call=body, language='CXX',
                                             autoadd=False)
        context.env.Replace(LIBS=olibs)
        if ret:
            if libname is None:
                libs = lc
            else:
                libs = [libname] + lc
            return (True, libs, None)
    return (False, None, None)

def add_dependency_link_flags(env, dependencies):
    for d in dependencies:
        env.Append(LIBS=scons_tools.data.get(env).dependencies[d].libs)

def _get_version(context, name, includepath, versioncpp, versionheader):
    if versioncpp:
        context.Message('Checking for version of '+name+"...")
        vs="<< ' ' << ".join(versioncpp)
        #if includepath:
        #    oldcpp= context.env.get('CPPPATH', None)
        #    context.env.Replace(CPPPATH=context.env.get('CPPPATH', [])[:]+[includepath])
        #print context.env['LINKFLAGS'], context.env['RPATH']
        r = context.TryRun("#include <"+versionheader+">\n"+\
                             """#include <iostream>

        int main()
        {
            std::cout << """+vs +"""<<std::endl;
            return 0;
        }
""", '.cpp')
        if includepath:
            pass
            #context.env.Replace(CPPPATH=oldcpp)
        if not r[0]:
            context.Result("None")
            return None
        else:
            v= r[1].split('\n')[0]
            if type(versioncpp) == type([]):
                version=v.split()
            else:
                version=v
        if type(version) == type([]):
            context.Result(" ".join(version))
        else:
            context.Result(str(version))
        return version
    else:
        return None

def check_lib(context, name, lib, header, body="", extra_libs=[], versioncpp=None,
              versionheader=None):
    if lib is not None and type(lib) != type([]):
        scons_tools.utility.report_error(context.env,
                                         "The lib argument must be given as a list. It was not for "+name)
    if versioncpp != None and type(versioncpp) != type([]):
        scons_tools.utility.report_error(context.env,
                                         "The versioncpp argument must be given as a list. It was not for "+name)
    #oldflags= context.env.get('LINKFLAGS')
    #context.env.Replace(LINKFLAGS=context.env['IMP_BIN_LINKFLAGS'])
    #print context.env["LIBPATH"]
    #print context.env["CPPPATH"]

    if lib is not None:
        ret=_search_for_deps(context, lib[0], lib[1:], header, body, extra_libs)
    else:
        ret=(context.sconf.CheckHeader(header, language="C++"), [])
    if not ret[0]:
        #context.env.Replace(LINKFLAGS=oldflags)
        return (ret[0], ret[1], None)
    if context.env['IMP_OUTER_ENVIRONMENT']['IMP_BUILD_STATIC'] and lib != None:
        scons_tools.utility.make_static_build(context.env)
        if type(lib) == list:
            bret=_search_for_deps(context, lib[0], lib[1:], header, body, extra_libs)
        else:
            bret=_search_for_deps(context, lib, [], header, body, extra_libs)
        scons_tools.utility.unmake_static_build(context.env)
        # should be the sum of the two
        if bret[0]:
            #context.env.Replace(LINKFLAGS=oldflags)
            return (bret[0], ret[1]+bret[1], _get_version(context, name, None,
                                                          versioncpp,
                                                          versionheader))
        else:
            #context.env.Replace(LINKFLAGS=oldflags)
            return (False, [], None)
    vers= _get_version(context, name, None, versioncpp, versionheader)
    #print "version", vers
    #context.env.Replace(LINKFLAGS=oldflags)
    return  (True, ret[1], vers)

def get_dependency_string(name):
    lname= name.lower()
    nname=lname.replace(".", "_")
    return nname

# return (ok, libs, version, includepath, libpath)
def _get_bad():
    return (False, None, None, None, None)
def _get_info_variables(context, env, name, has_version):
    lcname= get_dependency_string(name)
    context.Message('Checking for '+name+' with variables...')
    if not env['IMP_OUTER_ENVIRONMENT'].get(lcname, None)\
            or env['IMP_OUTER_ENVIRONMENT'].get(lcname) != "yes":
        context.Result("no")
        return _get_bad()
    if env['IMP_OUTER_ENVIRONMENT'].get(lcname+"libs", None) is None:
        scons_tools.utility.report_error(env['IMP_OUTER_ENVIRONMENT'],
                                         "If configure specifies 'yes' for "+
                                         name+" it must also specify "+lcname+"libs"+
                                         env['IMP_OUTER_ENVIRONMENT'].get(lcname+"libs", "no found"))
        context.Result("no")
        return _get_bad()
    if has_version and not env['IMP_OUTER_ENVIRONMENT'].get(lcname+"version", None):
        scons_tools.utility.report_error(env['IMP_OUTER_ENVIRONMENT'], "If configure specifies 'yes' for "+
                                         name+" it must also specify "+lcname+"version")
        context.Result("no")
        return _get_bad()
    vers=None
    if has_version:
        vers= env['IMP_OUTER_ENVIRONMENT'].get(lcname+'version')
        if vers.find(" ") != -1:
            vers=vers.split()
        else:
            vers=[vers]
    context.Result("yes")
    return (True, env['IMP_OUTER_ENVIRONMENT'].get(lcname+"libs").split(":"),
            vers, None, None)
def _get_info_pkgconfig(context, env,  name, versioncpp, versionheader):
    if not context.env['IMP_OUTER_ENVIRONMENT']['IMP_HAS_PKG_CONFIG']:
        return _get_bad()
    lcname= get_dependency_string(name)
    context.Message('Checking for '+name+' with pkg-config...')
    ret = context.TryAction('pkg-config --exists \'%s\'' % lcname)[0]
    if not ret:
        context.Result("no")
        return _get_bad()
    context.Result("yes")
    (includepath, libpath, libs)= scons_tools.dependency.pkgconfig.get_config(context, lcname)
    if not versioncpp:
        version=None
    else:
        version= _get_version(context, name, includepath, versioncpp, versionheader)
    return (True, libs, version, includepath, libpath)

def _get_info_test(context, env, name, lib, header, body,
                   extra_libs, versioncpp, versionheader):
    lcname= get_dependency_string(name)
    #print context.env["LIBPATH"]
    #print context.env["CPPPATH"]

    (ret, libs, version)= check_lib(context, name, lib=lib, header=header,
                                    body=body,
                                    extra_libs=extra_libs,
                                    versioncpp=versioncpp,
                                    versionheader=versionheader)
    if not ret:
        return _get_bad()
    else:
        return (True, libs, version, None, None)


def add_external_library(env, name, lib, header, body="", extra_libs=[],
                         versioncpp=None, versionheader=None,
                         enabled=True, build=None):
    tenv= scons_tools.environment.get_test_environment(env)
    lcname= get_dependency_string(name)
    ucname= lcname.upper()
    dta= scons_tools.data.get(env)
    if scons_tools.data.get_has_configured_dependency(name):
        # already has been added
        return
    variables=[lcname, lcname+"libs", lcname+"version"]
    def _check(context):
        local=False
        pythonpath=None
        if context.env['IMP_OUTER_ENVIRONMENT'][lcname] == "no":
            context.Message('Checking for '+name+' ...')
            context.Result("disabled")
            scons_tools.data.add_dependency(name, variables=variables,
                                                             ok=False)
            ok=False
        else:
            (ok, libs, version, includepath, libpath)\
                  = _get_info_variables(context, context.env, name, versioncpp)
            if not ok:
                (ok, libs, version, includepath, libpath)=\
                      _get_info_pkgconfig(context, env, name, versioncpp, versionheader)
                if not ok:
                    (ok, libs, version, includepath, libpath)=\
                      _get_info_test(context, env, name, lib, header, body,
                                      extra_libs, versioncpp, versionheader)
                    if not ok and build:
                        local=True
                        paths={"builddir":Dir("#/build/").abspath,
                               "workdir":Dir("#/build/src/"+name).abspath,
                               "srcdir":scons_tools.paths.get_input_path(context.env, "dependency/"+name)}
                        if not os.path.exists(paths["workdir"]):
                            os.makedirs(paths["workdir"])

                        buildscript= build%paths
                        try:
                            os.system(buildscript)
                            (ok, libs, version, includepath, libpath)=\
                             _get_info_test(context, env, name, lib, header, body,
                                extra_libs, versioncpp, versionheader)
                                 #print "found", ok
                        except:
                            pass

            if not ok:
                scons_tools.data.add_dependency(name, variables=variables,
                                                                 ok=False)
                return False
            else:
                if not version:
                    pversioncpp=None
                    pversionheader=None
                else:
                    pversioncpp=versioncpp
                    pversionheader=versionheader
                scons_tools.data.add_dependency(name,
                                   variables=variables,
                                   libs=libs,
                                   includepath=includepath,
                                   libpath=libpath,
                                   pythonpath=pythonpath,
                                   version=version,
                                   versioncpp=pversioncpp,
                                   versionheader=pversionheader,
                                   local=local,
                                   build=build)
                return True
    vars = env['IMP_VARIABLES']
    if enabled:
        vars.Add(SCons.Variables.EnumVariable(lcname, 'Whether to use the '+name+' package', "auto", ["yes", "no", "auto"]))
    else:
        vars.Add(SCons.Variables.EnumVariable(lcname, 'Whether to use the '+name+' package', "no", ["yes", "no", "auto"]))
    vars.Add(lcname+'libs', 'Libs to link against when using '+name+'. Needed if "'+lcname+'" is "yes".', None)
    vars.Add(lcname+'version', 'Version to test against when using '+name, None)
    vars.Update(env)
    if not env.GetOption('help'):
        custom_tests = {'CheckThisLib':_check}
        conf = tenv.Configure(custom_tests=custom_tests)
    #if not env.GetOption('clean') and not env.GetOption('help'):
        if conf.CheckThisLib():
            env.Append(IMP_ENABLED=[name])
        else:
            env.Append(IMP_DISABLED=[name])
            env.Append(IMP_CONFIGURATION=[lcname+"='no'"])
        conf.Finish()
