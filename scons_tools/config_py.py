from SCons.Script import Glob, Dir, File, Builder, Action, Exit
import os
import sys
import re
import dependency.gcc

def _flatten(strorlist, delim):
    if type(strorlist)== type(""):
        return strorlist
    elif type(strorlist) == type(True):
        return str(strorlist)
    else:
        return delim.join([_flatten(x, delim) for x in list(strorlist)])

def write_config(h, env):
    config=env['IMP_CONFIGURATION']
    def _export_to_config(name, ename, env, opt=False, delim=":"):
        if opt and not env.get(ename, None):
            return
        #config.append("# "+str(type(env[ename])))
        #config.append( "# "+repr(env[ename]))
        config.append(name+"='"+_flatten(env.subst(env[ename]), delim)+"'")
    simple=['build', 'repository',
            'precommand',  'modeller',
            'prefix', 'local',
            'pythonpath', 'python_include', 'ldlibpath']
    if not dependency.gcc.get_is_gcc(env):
        simple.extend(['includepath', 'libpath'])
    for v in simple:
        _export_to_config(v, v, env, True)
    config.append('platformflags=False')
    for vp in [('cxxcompiler', 'CXX', " "),
               ('cxxflags', 'CXXFLAGS', " "),
               ('pythoncxxflags', 'IMP_PYTHON_CXXFLAGS', " "),
               ('shliblinkflags', 'IMP_SHLIB_LINKFLAGS', " "),
               ('arliblinkflags', 'IMP_ARLIB_LINKFLAGS', " "),
               ('binlinkflags', 'IMP_BIN_LINKFLAGS', " "),
               ('pythonlinkflags', 'IMP_PYTHON_LINKFLAGS', " "),
               ('python', 'IMP_PROVIDE_PYTHON', " "),
               ('rpath', 'IMP_USE_RPATH', " "),
               ('static', 'IMP_BUILD_STATIC', " "),
               ('pythonsosuffix', 'IMP_PYTHON_SO', " "),
               ('pkgconfig', "IMP_HAS_PKG_CONFIG", " ")]:
        _export_to_config(vp[0], vp[1], env, delim=vp[2])
    config.append('path="'+_flatten(os.environ['PATH'], ":")+'"')
    #print "opening h at " +target[0].abspath + " for module %(module)s"%vars
    #print "Generating "+str(h)
    s= config
    for l in s:
        print >> h, l


def _action_config_py(target, source, env):
    #config= source[0].get_contents().split("#")
    h = file(target[0].abspath, 'w')
    _do_write_config(h, env)

def _print_config_py(target, source, env):
    print "Generating config.py"

_ConfigPY = Builder(action=Action(_action_config_py,
                                 _print_config_py))

def add(env):
    env.Append(BUILDERS={'IMPConfigPY':_ConfigPY})
    config_py=env.IMPConfigPY(target=["#/config.py"],
                              source=[env.Value("#".join(env['IMP_CONFIGURATION']))])
    return config_py
