"""Simple configure checks for boost"""

from SCons.Script import Exit
import gcc
import clang
import os
import scons_tools.utility

def _check(context):
    context.Message('Checking for pkg-config >= 0.15... ')
    ret = context.TryAction('pkg-config --atleast-pkgconfig-version=0.15')[0]
    context.Result(ret)
    return ret


def configure_check(env):
    if env.get('pkgconfig') == "auto":
        # We currently only parse pkg-config output for gcc (or closely related
        # compilers, such as clang), so don't use
        # pkg-config on non-gcc systems unless the user forces us to
        # Note: really we should use pkg-config for any compiler that accepts
        # gcc-like options (-I, -L etc.); need a configure test
        if gcc.get_is_gcc(env) or clang.get_is_clang(env):
            custom_tests = {'CheckPK':_check}
            conf = env.Configure(custom_tests=custom_tests)
            if not conf.CheckPK():
                env['IMP_HAS_PKG_CONFIG']=False
            else:
                env['IMP_HAS_PKG_CONFIG']=True
            conf.Finish()
        else:
            env['IMP_HAS_PKG_CONFIG']=False
    elif env.get('pkgconfig')=="no":
        env['IMP_HAS_PKG_CONFIG']=False
    else:
        env['IMP_HAS_PKG_CONFIG']=True


def get_config(context, lcname):
    if not gcc.get_is_gcc(context.env) and not clang.get_is_clang(context.env):
        scons_tools.utility.report_error(context.env,
                                         "pkg-config only supported with g++ like compilers")
    #print context.env.Execute('pkg-config --cflags-only-I \'%s\'' % lcname)
    retI = os.popen('pkg-config --cflags-only-I \'%s\'' % lcname).read()
    retL = os.popen('pkg-config --libs-only-L \'%s\'' % lcname).read()
    retl = os.popen('pkg-config --libs-only-l \'%s\'' % lcname).read()
    #retl = os.popen('pkg-config --version \'%s\'' % lcname).read()
    #print retI[:-1]
    #print retL[:-1]
    #print retl[:-1]
    if retI.startswith("-I"):
        inc=[retI[2:-1].strip()]
    else:
        inc=None
    if retL.startswith("-L"):
        lp=[retL[2:-1].strip()]
    else:
        lp=None
    libs=[x[2:].strip() for x in retl[:-1].split(" ") if x != '']
    #version= retl.split('\n')[0]
    #print "version", version
    ret= (inc, lp, libs)
    #print ret
    return ret
