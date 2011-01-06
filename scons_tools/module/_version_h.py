
import scons_tools.module
from SCons.Script import Glob, Dir, File, Builder, Action, Exit
import os
import sys
import re


def _action_version_h(target, source, env):
    vars= scons_tools.module._get_module_variables(env)
    #print "opening h at " +target[0].abspath + " for module %(module)s"%vars
    h = file(target[0].abspath, 'w')
    #print "Generating "+str(h)
    print >> h, """/*
 * \\file %(module_include_path)s/%(module)s_version.h
 * \\brief Provide the module version in a way available to the preprocessor
 *
 * This header is auto-generated by scons_tools/module/_version_h.py; it should not be
 * edited manually.
 *
 * Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef %(PREPROC)s_VERSION_H
#define %(PREPROC)s_VERSION_H

""" % vars

    vers= source[0].get_contents()
    if vers.startswith("SVN"):
        svn=True
        vers=vers[4:]
    else:
        svn=False
    version=vers.split(" ")[0]
    if version.find(":") != -1:
        min_version=version.split(":")[0]
        max_version=version.split(":")[1]
    else:
        min_version=version
        max_version=version
    local=False
    if min_version.endswith("M"):
        min_version=min_version[:-1]
        local=True
    if max_version.endswith("M"):
        max_version=max_version[:-1]
        local=True
    #print vers, version
    print >> h, "#define %(PREPROC)s_MIN_VERSION "%vars + min_version
    print >> h, "#define %(PREPROC)s_VERSION "%vars + max_version
    if svn:
        print >> h, "#define %(PREPROC)s_IS_SVN 1"%vars
    if local:
        print >> h, "#define %(PREPROC)s_HAS_LOCAL_CHANGES 1"%vars
    # test that it is numeric
    print >> h, """#if %(PREPROC)s_VERSION <0
#endif"""%vars

    print >> h, """
#endif  /* %(PREPROC)s_CONFIG_H */""" % vars

def _print_version_h(target, source, env):
    vars= scons_tools.module._get_module_variables(env)
    print "Generating %(module)s_config.h"%vars

VersionH = Builder(action=Action(_action_version_h,
                                 _print_version_h))
