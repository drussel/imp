"""Utility function to install a directory hierarchy of include files, with
   a top-level 'include all' header."""

import os
import UserList
from SCons.Script import Action, Entry, File
import imp_module

# should merge with one in imp_module.py




def _build_header(target, source, env):
    vars= imp_module.make_vars(env);
    fh = file(target[0].abspath, 'w')
    print >> fh, "/**\n *  \\file %(module_include_path)s.h   \\brief Include all the headers\n *" \
             % vars
    print >> fh, " *  Copyright 2007-9 Sali Lab. All rights reserved."
    print >> fh, " *\n */\n"
    print >> fh, "#ifndef %(PREPROC)s_H\n#define %(PREPROC)s_H\n" % vars
    # prefix does not work when there are a mix of generated and source files
    #= len(os.path.commonprefix([f.path for f in source]))
    for f in source[0].get_contents().split(" "):
        #print src
        if not f.startswith('internal'):
            vars['header']= f
            print >> fh, '#include <%(module_include_path)s/%(header)s>' %vars
    print >> fh, "\n#endif  /* %(PREPROC)s_H */" % vars

def _make_nodes(files):
    nodes = []
    for f in files:
        if isinstance(f, str):
            nodes.append(Entry(f))
        elif isinstance(f, (list, tuple, UserList.UserList)):
            nodes.extend(_make_nodes(f))
        else:
            nodes.append(f)
    return nodes

def _install_hierarchy_internal(env, dir, sources, can_link):
    insttargets = []
    sources = _make_nodes(sources)
    for f in sources:
        full = f.path
        if full.find("include") != -1:
            src = full[full.find("include")+8:]
        elif full.find("src") != -1:
            src= full[full.find("src")+4]
        elif full.find("data") != -1:
            src= full[full.find("data")+5]
        elif full.find("examples") != -1:
            src= full[full.find("examples")+9]
        else:
            raise ValueError(full)
        dest = os.path.join(dir, os.path.dirname(src))
        if can_link:
            insttargets.append(env.LinkInstall(dest, f))
        else:
            insttargets.append(env.Install(dest, f))
    return insttargets

def InstallHierarchy(env, dir, sources, can_link=False):
    """Given a set of header files, install them all under `dir`. A file
       dir.h is created , and with the given comment description. A list of all
       installed headers is returned, suitable for an 'install' alias."""
    targets = \
       _install_hierarchy_internal(env, dir, sources, can_link)
    source_list=[str(x) for x in sources]
    source_list.sort()
    t = env.Command(dir + '.h', env.Value(" ".join(source_list)),
                    Action(_build_header,
                               'Auto-generating header ${TARGET}'))

    targets.append(t)
    return targets

def InstallPythonHierarchy(env, dir, sources, can_link=False):
    """Given a set of Python files, install them all under `dir`. They are
       placed in the `module` subdirectory (common prefix is stripped from the
       filenames). A list of all installed files is returned, suitable for an
       'install' alias, plus another list of the files in the build
       directory."""
    return _install_hierarchy_internal(env, dir, sources, can_link)

def InstallDataHierarchy(env, dir, sources, can_link):
    """Given a set of data files, install them all under `dir`. They are
       placed in the `module` subdirectory (common prefix is stripped from the
       filenames). A list of all installed files is returned, suitable for an
       'install' alias, plus another list of the files in the build
       directory."""
    return _install_hierarchy_internal(env, dir, sources, can_link)

def InstallExampleHierarchy(env, dir, sources, can_link):
    """Given a set of data files, install them all under `dir`. They are
       placed in the `module` subdirectory (common prefix is stripped from the
       filenames). A list of all installed files is returned, suitable for an
       'install' alias, plus another list of the files in the build
       directory."""
    return _install_hierarchy_internal(env, dir, sources, can_link)
