import sys
import re
import os
import shutil
import glob
import fnmatch
import tempfile
from SCons.Script import Dir
import environment

class _TempDir(object):
    """Simple RAII-style class to make a temporary directory for gcov"""
    def __init__(self, copy_graph_files):
        self._origdir = os.getcwd()
        self.prefix_strip = len(self._origdir.split(os.path.sep)) - 1
        self.tmpdir = tempfile.mkdtemp()
        # Fool gcov into thinking this dir is the IMP top-level dir
        self._link_tree('modules', copy_graph_files)
        self._link_tree('applications', copy_graph_files)
        os.mkdir(os.path.join(self.tmpdir, 'build'))
        self._link_tree('build/src', copy_graph_files)
        os.symlink(os.path.join(self._origdir, 'build', 'include'),
                   os.path.join(self.tmpdir, 'build', 'include'))

    def _link_tree(self, subdir, copy_graph_files):
        lenorig = len(self._origdir)
        for root, dirs, files in os.walk(self._origdir + '/' + subdir):
            # Reproduce each directory under the temporary directory
            tmpdir = os.path.join(self.tmpdir, root[lenorig+1:])
            os.mkdir(tmpdir)
            for f in files:
                # Link any cpp files into the new directory
                if f.endswith('.cpp') or f.endswith('.h'):
                    os.symlink(os.path.join(root, f), os.path.join(tmpdir, f))
                # Link or copy any graph files into the new directory (links
                # are fine to fool gcov, but lcov resolves symlinks so we
                # need actual copies if we want HTML coverage reports)
                elif f.endswith('.gcno'):
                    orig = os.path.join(root, f)
                    dest = os.path.join(tmpdir, f)
                    if copy_graph_files:
                        shutil.copy(orig, dest)
                    else:
                        os.symlink(orig, dest)
            # Prune uninteresting subdirectories
            for prune in ('bin', 'data', 'doc', 'examples', 'include',
                          'pyext', 'test', 'generated', '.svn'):
                if prune in dirs:
                    dirs.remove(prune)
    def __del__(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)


class _CoverageTester(object):
    def __init__(self, env, coverage, test_type, output_file):
        self._env = env
        self._test_type = None
        self._sources = []
        self._headers = []
        self._header_callcounts = {}
        self._output_file = output_file
        self._coverage = coverage
        self._html_coverage = env.get('html_coverage', False)
        self._coverage_dir = Dir(env["builddir"]+"/coverage").abspath
        self._name = name = environment.get_current_name(env)
        if test_type.startswith('module'):
            self._test_type = 'module'
            moddir = self.get_module_dir(name)
            self.add_source('modules/%s/src' % moddir, '*.cpp', report=True)
            self.add_source('modules/%s/src/internal' % moddir, '*.cpp',
                            report=True)
            self.add_source('build/src', self.get_wrap(name) + '.cpp',
                            report=False)

            if name == 'kernel':
                h = 'IMP'
            elif name == 'RMF':
                h = name
            else:
                h = 'IMP/' + name
            self.add_header('build/include/%s' % h, '*.h', report=True)
            self.add_header('build/include/%s/internal' % h, '*.h', report=True)
        elif test_type.startswith('application'):
            self._test_type = 'application'
            m = re.search('(applications/(.+?/)?%s)' % name, output_file)
            if m:
                self.add_source(m.group(1), '*.cpp', report=True, recurse=True)
                self.add_header(m.group(1), '*.h', report=True, recurse=True)
            else:
                raise ValueError("Cannot determine path for %s" % name)

    def get_wrap(self, name):
        if name == 'RMF':
            return 'RMF_wrap'
        else:
            return 'IMP_%s_wrap' % name

    def get_module_dir(self, name):
        if name == 'RMF':
            return 'librmf'
        else:
            return name

    def add_source(self, directory, pattern, report, recurse=False):
        self._add_source_or_header(self._sources, directory, pattern,
                                   report, recurse)

    def add_header(self, directory, pattern, report, recurse=False):
        self._add_source_or_header(self._headers, directory, pattern,
                                   report, recurse)

    def _add_source_or_header(self, sources, directory, pattern,
                              report, recurse):
        if recurse:
            for dirpath, dirnames, filenames in os.walk(directory):
                if len(glob.glob(os.path.join(dirpath, pattern))) > 0:
                    sources.append([dirpath, pattern, report])
        else:
            sources.append([directory, pattern, report])

    def Execute(self, *args, **keys):
        self._tmpdir = _TempDir(self._html_coverage)
        self._env['ENV']['GCOV_PREFIX'] = self._tmpdir.tmpdir
        self._env['ENV']['GCOV_PREFIX_STRIP'] = self._tmpdir.prefix_strip
        ret = self._env.Execute(*args, **keys)
        if self._test_type:
            self._report()
        return ret

    def _report(self):
        if self._coverage == 'lines':
            self._report_lines()
        elif self._coverage == 'annotate':
            self._report_annotate()
        if self._html_coverage:
            self._make_lcov_info_file()

    def _report_annotate(self):
        t = self._tmpdir
        for dir, pattern, report in self._sources:
            self._run_gcov(dir, pattern, running_dir=t.tmpdir)
            covs = glob.glob(os.path.join(t.tmpdir, "*.gcov"))
            for cov in covs:
                ret = self._parse_gcov_file(cov)
                if ret:
                    shutil.copy(cov, '.')
        for header in self._header_callcounts.keys():
            self._make_gcov_for_header(header, self._header_callcounts[header])
        print >> sys.stderr, \
                 "\nC++ coverage of %s %s written to *.gcov in " \
                 "top-level directory." % (self._name, self._test_type)

    def _report_lines(self):
        t = self._tmpdir
        outfile = self._output_file + '.cppcoverage'
        fh = open(outfile, 'w')
        print >> fh, "%-41s Stmts   Exec  Cover   Missing" % "Name"
        divider = "-" * 71
        print >> fh, divider
        total_statements = total_executed = 0
        for dir, pattern, report in self._sources:
            # Run gcov in a temporary directory so that parallel builds work
            self._run_gcov(dir, pattern, running_dir=t.tmpdir)
            covs = glob.glob(os.path.join(t.tmpdir, "*.gcov"))
            covs.sort()
            for cov in covs:
                ret = self._parse_gcov_file(cov)
                if ret:
                    source, statements, executed, missing = ret
                    self._report_gcov_file(fh, source, statements, executed,
                                           missing)
                    total_statements += statements
                    total_executed += executed
                os.unlink(cov)
        headers = self._header_callcounts.keys()
        headers.sort()
        for header in headers:
            statements, executed, missing \
               = self._summarize_header(self._header_callcounts[header])
            self._report_gcov_file(fh, header, statements, executed,
                                   missing)
            total_statements += statements
            total_executed += executed
        print >> fh, divider
        self._report_gcov_file(fh, 'TOTAL', total_statements,
                               total_executed, [])
        fh.close()
        print >> sys.stderr, \
                 "\nC++ coverage of %s %s written to %s." \
                 % (self._name, self._test_type, outfile)

    def _get_missing_ranges(self, missing):
        ranges = []
        start_range = None
        end_range = None
        def add_range():
            if start_range is not None:
                if end_range == start_range:
                    ranges.append('%d' % start_range)
                else:
                    ranges.append('%d-%d' % (start_range, end_range))
        for line in missing:
            if end_range is not None and end_range == line - 1:
                end_range = line
            else:
                add_range()
                start_range = line
                end_range = line
        add_range()
        return ", ".join(ranges)

    def _parse_gcov_file(self, cov):
        executable_statements = 0
        missing = []
        header_callcounts = None
        for line in open(cov):
            spl = line.split(':', 2)
            calls = spl[0].strip()
            line_number = int(spl[1])
            if line_number == 0:
                if spl[2].startswith('Source:'):
                    source = os.path.normpath(spl[2][7:].strip())
                    header_callcounts = self._match_header(source)
                    if header_callcounts is None \
                       and not self._match_source(source):
                        return None
            else:
                if header_callcounts is not None:
                    self._update_header_callcounts(header_callcounts,
                                                   line_number, calls)
                if calls == '#####':
                    missing.append(line_number)
                if calls != '-':
                    executable_statements += 1
        # Ignore header info for now; we'll accumulate it and display at
        # the end of all source (cpp) files
        if header_callcounts is None:
            return (source, executable_statements,
                    executable_statements - len(missing), missing)

    def _summarize_header(self, header_callcounts):
        executable_statements = 0
        missing = []
        for n, line in enumerate(header_callcounts):
            if line >= 0:
                executable_statements += 1
            if line == 0:
                missing.append(n + 1)
        return (executable_statements, executable_statements - len(missing),
                missing)

    def _make_gcov_for_header(self, source, header_callcounts):
        inf = open(source, 'r')
        fh = open(source.replace('/', '#') + '.gcov', 'w')
        print >> fh, "%9s:%5d:Source:%s" % ('-', 0, source)
        for (n, count) in enumerate(header_callcounts):
            if count == -1:
                c = '-'
            elif count == 0:
                c = '#####'
            else:
                c = str(count)
            fh.write("%9s:%5d:" % (c, n+1))
            fh.write(inf.readline())

    def _update_header_callcounts(self, header_callcounts, line_number, calls):
        # All new lines are marked as non-executable (-1)
        while line_number > len(header_callcounts):
            header_callcounts.append(-1)

        # If this gcov file says the line was executable but not called
        # (call count '#####') then override another gcov file that said it was
        # non-executable (no effect if it was called in another gcov file)
        if calls == '#####':
            if header_callcounts[line_number - 1] == -1:
                header_callcounts[line_number - 1] = 0

        # Non-executable gcov lines (call count of '-') have no effect,
        # since all lines start that way (and this shouldn't override another
        # file that says the line is executable)

        # Executable lines that were called (positive integer call count)
        # override non-executable status, and accumulate
        elif calls != '-':
            count = int(calls)
            if header_callcounts[line_number - 1] == -1:
                header_callcounts[line_number - 1] = count
            else:
                header_callcounts[line_number - 1] += count

    def _match_header(self, fn):
        for dir, patt, report in self._headers:
            if report and fnmatch.fnmatch(fn, os.path.join(dir, patt)):
                if fn not in self._header_callcounts:
                    self._header_callcounts[fn] = []
                return self._header_callcounts[fn]

    def _match_source(self, fn):
        for dir, patt, report in self._sources:
            if report and fnmatch.fnmatch(fn, os.path.join(dir, patt)):
                return True

    def _report_gcov_file(self, fh, source, statements, executed, missing):
        if statements == 0:
            cover = 0
        else:
            cover = float(executed) * 100. / float(statements)

        if len(source) > 40:
            source = "[..]" + source[-36:]

        print >> fh, "%-40s %6d %6d %5d%%   %s" \
              % (source, statements, executed, cover,
                 self._get_missing_ranges(missing))

    def _run_gcov(self, dir, pattern, running_dir=None):
        import subprocess
        # Note that gcov expects to find the .cpp file in the same directory
        # as the coverage info, so annotated output probably won't work
        # with out of tree builds
        cmd = 'gcov -l -p -o %s %s' % (dir, os.path.join(dir, pattern))
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, cwd=running_dir)
        out = p.stdout.read()
        err = p.stderr.read()
        ret = p.wait()
        if ret != 0:
            raise OSError("'%s' failed with code %d, error %s" \
                          % (cmd, ret, err))

    def _make_lcov_info_file(self):
        import subprocess
        def call(args):
            r = subprocess.call(args)
            if r != 0:
                raise OSError("%s failed with exit code %d" % (args[0], r))
        topdir = os.getcwd()
        tmpdir = self._tmpdir.tmpdir

        all_info = os.path.join(tmpdir, 'all.info')
        out_info = os.path.join(self._coverage_dir,
                                '%s.%s.info' % (self._test_type, self._name))
        if self._test_type == 'module':
            # Get all coverage info (includes all dependencies,
            # e.g. boost headers)
            call(['lcov', '-c', '-b', '.',
                  '-d', '%s/build/src/%s.gcda' % (tmpdir,
                                                  self.get_wrap(self._name)),
                  '-d', '%s/modules/%s/src/' % (tmpdir,
                                              self.get_module_dir(self._name)),
                  '-o', all_info])
        else:
            # Get all coverage info (includes all dependencies,
            # e.g. boost headers)
            call(['lcov', '-c', '-b', tmpdir,
                  '-d', '%s/applications/%s/' % (tmpdir, self._name),
                  '-o', all_info])
        self._extract_lcov_info(all_info, out_info)

    def _extract_lcov_info(self, all_info, out_info):
        """Extract only the information on the current module or application
           from the lcov info file (lcov -e and lcov -r can be extremely slow).
           We also take the opportunity to map file names back to their
           source here."""
        topdir = os.getcwd()
        tmpdir = self._tmpdir.tmpdir
        want_files = {}
        for directory, pattern, report in self._sources + self._headers:
            if report:
                for x in glob.glob(os.path.join(topdir, directory, pattern)):
                    want_files[x] = None
        def filter_filename(fname):
            return fname in want_files

        fin = open(all_info)
        fout = open(out_info, 'w')
        record = []
        write_record = False
        for line in fin:
            line = line.replace(tmpdir + '/', topdir + '/')
            if line.startswith('SF:'):
                write_record = filter_filename(line.rstrip('\r\n')[3:])
                if self._test_type == 'module' and write_record:
                    if self._name == 'kernel':
                        line = line.replace('build/include/IMP/',
                                            'modules/%s/include/' % self._name)
                    elif self._name == 'RMF':
                        line = line.replace('build/include/RMF/',
                                            'modules/librmf/include/')
                    else:
                        line = line.replace('build/include/IMP/%s/'% self._name,
                                            'modules/%s/include/' % self._name)
                    # Exclude auto-generated files that *only* live
                    # in build/include
                    write_record = os.path.exists(line.rstrip('\r\n')[3:])
            # Exclude branch coverage information for now. IMP uses macros
            # rather extensively, which yields a lot (~5000 in some cases)
            # of branches for what looks to lcov like a single line. Since
            # lcov looks up branches using a simple linear search, lcov
            # and genhtml become unusably slow (5+ hours vs. 0.3 seconds
            # without branch coverage).
            if not line.startswith('BRDA:'):
                record.append(line)
            if line.startswith('end_of_record'):
                if write_record:
                    fout.writelines(record)
                record = []
        fin.close()
        fout.close()
