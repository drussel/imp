#!/bin/sh

# Script to get SVN HEAD and make a .tar.gz on a shared disk, for use by
# autobuild scripts on our build machines.
#
# Should be run on the SVN server (or some other system that can access SVN
# readonly without a password) from a crontab, e.g.
#
# 10 1 * * * /cowbell1/home/ben/imp/tools/auto-build.sh
#
# In this case, the build user is in the apache group, and so has readonly
# access to the repositories.

VER=SVN
IMPSVNDIR=file:///cowbell1/svn/imp/trunk/

TMPDIR=/var/tmp/imp-build-$$
IMPTOP=/salilab/diva1/home/imp

rm -rf ${TMPDIR}
mkdir ${TMPDIR}
cd ${TMPDIR}

# Get top-most revision number (must be a nicer way of doing this?)
rev=$(svn log -q --limit 1 ${IMPSVNDIR} |grep '^r' | cut -f 1 -d' ')

# Get date and revision-specific install directories
IMPINSTALL=${IMPTOP}/`date -d '8 hours' "+%Y%m%d"`-${rev}
IMPVERSION="SVN.${rev}"
IMPSRCTGZ=${IMPINSTALL}/build/sources/imp-${IMPVERSION}.tar.gz
rm -rf ${IMPINSTALL}
mkdir -p ${IMPINSTALL}/build/sources ${IMPINSTALL}/build/logs

# Make link so build system can find the install location
rm -f ${IMPTOP}/.SVN-new
ln -s ${IMPINSTALL} ${IMPTOP}/.SVN-new

# Get IMP code from SVN
svn export -q -${rev} ${IMPSVNDIR} imp

# Put version number, date and revision into relevant files
DATE=`date +'%Y/%m/%d'`
(cd imp/doc/doxygen && sed -e "s#^PROJECT_NUMBER.*#PROJECT_NUMBER = ${VER}, ${DATE}, ${rev}#" < doxygen.conf-in > .dox && mv .dox doxygen.conf-in)
perl -pi -e "s/version=[\"']SVN[\"']/version='SVN.${rev}'/" imp/modules/*/SConscript imp/applications/*/SConscript imp/biological_systems/*/SConscript

# Write out a version file
verfile="${IMPINSTALL}/build/imp-version"
mkdir -p "${IMPINSTALL}/build"
echo "${rev}" > $verfile

# Write out list of all modules
modfile="${IMPINSTALL}/build/imp-modules"
python <<END
import glob
def Import(var): pass
def SConscript(var): pass
env = {'local':True} 
for path in ('imp/modules/', 'imp/applications/',
             'imp/biological_systems/'):
    def Glob(pattern):
        lp = len(path)
        return [x[lp:] for x in glob.glob(path + pattern)]
    exec(open(path + 'SConscript').read())
    
f = open('$modfile', 'w')
for m in modules:
    # Hack for librmf->RMF renaming
    if m == 'librmf':
        m = 'RMF'
    print >> f, "module\t" + m
for m in applications:
    print >> f, "application\t" + m
for m in systems:
    print >> f, "system\t" + m
END

# Write out a tarball:
mv imp imp-${IMPVERSION} && tar -czf ${IMPSRCTGZ} imp-${IMPVERSION}

# Cleanup
cd /
rm -rf ${TMPDIR}
