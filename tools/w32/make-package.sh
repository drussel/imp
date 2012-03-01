#!/bin/sh

# Make a Win32 installer

# First run the following to install files:
# scons -j3 destdir=w32-inst wine=true install
#
# This will only build Python wrappers for the 'default' Python version. To
# add wrappers for another version, use something like
# scons -j3 destdir=w32-inst wine=true pyextdir=/pylib/${PYVER} \
#           pythoninclude=/usr/lib/w32comp/w32python/${PYVER}/include \
#           libpath=/usr/lib/w32comp/w32python/${PYVER}/lib/ \
#           w32-inst/pylib/${PYVER}/
# where $PYVER is the Python version (e.g. 2.4).
#
# Then run (still in the top-level IMP directory)
# tools/w32/make-package.sh <version>
#
# where <version> is the IMP version number, e.g. 1.0

if [ $# -ne 1 ]; then
  echo "Usage: $0 <IMP version>"
  exit 1
fi

VER=$1
ROOT=w32-inst

# Put things in more w32-like arrangement
mv ${ROOT}/usr/include ${ROOT}/usr/bin ${ROOT} || exit 1
mv ${ROOT}/usr/lib/* ${ROOT}/bin || exit 1
rmdir ${ROOT}/usr/lib || exit 1

mv ${ROOT}/usr/share/imp ${ROOT}/data || exit 1
mv ${ROOT}/usr/share/doc/imp/examples ${ROOT} || exit 1

rmdir ${ROOT}/usr/share/doc/imp || exit 1
rmdir ${ROOT}/usr/share/doc || exit 1
rmdir ${ROOT}/usr/share || exit 1
rmdir ${ROOT}/usr || exit 1

# Add Windows-specific README
cp tools/w32/README.txt ${ROOT} || exit 1

# Move default Python extensions (2.6) to Windows location
mv ${ROOT}/bin/python2.6/site-packages ${ROOT}/python || exit 1
rmdir ${ROOT}/bin/python2.6 || exit 1

# Patch IMP/__init__.py so it can find Python version-specific extensions
# and the IMP DLLs
patch -d ${ROOT}/python/IMP -p1 < tools/w32/python-search-path.patch || exit 1

# Make Python version-specific directories for extensions (.pyd)
for PYVER in 2.3 2.4 2.5 2.6 2.7; do
  mkdir ${ROOT}/python/python${PYVER} || exit 1
done
mv ${ROOT}/python/*.pyd ${ROOT}/python/python2.6 || exit 1
for PYVER in 2.3 2.4 2.5 2.7; do
  mv ${ROOT}/pylib/${PYVER}/*.pyd ${ROOT}/python/python${PYVER} || exit 1
  rmdir ${ROOT}/pylib/${PYVER} || exit 1
done
rmdir ${ROOT}/pylib || exit 1

# Add redist MSVC runtime DLLs
DLLSRC=/usr/lib/w32comp/windows/system
cp ${DLLSRC}/msvc*100.dll ${ROOT}/bin || exit 1

# Add other DLL dependencies
cp ${DLLSRC}/hdf5dll.dll ${DLLSRC}/libgsl.dll ${DLLSRC}/libgslcblas.dll \
   ${DLLSRC}/boost_filesystem-vc100-mt-1_44.dll \
   ${DLLSRC}/boost_program_options-vc100-mt-1_44.dll \
   ${DLLSRC}/boost_system-vc100-mt-1_44.dll \
   ${DLLSRC}/libfftw3-3.dll \
   ${DLLSRC}/opencv_core220.dll ${DLLSRC}/opencv_highgui220.dll \
   ${DLLSRC}/opencv_ffmpeg220.dll \
   ${DLLSRC}/opencv_imgproc220.dll ${ROOT}/bin || exit 1

# Check all installed binaries for DLL dependencies, to make sure we
# didn't miss any
# We should really parse the PE files properly rather than using 'strings' here!
strings ${ROOT}/bin/*.exe ${ROOT}/bin/*.pyd ${ROOT}/bin/*.dll \
        | grep -i '\.dll' | sort -u | tr '[:upper:]' '[:lower:]' > w32.deps
(cd ${ROOT}/bin && ls *.dll) | tr '[:upper:]' '[:lower:]' > w32.dlls

# Add standard Windows DLLs
echo "kernel32.dll" >> w32.dlls
echo "advapi32.dll" >> w32.dlls
echo "avicap32.dll" >> w32.dlls
echo "avifil32.dll" >> w32.dlls
echo "comctl32.dll" >> w32.dlls
echo "gdi32.dll" >> w32.dlls
echo "msvcrt.dll" >> w32.dlls
echo "msvfw32.dll" >> w32.dlls
echo "ole32.dll" >> w32.dlls
echo "user32.dll" >> w32.dlls
echo "wsock32.dll" >> w32.dlls
echo "ws2_32.dll" >> w32.dlls

# Add DLLs of our prerequisites (Python)
for PYVER in 23 24 25 26 27; do
  echo "python${PYVER}.dll" >> w32.dlls
done

if grep -v -f w32.dlls w32.deps > w32.unmet_deps; then
  echo "The following non-standard libraries are linked against, and were"
  echo "not bundled:"
  echo
  cat w32.unmet_deps
  exit 1
fi

rm -f w32.dlls w32.deps w32.unmet_deps

tools/w32/gen-w32instlist w32-inst > w32files.tmp || exit 1
sed -e '/\.pyc"$/d' < w32files.tmp > w32files.install || exit 1
tac w32files.tmp | sed -e 's/File "w32-inst\\/Delete "$INSTDIR\\/' -e 's/^SetOutPath/RMDir/' > w32files.uninstall || exit 1
makensis -DVERSION=${VER} -NOCD tools/w32/w32-install.nsi || exit 1
