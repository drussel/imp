#!/usr/bin/python

import IMP.test
import IMP.atom
import IMP.saxs
import os
import sys
import shutil
import itertools
try:
    import numpy
    import scipy
except:
    numpy = None
    scipy = None

"""
TODO: Add More
- Nup2 (3N7C) : 25014
- Nup53 (3P3D) : 24888
- Hemoglobin S (2HBS) :
- Nup133 (the whole piece): 23904
"""

class SAXSApplicationTest(IMP.test.ApplicationTestCase):

    def setUp(self):
        IMP.test.ApplicationTestCase.setUp(self)
        if numpy is None or scipy is None:
            self.skipTest("could not import numpy or scipy")

    def run_app(self, name, args):
        destname = 'runapp_'+name
        if os.path.isdir(destname):
            return
        args.append('--destdir='+destname)
        args.append('--blimit_fitting=800')
        args.append('--elimit_fitting=800')
        args.append('--allfiles')
        args.append('--outlevel=full')
        args.extend(['--blimit_hessian=80', '--elimit_hessian=80',
                 '--berror','--eerror'])
        #print ' '.join(args)
        #sys.exit()
        p = self.run_python_application('saxs_merge.py',args)
        out, err = p.communicate()
        sys.stderr.write(err)
        self.assertApplicationExitedCleanly(p.returncode, err)
        #do some preliminary tests
        for i in [destname+'/data_merged.dat', destname+'/mean_merged.dat',
                destname+'/summary.txt']:
            self.assertTrue(os.path.exists(i))
        text=open(destname+'/summary.txt').read()
        m = text.find('Classification')
        self.assertIsNotNone(m,
                msg="Classification output not found in summary.txt")

    def uniqify(self, seq):
        #taken from http://stackoverflow.com/a/480227/958110
        seen = set()
        seen_add = seen.add
        return [ x for x in seq if x not in seen and not seen_add(x)]

    def plot_data_overlaid(self, name, d1, d2, datarange, title=None):
        # data overlaid in log scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_data_log_overlaid.png"\n' % name)
        fl.write('set log y\n')
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [%f:%f]\n' % (datarange[2], datarange[3]))
        if title:
            fl.write('set title "%s"\n' % title)
        fl.write('p "%s" u 1:2 w p t "automatic", "%s" u 1:2 w p t "manual"\n' %
            (d1,d2))
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)
        # data overlaid in linear scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_data_lin_overlaid.png"\n' % name)
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [0:%f]\n' % datarange[3])
        if title:
            fl.write('set title "%s"\n' % title)
        fl.write('p "%s" u 1:2 w p t "automatic", "%s" u 1:2 w p t "manual"\n' %
            (d1,d2))
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)

    def plot_data_colored(self, name, d1, datarange, transform=(1,0)):
        # data in linear scale, colored
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [0:%f]\n' % datarange[3])
        fl.write('set output "%s_data_lin_colored.png"\n' % name)
        fl.write('p "%s" u 1:(%f*($2+%f)):5 w p lc var t "automatic"\n' %
                   (d1, transform[0], transform[1]))
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)
        # data shifted in log scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_data_log_colored.png"\n' % name)
        fl.write('set log y\n')
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [%f:%f]\n' % (datarange[2], datarange[3]))
        fl.write('p "%s" u 1:(%f*($2+%f)):5 w p lc var t "automatic"\n' %
                (d1, transform[0],transform[1]))
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)

    def plot_means(self, name, d1, d2, datarange):
        #linear scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_means_lin.png"\n' % name)
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [0:%f]\n' % datarange[3])
        fl.write('p "%s" u 1:2 w l lt 1 t "automatic", '
                 '"" u 1:($2+$3) w l lt 1 not, "" u 1:($2-$3) w l lt 1 not, '
                 '"%s" u 1:2 w l lt 2 t "manual", '
                 '"" u 1:($2+$3) w l lt 2 not, '
                 '"" u 1:($2-$3) w l lt 2 not\n' % (d1,d2))
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)
        #log scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_means_log.png"\n' % name)
        fl.write('set log y\n')
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [%f:%f]\n' % (datarange[2], datarange[3]))
        fl.write('p "%s" u 1:2 w l lt 1 t "automatic", '
                 '"" u 1:($2+$3) w l lt 1 not, "" u 1:($2-$3) w l lt 1 not, '
                 '"%s" u 1:2 w l lt 2 t "manual", '
                 '"" u 1:($2+$3) w l lt 2 not, '
                 '"" u 1:($2-$3) w l lt 2 not\n' % (d1,d2))
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)

    def plot_data_mean(self, name, data, mean, datarange):
        #linear scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_data_lin_mean.png"\n' % name)
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [0:%f]\n' % datarange[3])
        fl.write('p "%s" u 1:2:3 w yerr t "data", "%s" u 1:2 w l t "mean" lw 2,'
                 ' "" u 1:($2+$3) w l lw 2 not, "" u 1:($2-$3) w l lw 2 not\n'  % (data,mean)   )
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)
        #log scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_data_log_mean.png"\n' % name)
        fl.write('set log y\n')
        fl.write('set xrange [%f:%f]\n' % (datarange[0], datarange[1]))
        fl.write('set yrange [%f:%f]\n' % (datarange[2], datarange[3]))
        fl.write('p "%s" u 1:2:3 w yerr t "data", "%s" u 1:2 w l t "mean" lw 2,'
                 ' "" u 1:($2+$3) w l lw 2 not, "" u 1:($2-$3) w l lw 2 not\n'  % (data,mean)   )
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)

    def plot_guinier(self, name, data, mean, Rg, qRgmax=1.3):
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_guinier.png"\n' % name)
        fl.write('set xrange [0:%f]\n' % (qRgmax/Rg))
        fl.write('p "%s" u ($1*$1):(log($2)):($3/$2) w yerr t "data", '
                 '  "%s" u ($1*$1):(log($2)) w l t "mean", '
                 '"" u ($1*$1):(log($2)+$3/$2) w l not, '
                 '"" u ($1*$1):(log($2)-$3/$2) w l not\n' % (data,mean))
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)

    def plot_inputs(self, name, inputs):
        #linear scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_inputs_lin.png"\n' % name)
        fl.write('p ')
        for i,inp in enumerate(inputs):
            fl.write('"%s" u 1:(%d+$2):3 w yerr t "%s"'
                    % (inp, i*10, os.path.basename(inp)))
            if i < len(inputs)-1:
                fl.write(', \\\n')
            else:
                fl.write('\n')
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)
        #log scale
        fl=open('Cpgnuplot'+name,'w')
        fl.write('set term png\n')
        fl.write('set output "%s_inputs_log.png"\n' % name)
        fl.write('set log y\n')
        fl.write('p ')
        for i,inp in enumerate(inputs):
            fl.write('"%s" u 1:(%d*$2):(%d*$3) w yerr t "%s"'
                    % (inp, i*10, i*10, os.path.basename(inp)))
            if i < len(inputs)-1:
                fl.write(', \\\n')
            else:
                fl.write('\n')
        fl.close()
        os.system('gnuplot Cpgnuplot'+name)

    def chisquare(self, fla, flb):
        #read first 3 columns
        da=[map(float, i.split()[:3]) for i in open(fla).readlines()]
        db=[map(float, i.split()[:3]) for i in open(flb).readlines()]
        #check that data has common q values, removing the first ones if different
        qa = self.uniqify(map(lambda a:a[0], da))
        qb = self.uniqify(map(lambda a:a[0], db))
        #print qa[0],qb[0]
        if qa[0] < qb[0]:
            i=0
            while i+1 < len(qa) and abs(qa[i] - qb[0]) > 1e-5:
                #print "   qa[%d]=%f" % (i,qa[i])
                i += 1
            qa = qa[i:]
            #print "qa[0] -> %f (%d)" % (qa[0],len(qa))
        else:
            i=0
            while i+1 < len(qb) and abs(qa[0] - qb[i]) > 1e-5:
                #print "   qb[%d]=%f" % (i,qb[i])
                i += 1
            qb = qb[i:]
            #print "qb[0] -> %f (%d)" % (qb[0],len(qb))
        if len(qa) == 0 or len(qb) == 0:
            self.fail("datasets do not have common q values!")
        #project back to data
        qa.reverse()
        qb.reverse()
        da.reverse()
        db.reverse()
        nda = []
        ndb = []
        while qa and qb:
            #find next q value that is common in both datasets
            qva = qa.pop()
            qvb = qb.pop()
            while qa and qb:
                #print qva,qvb,
                if abs(qva-qvb) < 1e-5:
                    #print "equal"
                    break
                elif qva < qvb:
                    qva = qa.pop()
                else:
                    qvb = qb.pop()
                #print "different"
            if abs(qva-qvb)>1e-5:
                break
            #find it in the datasets
            qval = qva
            #print "new qval",qval
            while da:
                if da[-1][0] < qval - 1e-5:
                    da.pop()
                    #print "  skipped da",da.pop()
                else:
                    break
            while da:
                if da[-1][0] > qval + 1e-5:
                    break
                else:
                    nda.append(da.pop())
                    #print "  appended",nda[-1]
            #same for db
            while db:
                if db[-1][0] < qval - 1e-5:
                    db.pop()
                    #print "  skipped db",db.pop()
                else:
                    break
            while db:
                if db[-1][0] > qval + 1e-5:
                    break
                else:
                    ndb.append(db.pop())
                    #print "  appended",ndb[-1]
            #same for db
        #compute normalized chisquare.
        #need to find better statistic
        #Multiple datapoints at same q value are downweighted.
        #each chunk is a set of values at the same q
        chi = []
        while nda and ndb:
            #get chunk
            qval = nda[-1][0]
            chunka = []
            while nda and abs(nda[-1][0] - qval) < 1e-5:
                chunka.append(nda.pop()[1:])
            chunkb = []
            while ndb and abs(ndb[-1][0] - qval) < 1e-5:
                chunkb.append(ndb.pop()[1:])
            #compute contributions
            contribs = []
            for ia,sa in chunka:
                for ib,sb in chunkb:
                    contribs.append((ia-ib)**2/(sa**2+sb**2))
            #downweight and add them
            chi.append(sum(contribs)/float(len(contribs)))
        if len(chi) < 50:
            print "warning: chisquare computed on only %d points!" % len(chi)
        return sum(chi)/len(chi)

    def get_transform(self, fname):
        data=open(fname).readlines()
        gamma = float([i for i in data if 'gamma' in i][0].split()[2])
        offset = [i for i in data if 'offset :' in i]
        if offset:
            offset=float(offset[0].split()[2])
            return gamma,offset
        else:
            return gamma,0

    def get_pdb_data(self, pdb, automerge, manualmerge):
        """get chi and radius of gyration of pdb"""
        m = IMP.Model()
        mp =IMP.atom.read_pdb(pdb, m,
                      IMP.atom.NonWaterNonHydrogenPDBSelector())
        particles = IMP.atom.get_by_type(mp, IMP.atom.ATOM_TYPE)

        # compute surface accessibility
        s = IMP.saxs.SolventAccessibleSurface()
        surface_area = s.get_solvent_accessibility(IMP.core.XYZRs(particles))

        # compute the theoretical profile
        model_profile = IMP.saxs.Profile()
        model_profile.calculate_profile_partial(particles, surface_area)

        # fit the data
        exp_profile = IMP.saxs.Profile(automerge)
        saxs_score = IMP.saxs.ProfileFitterChi(exp_profile)
        chi = (saxs_score.fit_profile(model_profile)).get_chi()
        Rg = model_profile.radius_of_gyration()

        # fit manual merge
        exp_profile = IMP.saxs.Profile(manualmerge)
        saxs_score = IMP.saxs.ProfileFitterChi(exp_profile)
        mchi = (saxs_score.fit_profile(model_profile)).get_chi()

        return chi,mchi,Rg

    def get_guinier_Rg(self, profile):
        #guinier
        exp_profile = IMP.saxs.Profile(profile)
        gRg = exp_profile.radius_of_gyration()
        return gRg

    def get_GPI_Rg(self, summary):
        #GPI
        lines=open(summary).readlines()
        lines = [ float(i.split()[2]) for i in lines if " Rg " in i and not
                'matrix' in i ]
        return lines

    def run_results(self, name, manual_merge, inputs, pdb=None,
            extra_args=None):
        #rescale and fit the two curves
        destdir='compapp_'+name
        if not os.path.isdir(destdir):
            args = ['--destdir='+destdir,
                 '--blimit_fitting=800', '--elimit_fitting=800',
                 '--blimit_hessian=80', '--elimit_hessian=80',
                 '--berror','--eerror',
                 '--stop=rescaling', '--postpone_cleanup',
                 #'--lambdamin=0.05',
                 '--npoints=-1', '--allfiles', '--outlevel=full',
                 'runapp_'+name+'/data_merged.dat', manual_merge]
            #print ' '.join(args)
            #sys.exit()
            p = self.run_python_application('saxs_merge.py', args)
            out, err = p.communicate()
            sys.stderr.write(err)
            self.assertApplicationExitedCleanly(p.returncode, err)
        #compute chi2 of data
        datachi = self.chisquare(destdir+'/data_data_merged.dat',
                destdir+'/data_'+os.path.basename(manual_merge))
        #compute chi2 of fits
        fitchi = self.chisquare(destdir+'/mean_data_merged.dat',
                destdir+'/mean_'+os.path.basename(manual_merge))
        #compute chi2 to pdb structure using foxs
        if pdb:
            pdbchi, mpdbchi, pdbRg = \
                    self.get_pdb_data(pdb, destdir+'/mean_data_merged.dat',
                        destdir+'/mean_'+os.path.basename(manual_merge))
        else:
            pdbchi = None
            mpdbchi = None
            pdbRg = None
        #radius of gyration
        guinierRg = self.get_guinier_Rg(destdir+'/mean_data_merged.dat')
        mguinierRg = self.get_guinier_Rg(destdir+'/mean_'
                                      +os.path.basename(manual_merge))
        Rg, mRg = self.get_GPI_Rg(destdir+'/summary.txt')
        #get proper bounds
        points=map(lambda a:map(float,a.split()[:2]),
                open(manual_merge).readlines())
        xmin = 0
        xmax = max([i[0] for i in points if len(i) >= 2])*1.2
        ymin = min([i[1] for i in points if len(i) >= 2])*0.8
        if ymin <=0: ymin = 0.01
        ymax = max([i[1] for i in points if len(i) >= 2])*1.2
        datarange = (xmin,xmax,ymin,ymax)
        #plot data
        self.plot_data_overlaid(name, destdir+'/data_data_merged.dat',
                destdir+'/data_'+os.path.basename(manual_merge), datarange)
        self.plot_data_colored(name,
                'runapp_'+name+'/data_merged.dat', datarange,
                transform = self.get_transform(destdir+'/summary.txt') )
        #plot mean
        self.plot_means(name, destdir+'/mean_data_merged.dat',
                destdir+'/mean_'+os.path.basename(manual_merge), datarange)
        self.plot_data_mean(name, destdir+'/data_data_merged.dat',
                destdir+'/mean_data_merged.dat', datarange)
        #guinier plot
        self.plot_guinier(name, destdir+'/data_data_merged.dat',
                destdir+'/mean_data_merged.dat', Rg)
        #plot all curves
        self.plot_inputs(name, inputs)
        return name,datachi,fitchi,Rg,guinierRg,mRg,mguinierRg,\
                pdbRg,pdbchi,mpdbchi

    def make_stats(self, paramnum, inputnum, ret):
        #name,datachi,fitchi,Rg,guinierRg,mRg,mguinierRg,\
        #        pdbRg,pdbchi,mpdbchi = ret
        ret = [paramnum,inputnum]+list(ret)
        return ' '.join(['%s' % i for i in ret])

def create_test(paramnum, paramname, params, inputnum, inputname, inputs,
        mergename, pdb=None):
    """params and inputs are lists of strings
       paramname and inputname are strings
       mergename is the path to manual merge file
       pdbname is the path to pdb file if available, else None
    """
    outname=inputname+'_'+paramname
    def testcase(self):
        inp=map(self.get_input_file_name, inputs)
        merge = self.get_input_file_name(mergename)
        if pdb:
            pdbname = self.get_input_file_name(pdb)
        else:
            pdbname = None
        self.run_app(outname, params+inp)
        ret = self.run_results(outname, merge, inp, pdb=pdbname)
        print self.make_stats(paramnum, inputnum, ret)
    return testcase

def create_datasets():
    class dataset:
        name = None
        inputs = None
        mergename = None
        pdb = None

    datasets=[]

    #Nup116
    d=dataset()
    d.name='Nup116'
    d.inputs = ['Nup116/25043_01C_S059_0_01.sub',
                'Nup116/25043_01B_S057_0_01.sub',
                'Nup116/25043_01D_S061_0_01.sub',
                'Nup116/25043_01E_S063_0_01.sub',
                'Nup116/25043_01F_S065_0_01.sub']
    d.mergename = 'Nup116/25043_manual_merge.dat'
    d.pdb = 'Nup116/3nf5_model.pdb'
    datasets.append(d)

    #Nup192
    d=dataset()
    d.name='Nup192'
    d.inputs = ['Nup192/Nup192_01B_S014_0_01.sub',
                'Nup192/Nup192_01C_S016_0_01.sub',
                'Nup192/Nup192_01D_S018_0_01.sub',
                'Nup192/Nup192_01E_S020_0_02.sub',
                'Nup192/Nup192_01F_S022_0_02.sub']
    d.mergename = 'Nup192/Nup192_merge.dat'
    datasets.append(d)

    #Nup145
    d = dataset()
    d.name='Nup145'
    d.inputs = ['Nup145/23923_A3_2.mccd.dat',
                'Nup145/23923_A2_2.mccd.dat',
                'Nup145/23923_A4_2.mccd.dat',
                'Nup145/23923_A5_2.mccd.dat']
    d.mergename = 'Nup145/23923_merge.dat'
    d.pdb = 'Nup145/3kep_model.pdb'
    datasets.append(d)

    #Nup133
    d=dataset()
    d.name = 'Nup133'
    d.inputs = ['Nup133/23922_G4_2.mccd.dat',
                'Nup133/23922_G2_2.mccd.dat',
                'Nup133/23922_G3_2.mccd.dat',
                'Nup133/23922_G5_2.mccd.dat']
    d.mergename = 'Nup133/23922_merge.dat'
    d.pdb = 'Nup133/3kfo_model.pdb'
    datasets.append(d)

    #mo_lair1s
    d=dataset()
    d.name = 'mo_lair1s'
    d.inputs = ['mo_lair1s/mo_lig_apo_02B_S012_0_02.sub',
                'mo_lair1s/mo_lig_apo_02C_S014_0_02.sub',
                'mo_lair1s/mo_lig_apo_02D_S016_0_02.sub',
                'mo_lair1s/mo_lig_apo_02E_S018_0_02.sub',
                'mo_lair1s/mo_lig_apo_02F_S020_0_02.sub']
    d.mergename = 'mo_lair1s/mo_lair1s_merged.dat'
    datasets.append(d)

    #mo_lair1_ecd
    d = dataset()
    d.name = 'mo_lair1_ecd'
    d.inputs = ['mo_lair1_ecd/mo_lecd_apo_01B_S036_0_02.sub',
                'mo_lair1_ecd/mo_lecd_apo_01C_S038_0_02.sub',
                'mo_lair1_ecd/mo_lecd_apo_01D_S040_0_02.sub',
                'mo_lair1_ecd/mo_lecd_apo_01E_S042_0_02.sub',
                'mo_lair1_ecd/mo_lecd_apo_01F_S044_0_02.sub']
    d.mergename = 'mo_lair1_ecd/mo_lair1_ecd_merged.dat'
    datasets.append(d)

    #amelogenin_pH56
    d = dataset()
    d.name = 'amelogenin_pH56'
    d.inputs = ['amelogenin_pH56/3_pH56_lCaP_01C_S014_0_01.sub',
                'amelogenin_pH56/3_pH56_lCaP_01B_S012_0_01.sub',
                'amelogenin_pH56/3_pH56_lCaP_01D_S016_0_01.sub',
                'amelogenin_pH56/3_pH56_lCaP_01E_S018_0_01.sub']
    d.mergename = 'amelogenin_pH56/amelogenin_pH56_merged.dat'
    datasets.append(d)

    #amelogenin_pH75
    d = dataset()
    d.name = 'amelogenin_pH75'
    d.inputs = ['amelogenin_pH75/5_pH75_lCaP_01f_01C_S014_0_01.sub',
                'amelogenin_pH75/5_pH75_lCaP_01f_01B_S012_0_01.sub',
                'amelogenin_pH75/5_pH75_lCaP_01f_01D_S016_0_01.sub',
                'amelogenin_pH75/5_pH75_lCaP_01f_01E_S018_0_01.sub']
    d.mergename = 'amelogenin_pH75/amelogenin_pH75_merged.dat'
    datasets.append(d)

    #Y1
    d=dataset()
    d.name = 'Y1'
    d.inputs = ['Y1/in1.dat',
                'Y1/in2.dat',
                'Y1/in3.dat']
    d.mergename = 'Y1/Y1_merge.dat'
    datasets.append(d)

    #Y2
    d=dataset()
    d.name = 'Y2'
    d.inputs = ['Y2/in1.dat',
                'Y2/in2.dat',
                'Y2/in3.dat']
    d.mergename = 'Y2/Y2_merge.dat'
    datasets.append(d)

    #Aldolase
    d=dataset()
    d.name = 'Aldolase'
    d.inputs = ['Aldolase/in1.dat',
                'Aldolase/in2.dat',
                'Aldolase/in3.dat',
                'Aldolase/in4.dat']
    d.mergename = 'Aldolase/merge.dat'
    datasets.append(d)

    #Anhydrase
    d=dataset()
    d.name = 'Anhydrase'
    d.inputs = ['Anhydrase/in1.dat',
                'Anhydrase/in2.dat',
                'Anhydrase/in3.dat',
                'Anhydrase/in4.dat']
    d.mergename = 'Anhydrase/merge.dat'
    d.pdb = 'Anhydrase/3ks3_model.pdb'
    datasets.append(d)

    #Conalbumin
    d=dataset()
    d.name = 'Conalbumin'
    d.inputs = ['Conalbumin/in1.dat',
                'Conalbumin/in2.dat',
                'Conalbumin/in3.dat',
                'Conalbumin/in4.dat']
    d.mergename = 'Conalbumin/merge.dat'
    d.pdb = 'Conalbumin/1aiv_model.pdb'
    datasets.append(d)

    #Ferritin
    d=dataset()
    d.name = 'Ferritin'
    d.inputs = ['Ferritin/in1.dat',
                'Ferritin/in2.dat',
                'Ferritin/in3.dat',
                'Ferritin/in4.dat']
    d.mergename = 'Ferritin/merge.dat'
    datasets.append(d)

    #Ribonuclease
    d=dataset()
    d.name = 'Ribonuclease'
    d.inputs = ['Ribonuclease/in1.dat',
                'Ribonuclease/in2.dat',
                'Ribonuclease/in3.dat',
                'Ribonuclease/in4.dat',
                'Ribonuclease/in5.dat']
    d.mergename = 'Ribonuclease/merge.dat'
    datasets.append(d)

    #Thyroglobulin
    d=dataset()
    d.name = 'Thyroglobulin'
    d.inputs = ['Thyroglobulin/in1.dat',
                'Thyroglobulin/in2.dat',
                'Thyroglobulin/in3.dat',
                'Thyroglobulin/in4.dat',
                'Thyroglobulin/in5.dat',
                'Thyroglobulin/in6.dat']
    d.mergename = 'Thyroglobulin/merge.dat'
    datasets.append(d)

    return datasets

datasets=create_datasets()

def create_params_list():
    params=[]
    params.append(['--cmodel=normal'])
    params.append(['--aalpha=1e-4', '--cmodel=normal'])
    params.append(['--postpone_cleanup', '--cmodel=normal'])
    params.append(['--cmodel=normal-offset'])
    params.append(['--cmodel=normal', '--bcomp', '--blimit_hessian=100',
                '--boptimize=Full'])
    return params

def create_params_shuffle():
    items=[]
    items.append(['--aalpha=1e-5',
                  '--aalpha=1e-6',
                  '--aalpha=1e-7',
                  '--aalpha=1e-8',
                  '--aalpha=1e-9'])
    items.append(['--bcomp --ecomp --boptimize=Full --eoptimize=Full',
                   '--boptimize=Flat --eoptimize=Generalized',
                   '--boptimize=Simple --eoptimize=Generalized'
                   ])
    items.append(['--cmodel=normal'])
    params = []
    for i in itertools.product(*items):
        tmp=[]
        for k in i:
            tmp.extend(k.split())
        params.append(tmp)
    return items,params

#params = create_params_list()
items,params = create_params_shuffle()

###### test creation
for k, param in enumerate(params):
    for l, dset in enumerate(datasets):
        test_method = create_test(k, str(k), param,
                                  l, dset.name, dset.inputs,
                                  dset.mergename, dset.pdb)
        test_method.__name__ = 'test_case_%d_%d' % (l,k)
        setattr(SAXSApplicationTest, test_method.__name__, test_method)

def print_corresp():
    print "Datasets"
    for k,p in enumerate(datasets):
        print "%s : %s" % (k,p.name)
    print
    print "Params"
    for k,p in enumerate(params):
        print "%s : %s" % (k, ' '.join(p))

def print_params():
    for k,p in enumerate(params):
        par=' '.join(p)
        ind = [k]
        for item in items:
            matchlen=-1
            for j,it in enumerate(item):
                if it in par:
                    if matchlen==-1:
                        ind.append(j)
                    else:
                        if matchlen < len(it):
                            ind[-1]=j
                    matchlen=len(it)
            if matchlen==-1:
                raise ValueError
        print " ".join(["%d" % i for i in ind])


if __name__ == '__main__':
    IMP.test.main()
    #print_corresp()
    #print_params()
