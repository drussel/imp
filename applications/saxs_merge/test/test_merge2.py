#!/usr/bin/python

import sys,os
import tempfile
import copy

import IMP.test
import IMP.isd

class MockArgs:
    def __init__(self,**kwargs):
        for k,v in kwargs.iteritems():
            setattr(self, k, v)

class MockGP:
    def __init__(self,a,b):
        self.a=a
        self.b=b

    def get_posterior_mean(self,q):
        return self.a*((q[0]-self.b)**2+1)

    def get_posterior_covariance(self,q,r):
        return self.a*(q[0]+1)

class MockGP2:
    def __init__(self,a,b):
        self.a=a
        self.b=b

    def get_posterior_mean(self,q):
        if q[0] > self.a:
            return self.b
        else:
            return 1

    def get_posterior_covariance(self,q,r):
        return self.a*(q[0]+1)

class SAXSProfileTestThree(IMP.test.ApplicationTestCase):

    def setUp(self):
        IMP.test.ApplicationTestCase.setUp(self)
        try:
            import scipy
            import numpy
        except ImportError:
            self.skipTest("could not import scipy and numpy")
        merge = self.import_python_application('saxs_merge.py')
        self.SAXSProfile = merge.SAXSProfile
        self.merge = merge

    def set_interpolant(self, profile, a, b, interpolant=MockGP):
        m=IMP.Model()
        s=IMP.isd.Scale.setup_particle(IMP.Particle(m),3.0)
        gp=interpolant(a,b)
        profile.set_interpolant(gp, {'sigma':s}, m)
        return gp

    def test_rescaling_normal(self):
        """test rescaling of two perfectly agreeing functions"""
        #data just used to set the q-range
        data=[[0,1,1,True],[1,10,1,True],[2,1,1,True],[3,10,1,True]]
        p1=self.SAXSProfile()
        p1.new_flag('agood',bool)
        p1.add_data(data)
        gp1=self.set_interpolant(p1,1,0)
        p2=self.SAXSProfile()
        p2.new_flag('agood',bool)
        p2.add_data(data)
        gp2=self.set_interpolant(p2,10,0)
        p3=self.SAXSProfile()
        p3.new_flag('agood',bool)
        p3.add_data(data)
        gp3=self.set_interpolant(p3,30,0)
        args=MockArgs(verbose=0, cnormal=True, cnpoints=100, creference='last')
        self.assertEqual(p1.get_gamma(),1)
        self.assertEqual(p2.get_gamma(),1)
        self.assertEqual(p3.get_gamma(),1)
        self.merge.rescaling([p1,p2,p3],args)
        self.assertTrue('cgood' in p1.get_flag_names())
        self.assertTrue('cgood' in p2.get_flag_names())
        self.assertTrue('cgood' in p3.get_flag_names())
        self.assertEqual(p1.get_data(colwise=True)['cgood'],[True]*len(data))
        self.assertEqual(p2.get_data(colwise=True)['cgood'],[True]*len(data))
        self.assertEqual(p3.get_data(colwise=True)['cgood'],[True]*len(data))
        self.assertAlmostEqual(p1.get_gamma(),30)
        self.assertAlmostEqual(p2.get_gamma(),3)
        self.assertAlmostEqual(p3.get_gamma(),1)

    def test_rescaling_lognormal(self):
        """test rescaling of two perfectly agreeing functions"""
        #data just used to set the q-range
        data=[[0,1,1,True],[1,10,1,True],[2,1,1,True],[3,10,1,True]]
        p1=self.SAXSProfile()
        p1.new_flag('agood',bool)
        p1.add_data(data)
        gp1=self.set_interpolant(p1,1,0)
        p2=self.SAXSProfile()
        p2.new_flag('agood',bool)
        p2.add_data(data)
        gp2=self.set_interpolant(p2,10,0)
        p3=self.SAXSProfile()
        p3.new_flag('agood',bool)
        p3.add_data(data)
        gp3=self.set_interpolant(p3,30,0)
        args=MockArgs(verbose=0, cnormal=False, cnpoints=100, creference='last')
        self.assertEqual(p1.get_gamma(),1)
        self.assertEqual(p2.get_gamma(),1)
        self.assertEqual(p3.get_gamma(),1)
        self.merge.rescaling([p1,p2,p3],args)
        self.assertTrue('cgood' in p1.get_flag_names())
        self.assertTrue('cgood' in p2.get_flag_names())
        self.assertTrue('cgood' in p3.get_flag_names())
        self.assertEqual(p1.get_data(colwise=True)['cgood'],[True]*len(data))
        self.assertEqual(p2.get_data(colwise=True)['cgood'],[True]*len(data))
        self.assertEqual(p3.get_data(colwise=True)['cgood'],[True]*len(data))
        self.assertAlmostEqual(p1.get_gamma(),30)
        self.assertAlmostEqual(p2.get_gamma(),3)
        self.assertAlmostEqual(p3.get_gamma(),1)

    def test_classification(self):
        """simple classification test"""
        data=[[0,1,1,True],[1,10,1,True],[2,1,1,True],[3,10,1,True]]
        data2=[[.1,1,1,False],[1.1,0,1,True],[2.1,1,1,True],[3,10,1,True],
                [4,10,1,True]]
        #prepare p1
        p1=self.SAXSProfile()
        p1.new_flag('agood',bool)
        p1.add_data(data[:3])
        p1.set_filename('test1')
        gp1=self.set_interpolant(p1,1.5,10,MockGP2)
        self.merge.create_intervals_from_data(p1,'agood')
        #prepare p2
        p2=self.SAXSProfile()
        p2.new_flag('agood',bool)
        p2.add_data(data)
        p2.set_filename('test2')
        gp2=self.set_interpolant(p2,2.5,10,MockGP2)
        self.merge.create_intervals_from_data(p2,'agood')
        #prepare p3
        p3=self.SAXSProfile()
        p3.new_flag('agood',bool)
        p3.add_data(data2)
        p3.set_filename('test3')
        gp3=self.set_interpolant(p3,2.5,10,MockGP2)
        self.merge.create_intervals_from_data(p3,'agood')
        #run classification
        args=MockArgs(verbose=0, dalpha=0.05)
        self.merge.classification([p1,p2,p3],args)
        #p1
        self.assertTrue(
                set(p1.get_flag_names()).issuperset(
                    set(['drefnum','drefname','dgood','dselfref','dpvalue'])))
        test1=p1.get_data(colwise=True)
        self.assertEqual(test1['drefnum'],[0,0,0])
        self.assertEqual(test1['drefname'],['test1']*3)
        self.assertEqual(test1['dgood'], [True]*3)
        self.assertEqual(test1['dselfref'],[True]*3)
        #p2
        self.assertTrue(
                set(p2.get_flag_names()).issuperset(
                    set(['drefnum','drefname','dgood','dselfref','dpvalue'])))
        test2=p2.get_data(colwise=True)
        self.assertEqual(test2['drefnum'],[0,0,0,1])
        self.assertEqual(test2['drefname'],['test1']*3+['test2'])
        self.assertEqual(test2['dgood'], [True,True,False,True])
        self.assertEqual(test2['dselfref'],[False,False,False,True])
        #p3
        self.assertTrue(
                set(p2.get_flag_names()).issuperset(
                    set(['drefnum','drefname','dgood','dselfref','dpvalue'])))
        test3=p3.get_data(colwise=True)
        self.assertEqual(test3['drefnum'],[None,0,1,1,2])
        self.assertEqual(test3['drefname'],[None,'test1','test2',
                                            'test2','test3'])
        self.assertEqual(test3['dgood'], [None,True,True,True,True])
        self.assertEqual(test3['dselfref'],[None,False,False,False,True])

    def test_merging(self):
        """simple merge test without the fitting part"""
        data=[[0,1,1,True],[1,10,1,True],[2,1,1,True],[3,10,1,True]]
        data2=[[.1,1,1,False],[1.1,0,1,True],[2.1,1,1,True],[3,10,1,True],
                [4,10,1,True]]
        #prepare p1
        p1=self.SAXSProfile()
        p1.new_flag('agood',bool)
        p1.add_data(data[:3])
        p1.set_filename('test1')
        gp1=self.set_interpolant(p1,1.5,10,MockGP2)
        self.merge.create_intervals_from_data(p1,'agood')
        #prepare p2
        p2=self.SAXSProfile()
        p2.new_flag('agood',bool)
        p2.add_data(data)
        p2.set_filename('test2')
        gp2=self.set_interpolant(p2,2.5,10,MockGP2)
        self.merge.create_intervals_from_data(p2,'agood')
        #prepare p3
        p3=self.SAXSProfile()
        p3.new_flag('agood',bool)
        p3.add_data(data2)
        p3.set_filename('test3')
        gp3=self.set_interpolant(p3,2.5,10,MockGP2)
        self.merge.create_intervals_from_data(p3,'agood')
        #run classification and merging
        args=MockArgs(verbose=0, eschedule=[(1,10)],mergename="merge",
                dalpha=0.05, eextrapolate=0)
        self.merge.classification([p1,p2,p3],args)
        self.merge.fitting_step = lambda a,b,c,d: b
        def thing(b,c,d):
            m=IMP.Model()
            s=IMP.isd.Scale.setup_particle(IMP.Particle(m),3.0)
            gp=MockGP(1,10)
            return m,{'sigma':s},gp
        self.merge.setup_process = thing
        merge, profiles, args = self.merge.merging([p1,p2,p3],args)
        #test
        test=merge.get_data(colwise=True)
        self.assertTrue(
                set(merge.get_flag_names()).issuperset(
                    set(['eorigin','eoriname'])))
        self.assertEqual(len(test['q']),10)
        self.assertEqual(set(zip(test['q'],test['eorigin'])),
                set([(0,0),(0,1),(1,0),(1,1),(2,0),(3,1),
                     (1.1,2),(2.1,2),(3,2),(4,2)]))

if __name__ == "__main__":
    IMP.test.main()
