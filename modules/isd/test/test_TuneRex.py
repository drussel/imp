#!/usr/bin/env python

import os
import numpy.random as random
from numpy import *

import IMP
import IMP.isd.Replica
import IMP.isd.TuneRex
import MockGrid
import IMP.test

class TestTuneRex(IMP.test.TestCase):
    def setUp(self):
        IMP.test.TestCase.setUp(self)
        random.seed()
        nreps = self.nreps = 9
        temps = self.temps = [300.0 + (500.0-300.0)*i/float(nreps-1) for i in 
                xrange(nreps)]
        steps = self.steps = [1.0 for i in xrange(nreps)]
	
	
	
	tune_data = {'rate' : 100, #temp optimization rate, in rex steps.
        'method':'ar', 	   #flux-optimization or cv-optimization
	'targetAR':0.4,
	'dumb_scale':0.7,
        'alpha':0.05}              #type I error on the estimates

	
	
        grid=self.grid=MockGrid.MockGrid(nreps,temps,steps)
        replica=self.replica=IMP.isd.Replica.ReplicaTracker(grid.nreps,
            [1/(MockGrid.kB*t) for t in grid._temps], 
            grid, 123, tune_temps=True, tune_data=tune_data)
    	
    def read_replicanums_file(self):
          fr=open('replicanums.txt')
	  longls=[]
	  for line in fr:
                ls=map(int,line.split())
		ls.pop(0)
	        longls.append(ls)
          
	  return transpose(longls)-1
	  
	  
	  
    def test_flux(self):


         self.replica.tune_data['dumb_scale']=0.5
         for i in range(1000):

	      self.replica.replica_exchange()
    	      self.replica.write_rex_stats()


         del self.replica.tune_data['dumb_scale']
         del self.replica.tune_data
	 #self.replica.tune_data['CvMethod']='constant'
	 #self.replica.tune_data['goodMethod']='step'
	 #self.replica.tune_data['badMethod']='step'
	 #self.replica.tune_data['rate']=500
	 self.replica.tune_data['method']='flux'
	 
         for i in range(10000):

	      self.replica.replica_exchange()
    	      self.replica.write_rex_stats()

         self.replica.tune_data['dumb_scale']=0.1


	 
	 
	 replicanums=self.read_replicanums_file()
	 indicators = IMP.isd.TuneRex.compute_indicators(replicanums, subs=1,start=5000)
         print  array([sum(ind)/float(len(ind)) for ind in indicators])
	
	
    def tearDown(self):
        IMP.test.TestCase.tearDown(self)
        #if os.path.exists('temps.txt'):
        #    os.remove('temps.txt')
        #if os.path.exists('replicanums.txt'):
        #    os.remove('replicanums.txt')

    


if __name__ == '__main__':
    IMP.test.main()

        #tune_data={'rate':100000,'method':'cv','targetAR':0.4,'alpha':0.1,
         #   'CvMethod':'constant'})
        #for i in xrange(50000):
        #    replica.replica_exchange()
        #    replica.write_rex_stats()
