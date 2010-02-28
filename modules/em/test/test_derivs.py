# test that derivatives agree with numerical ones
try:
    import modeller
    import IMP.modeller
except ImportError:
    modeller = None
import IMP
import IMP.test
import sys
import IMP.em
import unittest
import os


class DerivativesTest(IMP.test.TestCase):
    """check the agreement of numerical and analytical
       derivatives"""
    def test_deriv(self):
        """Test calculated derivatives for a distorted model's map"""
        if modeller is None:
            sys.stderr.write("test skipped: modeller module unavailable: ")
            return
        modeller.log.level= (0,0,0,0,1)
        self.env = modeller.environ()
        self.env.edat.dynamic_sphere = False
        self.env.libs.topology.read(file='$(LIB)/top_heav.lib')
        self.env.libs.parameters.read(file='$(LIB)/par.lib')
        #init IMP model ( the environment)
        self.imp_model = IMP.Model()
        self.particles = IMP.Particles()
        #add IMP Restraints into the modeller scoring function
        t = self.env.edat.energy_terms
        t.append(IMP.modeller.IMPRestraints(self.imp_model, self.particles))
        ## -  create a set of three particles in imp
        npart = 3
        self.modeller_model = IMP.modeller.create_particles(npart,
                                  self.env, self.imp_model, self.particles)
        # - add the particles attributes ( other than X,Y,Z)
        rad = 1.0
        wei = 1.0
        rad_key=IMP.FloatKey("radius")
        x_key=IMP.FloatKey("x")
        y_key=IMP.FloatKey("y")
        z_key=IMP.FloatKey("z")
        wei_key=IMP.FloatKey("weight")
        prot_key=IMP.IntKey("protein")
        id_key=IMP.IntKey("id")

        for i,p_data in enumerate([[9.0,9.0,9.0,rad,wei,1],[12.0,3.0,3.0,rad,wei,1],[3.0,12.0,12.0,rad,wei,1]]):
            p=self.particles[i]
            p.set_value(x_key,p_data[0])
            p.set_value(y_key,p_data[1])
            p.set_value(z_key,p_data[2])
            p.add_attribute(rad_key,p_data[3])
            p.add_attribute(wei_key,p_data[4])
            p.add_attribute(prot_key,p_data[5])
            self.particles[i].add_attribute(id_key,i)


        IMP.modeller.copy_imp_coords_to_modeller(self.particles,self.modeller_model.atoms)
        #modeller_model.write(file='xxx.pdb')
        self.atmsel = modeller.selection(self.modeller_model)
        print "initialization done ..."

        resolution=3.
        voxel_size=1.
        model_map = IMP.em.SampledDensityMap(self.particles, resolution, voxel_size,rad_key,wei_key)
        erw = IMP.em.EMReaderWriter()
        xorigin = model_map.get_header().get_xorigin()
        yorigin = model_map.get_header().get_yorigin()
        zorigin = model_map.get_header().get_zorigin()
        print("x= " + str(xorigin) + " y=" + str(yorigin) + " z=" + str(zorigin) )
        IMP.em.write_map(model_map, "xxx.em",erw)
        # EM restraint
        em_map = IMP.em.read_map("xxx.em",erw)
        em_map.get_header_writable().set_xorigin(xorigin)
        em_map.get_header_writable().set_yorigin(yorigin)
        em_map.get_header_writable().set_zorigin(zorigin)
        em_map.get_header_writable().set_resolution(resolution)
        ind_emrsr = []
        print "======BEFORe rad_key:",rad_key
        ind_emrsr.append(IMP.em.FitRestraint(self.particles,
                                             em_map,
                                             rad_key,wei_key,
                                             1.0))
        print "======AFTER"
        self.imp_model.add_restraint(ind_emrsr[0])
        print("EM-score score: "+str(self.atmsel.energy()) )
        self.atmsel.randomize_xyz(1.0)
        nviol = self.atmsel.debug_function(debug_function_cutoff=(.010, 0.010, 0.01),
                                      detailed_debugging=True)
        self.assert_(nviol < 1, "at least one partial derivative is wrong!")
        print " derivs done ..."
        os.unlink("xxx.em")

if __name__ == '__main__':
    unittest.main()
