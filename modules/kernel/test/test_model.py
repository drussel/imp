import IMP
import IMP.test
import StringIO
import random

class DummyRestraint(IMP.Restraint):
    """Dummy do-nothing restraint"""
    def __init__(self, ps=[], cs=[]):
        IMP.Restraint.__init__(self)
        self.ps=ps
        self.cs=cs
    def unprotected_evaluate(self, accum):
        return 0.
    def get_version_info(self):
        return IMP.get_module_version_info()
    def get_input_particles(self):
        return self.ps
    def get_input_containers(self):
        return self.cs


class CustomError(Exception):
    pass

class FailingRestraint(IMP.Restraint):
    """Restraint that fails in evaluate"""
    def unprotected_evaluate(self, accum):
        raise CustomError("Custom error message")
    def get_version_info(self):
        return IMP.get_module_version_info()
    def get_input_particles(self):
        return []
    def get_input_containers(self):
        return []



class DummyScoreState(IMP.ScoreState):
    """Dummy do-nothing score state"""
    def __init__(self, ips=[], ics=[], ops=[], ocs=[]):
        IMP.ScoreState.__init__(self)
        self.ips=ips
        self.ics=ics
        self.ops=ops
        self.ocs=ocs
    def update(self):
        pass
    def get_input_particles(self):
        return self.ips
    def get_output_particles(self):
        #print [type(p) for p in self.ops]
        return self.ops
    def get_input_containers(self):
        return self.ics
    def get_output_containers(self):
        return self.ocs


class ClassScoreState(IMP.ScoreState):
    """Score state that shows the filehandle class"""
    def update(self):
        pass
    def do_show(self, fh):
        fh.write(str(fh.__class__))
        fh.write("; ")
    def get_type_name(self):
        return "ScoreStateTest"
    def get_version_info(self):
        return IMP.get_module_version_info()
    def get_input_particles(self):
        return []
    def get_output_particles(self):
        return []
    def get_input_objects(self):
        return IMP.ObjectsTemp()
    def get_output_objects(self):
        return IMP.ObjectsTemp()



class ModelTests(IMP.test.TestCase):
    def test_state_show(self):
        """Test score state show method"""
        m = IMP.Model()
        s = ClassScoreState()
        sio = StringIO.StringIO()
        s.show(sio)
        m.add_score_state(s)
        for s in m.get_score_states():
            s.show(sio)
        # Output should work for a direct call (in which the filehandle is
        # just the Python file-like object) or via a C++ proxy (in which case
        # the filehandle is a std::ostream adapter)
        self.assertGreater(len(sio.getvalue()), 0)

    def test_score_state(self):
        """Check score state methods"""
        m = IMP.Model()
        #self.assertRaises(IndexError, m.get_score_state,
        #                  0);
        s = DummyScoreState()
        m.add_score_state(s)
        news = m.get_score_state(0)
        self.assertIsInstance(news, IMP.ScoreState)
        #self.assertRaises(IndexError, m.get_score_state,
        #                  1);
        for s in m.get_score_states():
            s.show()

    def test_show(self):
        """Check Model.show() method"""
        class BrokenFile(object):
            def write(self, str):
                raise NotImplementedError()
        m = IMP.Model()
        self.assertRaises(NotImplementedError, m.show, BrokenFile())
        self.assertRaises(AttributeError, m.show, None)
        s = StringIO.StringIO()
        m.show(s)
        self.assertGreater(len(s.getvalue()), 0)

    def test_refcount_director_score_state(self):
        """Refcounting should prevent director ScoreStates from being deleted"""
        dirchk = IMP.test.DirectorObjectChecker(self)
        m = IMP.Model()
        s = DummyScoreState()
        s.python_member = 'test string'
        m.add_score_state(s)
        # Since C++ now holds a reference to s, it should be safe to delete the
        # Python object (director objects should not be freed while C++ holds
        # a reference)
        del s
        news = m.get_score_state(0)
        self.assertEqual(news.python_member, 'test string')
        # Make sure we kept a reference somewhere to this director object
        dirchk.assert_number(1)
        # Cleanup should touch nothing as long as we have Python reference news
        dirchk.assert_number(1)
        del news
        # Should also touch nothing as long as we have C++ references (from m)
        dirchk.assert_number(1)
        del m
        # No refs remain, so make sure all director objects are cleaned up
        dirchk.assert_number(0)

    def test_director_python_exceptions(self):
        """Check that exceptions raised in directors are handled"""
        m = IMP.Model()
        r = FailingRestraint()
        m.add_restraint(r)
        self.assertRaises(CustomError, m.evaluate, False)

    def test_restraints(self):
        """Check restraint methods"""
        m = IMP.Model()
        #self.assertRaises(IndexError, m.get_restraint, 0);
        self.assertEqual(m.get_number_of_restraints(), 0)
        r = DummyRestraint()
        m.add_restraint(r)
        self.assertEqual(m.get_number_of_restraints(), 1)
        newr = m.get_restraint(0)
        self.assertIsInstance(newr, IMP.Restraint)
        #self.assertRaises(IndexError, m.get_restraint,1);
        for s in m.get_restraints():
            s.show()
    def test_temp_restraints(self):
        """Check free restraint methods"""
        m = IMP.Model()
        #self.assertRaises(IndexError, m.get_restraint, 0);
        self.assertEqual(m.get_number_of_restraints(), 0)
        r = DummyRestraint()
        r.set_model(m)
        print r.evaluate(False)
        del r
        m.evaluate(False)

    def test_refcount_director_restraints(self):
        """Refcounting should prevent director Restraints from being deleted"""
        dirchk = IMP.test.DirectorObjectChecker(self)
        m = IMP.Model()
        r = DummyRestraint()
        r.python_member = 'test string'
        m.add_restraint(r)
        # Since C++ now holds a reference to r, it should be safe to delete the
        # Python object (director objects should not be freed while C++ holds
        # a reference)
        del r
        newr = m.get_restraint(0)
        self.assertEqual(newr.python_member, 'test string')
        # Make sure that all director objects are cleaned up
        dirchk.assert_number(1)
        del newr, m
        dirchk.assert_number(0)

    def test_particles(self):
        """Check particle methods"""
        m = IMP.Model()
        p = IMP.Particle(m)
        self.assertEqual(m.get_number_of_particles(), 1)
        for s in m.get_particles():
            s.show()

    def _select(self, ps, n):
        ret=[]
        for i in range(0,n):
            ret.append(random.choice(ps))
        return ret
    def test_dependencies(self):
        """Check dependencies with restraints and score states"""
        m= IMP.Model()
        ps=[IMP.Particle(m) for i in range(0,20)]
        cs=[DummyScoreState(ips=self._select(ps[:5], 2),
                            ops= self._select(ps[5:], 2))
            for i in range(5)]
        for c in cs:
            m.add_score_state(c)
        rs=[DummyRestraint(ps=self._select(ps, 4))
            for i in range(5)]
        for r in rs:
            m.add_restraint(r)
        dg= IMP.get_dependency_graph([m.get_root_restraint_set()])
        #IMP.show_graphviz(dg)
        for r in rs:
            print "now restraint",r
            rcsl=IMP.get_required_score_states([r])
            rcs= set(rcsl)
            rdg= IMP.get_dependency_graph([r])
            #IMP.show_graphviz(rdg)
            ccsl=[]
            for n in rdg.get_vertices():
                nn= rdg.get_vertex_name(n)
                print nn.get_name()
                try:
                    IMP.ScoreState.get_from(nn)
                except:
                    print "not", nn.get_name()
                    pass
                else:
                    print "found", nn.get_name()
                    ccsl.append(nn)
            ccs= set(ccsl)
            # disabled as the graphs now include all score states
            #self.assertEqual(ccs, rcs)
if __name__ == '__main__':
    IMP.test.main()
