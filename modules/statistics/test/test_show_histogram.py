import IMP.test
import IMP
import IMP.statistics
import IMP.algebra
import random
import sys


class MockAxes(object):
    def __init__(self, events):
        self.events = events

    def bar(self, left, height, width=0.8, bottom=None, **kwargs):
        self.events.append('bar %d[%.1f %.1f] %d %.1f %s' \
                           % (len(left), left[0], left[-1],
                              len(height), width, str(kwargs)))

    def set_xlim(self, left=None, right=None, emit=True, auto=False, **kw):
        self.events.append('xlim %.1f %.1f' % (left, right))

    def set_xscale(self, value, **kwargs):
        self.events.append('xscale ' + value)

    def set_yscale(self, value, **kwargs):
        self.events.append('yscale ' + value)

    def set_autoscaley_on(self, b):
        self.events.append('set_autoscaley_on ' + str(b))

class MockMatplotlib(object):
    def __init__(self):
        self.events = []
        self.pyplot = MockPyplot(self.events)
        self.mlab = MockMlab()

class MockMlab(object):
    pass

class MockPyplot(object):
    def __init__(self, events):
        self.events = events

    def figure(self):
        self.events.append('figure')
        return MockFigure(self.events)

    def show(self):
        self.events.append('plot.show')


class MockFigure(object):
    def __init__(self, events):
        self.events = events

    def add_subplot(self, *args, **kwargs):
        self.events.append('add_subplot %s' % str(args))
        return MockAxes(self.events)


def show_2d(h):
    import numpy as np
    import matplotlib.cm as cm
    import matplotlib.mlab as mlab
    import matplotlib.pyplot as plt
    cg= h.get_counts()
    steps=cg.get_unit_cell()
    x = np.arange(cg.get_bounding_box().get_corner(0)[0]+.5*steps[0],
                  cg.get_bounding_box().get_corner(1)[0]-.5*steps[0])
    y = np.arange(cg.get_bounding_box().get_corner(0)[1]+.5*steps[1],
                  cg.get_bounding_box().get_corner(1)[1]-.5*steps[1])
    X, Y = np.meshgrid(x, y)
    Z,junk= np.meshgrid(x,y)
    for i,xi in enumerate(x):
        for j, yj in enumerate(y):
            Z[i][j]=cg[cg.get_nearest_index(IMP.algebra.Vector2D(xi,yj))]
    im = plt.pcolor(X,Y, Z, cmap=cm.jet)
    plt.colorbar(im)
    plt.show()

class Tests(IMP.test.TestCase):
    "Test that the histogram functionality works"
    def Setup(self):
        IMP.test.TestCase.setUp(self)
    def test_histogram_1d(self):
        """Check that the histogram1D of a gaussian is OK"""
        try:
            import numpy
        except ImportError:
            self.skipTest('no numpy module present')
        hist = IMP.statistics.Histogram1D(5.0, IMP.algebra.BoundingBox1D(IMP.algebra.Vector1D(-100),
                                                                IMP.algebra.Vector1D( 100)))
        sigma=10
        for i in range(1000):
            x=random.gauss(0, 10)
            #print x
            hist.add(IMP.algebra.Vector1D(x))

        mock = MockMatplotlib()
        sys.modules['matplotlib'] = mock
        sys.modules['matplotlib.pyplot'] = mock.pyplot
        sys.modules['matplotlib.mlab'] = mock.mlab
        try:
            IMP.statistics.show_histogram(hist)
            self.assertEqual(mock.events,
                 ['figure', 'add_subplot (111,)',
                  "bar 40[-100.0 95.0] 40 5.0 {'align': 'edge'}",
                  'xlim -100.0 100.0', 'xscale linear', 'yscale linear',
                  'set_autoscaley_on False', 'plot.show'])

        finally:
            del sys.modules['matplotlib']
            del sys.modules['matplotlib.pyplot']
            del sys.modules['matplotlib.mlab']

    def test_histogram_2d(self):
        """Check that the histogram2D of a gaussian is OK"""
        hist = IMP.statistics.Histogram2D(5.0, IMP.algebra.BoundingBox2D(IMP.algebra.Vector2D(-100, -100),
                                                                IMP.algebra.Vector2D( 100, 100)))
        sigma=10
        for i in range(1000):

            x=random.gauss(0, 10)
            y=random.gauss(0, 10)
            #print x
            hist.add(IMP.algebra.Vector2D(x, y))
        print hist.get_counts()
        print hist.get_frequencies()
        #show_2d(hist)


    def test_histogram_3d(self):
        """Check that the histogram3D of a gaussian is OK"""
        hist = IMP.statistics.Histogram3D(1.0, IMP.algebra.get_cube_3d(6))
        for i in range(1000):

            x=random.gauss(0, 1)
            y=random.gauss(0, 1)
            z=random.gauss(0, 1)
            #print x
            hist.add(IMP.algebra.Vector3D(x, y, z))
        print hist.get_counts()
        print hist.get_frequencies()
        IMP.statistics.show_histogram(hist.get_frequencies(), vmin=.001, vmax=.2)

if __name__ == '__main__':
    IMP.test.main()
