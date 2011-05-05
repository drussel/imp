import IMP.domino

m= IMP.Model()
# An undirected graph with an IMP::Object for each node
g= IMP.domino.InteractionGraph()
vs=[]
ps=[]
for i in range(0,10):
    ps.append(IMP.Particle(m))
    vs.append(g.add_vertex(ps[-1]));
g.add_edge(vs[0], vs[1])
g.add_edge(vs[1], vs[2])
g.show_graphviz()
# use (graphviz) dot to render the above graph
# ./tools/imppy.sh python ../svn/kernel/examples/graph.py >/tmp/graph.dot
# drussel@udp012430uds:debug> dot /tmp/graph.dot -Tpdf > /tmp/out.pdf


try:
    import matplotlib
    # the engine to be used must be selected before pyplot is imported
    matplotlib.use("macosx")
    import matplotlib.pyplot as plt

    # the method below requires the altgraph python package
    xg=IMP.get_networkx_graph(g)

    import networkx
    networkx.draw(xg)
    networkx.draw_shell(xg)
    plt.show()
except:
    print "networkx not fully installed"

g.remove_vertex(0)
g.show_graphviz()
# in and out neighbors are the same
for n in g.get_in_neighbors(8):
    print g.get_vertex_name(n).get_name()
