#!/usr/bin/python

#analyse the ensemble, first we will do the rmsd stuff
import sys,os,string,operator
import IMP.multifit
import IMP.multifit2
from optparse import OptionParser
def usage():
    usage =  "usage %prog [options] <asmb.input> <proteomics.input> <mapping.input> <alignment params> <combinatins> <diameter> <output combinations>\n"
    usage+="A script for clustering an ensemble of solutions"
    parser = OptionParser(usage)
    (options, args) = parser.parse_args()
    if len(args) != 7:
        parser.error("incorrect number of arguments")
    return [options,args]

def run(asmb_fn,proteomics_fn,mapping_fn,align_param_fn,
        comb_fn,diameter,output_comb_fn):
    asmb_data=IMP.multifit.read_settings(asmb_fn)
    prot_data=IMP.multifit.read_proteomics_data(proteomics_fn)
    mapping_data=IMP.multifit2.read_protein_anchors_mapping(prot_data,mapping_fn)
    alignment_params = IMP.multifit2.AlignmentParams(align_param_fn)
    alignment_params.process_parameters()

    #load all proteomics restraints
    align=IMP.multifit2.ProteomicsEMAlignmentAtomic(mapping_data,asmb_data,alignment_params)
    mdl=align.get_model()
    mhs=align.get_molecules()
    ensb=IMP.multifit2.Ensemble(asmb_data,mapping_data)
    for i,mh in enumerate(mhs):
        ensb.add_component_and_fits(mh,
                                    IMP.multifit.read_fitting_solutions(asmb_data.get_component_header(i).get_transformations_fn()))

    mol_path_centers=[] #save the molecule centers for each path
    #iterate over the molecules
    print "NUMBER OF COMPS:",asmb_data.get_number_of_component_headers()
    for i in range(asmb_data.get_number_of_component_headers()):
        mol_centers=[] #all the centers of a specific molecule
        mh_leaves=IMP.core.get_leaves(mhs[i])
        #iterate over the paths and add the center of the path
        mh_paths=mapping_data.get_paths_for_protein(prot_data.get_protein_name(i))
        dummy_comb=[]
        for j in range(asmb_data.get_number_of_component_headers()):
            dummy_comb.append(0)
        for j in range(len(mh_paths)):
            dummy_comb[i]=j
            ensb.load_combination(dummy_comb)
            #print IMP.core.XYZs(mh_leaves)
            mol_centers.append(IMP.core.get_centroid(IMP.core.XYZs(mh_leaves)))
            ensb.unload_combination(dummy_comb)
        mol_path_centers.append(mol_centers)
    for i,p in enumerate(mol_path_centers):
        print "number of paths for mol:",i,"is",len(p)
    #load combinations
    combs=IMP.multifit2.read_paths(comb_fn)
    comb_centroids=[]
    for comb in combs:
        mh_c=[]
        for i in range(len(mhs)):
            mh_c+=mol_path_centers[i][comb[i]]
        comb_centroids.append(IMP.algebra.VectorKD(mh_c))
    embed=IMP.statistics.VectorDEmbedding(comb_centroids)
    #TODO - use your RMSD clustering
    bin_cluster= IMP.statistics.create_bin_based_clustering(embed,diameter)
    print "number of clusters:",bin_cluster.get_number_of_clusters()
    cluster_stat=[]
    for k in range(bin_cluster.get_number_of_clusters()):
        bc= bin_cluster.get_cluster(k)
        cluster_stat.append([len(bc),k,bc])
    cluster_stat=sorted(cluster_stat,key=operator.itemgetter(0),reverse=True)
    cluster_reps=[]
    for ind,[cluster_size,cluster_ind,cluster_elems] in enumerate(cluster_stat):
        print "cluster index:",ind,"with",cluster_size,"combinations"
        cluster_reps.append(combs[cluster_elems[0]])
    print "============clustering============"
    print "Number of clusters found "+str(len(cluster_reps))
    print "=================================="
    IMP.multifit2.write_paths(cluster_reps,output_comb_fn)

if __name__=="__main__":
    (options,args) = usage()
    run(args[0],args[1],args[2],args[3],args[4],float(args[5]),args[6])