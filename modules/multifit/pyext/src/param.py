#!/usr/bin/python

__doc__ = 'Build initial parameters files.'

import shutil
import IMP.multifit
from optparse import OptionParser

def parse_args():
    usage = """%prog [options] <assembly name> <subunits file>
       <coarse level> <density map> <resolution> <spacing> <density threshold>
       <origin X> <origin Y> <origin Z>

Build the parameters files for MultiFit.

Notice: If you have negative numbers as input, add -- as the first parameter,
so that the numbers are not treated as options."""

    parser = OptionParser(usage)
    parser.add_option("-i", "--asmb_input", dest="asmb_input",
                      default="asmb.input",
                      help="the name of the MultiFit input file. The default "
                           "filename is asmb.input")
    parser.add_option("-m", "--model", dest="model",default="asmb.model",
                      help="the base filename of the solutions output by "
                           "MultiFit (.X.pdb, where X is the solution number, "
                           "is suffixed to create each output file name). "
                           "The default filename is asmb.model")
    parser.add_option("-a", "--anchor_dir", dest="anchor_dir",default="./",
                      help="the name of the directory to store anchor points. "
                           "The default is ./")
    parser.add_option("-f", "--fit_dir", dest="fit_dir",default="./",
                      help="the name of the directory to store fitting "
                           "solutions. The default is ./")
    (options, args) = parser.parse_args()
    if len(args) < 10:
        parser.error("incorrect number of arguments")
    return options,args

def get_density_data(name,density_fn,resolution,spacing,threshold,
                     origin,anchor_dir_name,fit_dir_name):
    sd=IMP.multifit.SettingsData()
    sd.set_was_used(True)
    msg=sd.get_density_header_line()
    msg+=density_fn+"|"+str(resolution)+"|"+str(spacing)+"|"+str(threshold)+"|"+str(origin[0])+"|"+str(origin[1])+"|"+str(origin[2])
    msg+="|"+anchor_dir_name+name+"_em_coarse_anchors.txt|"+anchor_dir_name+name+"_em_coarse_anchors_FINE.txt|"
    msg+=anchor_dir_name+name+"_em_fine_anchors.txt|"+anchor_dir_name+name+"_em_fine_anchors_FINE.txt|\n"
    return msg;

def get_log_data(intermediate_fn,output_fn,model_fn):
    msg  = "# log Parameters:\n"
    msg += "output "+output_fn+"\n"
    msg += "model "+model_fn+"\n"
    if intermediate_fn != "":
        msg += "intermediate "+intermediate_fn+"\n"
    return msg
def get_clustering_data():
    msg="#    Clustering Parameters:\n";
    msg +="#    clusterParams < axis_angle_thr DEGREES > < min_cluster_size > < distance_between_centers_of_mass >\n";
    msg += "clusterParams 18 1 2.0\n";
    return msg;

def get_base_data():
    msg = "\n#    Base Parameters:\n"
    msg += "#    baseParams <min_base_dist> <max_base_dist>\n"
    msg +="baseParams 5.0 50.0\n";
    return msg;

def get_grid_data():
    msg = "\n#    Grid Parameters:\n"
    msg+="#      grid <grid_step> <max_distance> <vol_func_radius>\n"
    msg += "grid 0.5 6.0 6.0\n"
    return msg

def get_protein_data(pdb_list,coarse_level,anchor_dir_name,fit_dir_name,fit_fn_header,add_reference_fn):
    sd=IMP.multifit.SettingsData()
    sd.set_was_used(True)
    msg=sd.get_component_header_line()
    mdl=IMP.Model()
    for i,fnn in enumerate(open(pdb_list)):
        name=fnn[:-1].split()[0]
        fn=fnn[:-1].split()[1]
        surface_fn=fnn[:-1].split()[1]+".ms"
        #TODO - add the number of copies data
        mh=IMP.atom.read_pdb(fn,mdl)
        num_anchors=len(IMP.atom.get_by_type(mh,IMP.atom.RESIDUE_TYPE))/coarse_level
        msg+=name+"|"+fn+"|"+surface_fn+"|"+anchor_dir_name+name+"_anchors.txt|"+str(num_anchors)+"|"
        msg+=anchor_dir_name+name+"_fine_anchors.txt|"+str(len(IMP.atom.get_by_type(mh,IMP.atom.RESIDUE_TYPE)))
        msg+="|"+fit_dir_name+name+fit_fn_header+"|"
        if add_reference_fn:
            msg=msg+fn+"|\n"
        else:
            msg=msg+"|\n"
    return msg

def create_alignment_param_file(asmb_name,coarse_level):
    #TODO - make load_atomic and rigid parameters
    shutil.copy(IMP.multifit.get_data_path("atomic.alignment.param"),
                asmb_name + ".alignment.param")

def create_fitting_param_file(asmb_name):
    f=open(asmb_name+".multifit.param","w")
    f.write("""#triangle matching parameters
#triangle max_edge min_edge query_radius
#  max_edge: maximum edge length considered for matching triangles
#  min_edge: minimum edge length considered for matching triangles
#  query_radius: maximum distance for which points are considered a match
triangle 20 5 6

#general clustering parameters
#clustering min_cluster_size match_percentage
#  min_cluster_size: clusters of size smaller than this number are removed
#  match_percentage: the percentage of probe points that needs to be matched
#                    after extension.
clustering 2 0.7

#rotation clustering parameters
#rot_clustering max_angle_size
#  max_angle_size: rotations whose angular distance is smaller than this number
#                  are considered to be in the same cluster. The size is
#                  in angles.
rot_clustering 20
#trans_clustering max_angle_size max_displacement
#  max_angle_size: transformations whose angular distance is smaller than this
#                  number are considered to be in the same cluster. The size is
#                  in angles.
#  max_displacement: transformations whose translation is smaller than this
#                    number are considered to be in the same cluster.
#                    The size is in A.
trans_clustering 10 3
#rmsd_clustering max_rmsd
#  max_rmsd: transformations that result in models with rmsd smaller than this
#            number are considered to be in the same cluster.
rmsd_clustering 4
#penetration mode percentage
#  mode: 0 - do not calculate envelope penetration
#        1 - score solutions by envelope penetration
#        2 - prune solutions that penetrate the EM density
#  percentage: prune solutions where X% of the atoms are outside of the envelope
penetration 2 0.6
#ranking min_percentage max_rmsd
#  min_percentage: minimum percentage of matching anchor points
#  max_rmsd: maximum rmsd between matching anchors
ranking .5 15""")
    f.close()


def create_assembly_input_file(pdb_list,coarse_level,anchor_dir,fit_dir,asmb_name,
                               density_map_fn,resolution,spacing,threshold,
                               origin,
                               asmb_input_fn,
                               fit_fn_header="_fitting.txt",
                               add_reference_fn=False):

    msg=""
    msg=msg+get_protein_data(pdb_list,coarse_level,anchor_dir,fit_dir,fit_fn_header,add_reference_fn)
    msg=msg+get_density_data(asmb_name,density_map_fn,resolution,spacing,
                             threshold,origin,anchor_dir,fit_dir)
    f=open(asmb_input_fn,"w")
    f.write(msg)
    f.close()

def main():
    options,args = parse_args()
    asmb_name=args[0]
    pdb_list = args[1]
    coarse_level=int(args[2])
    anchor_dir=options.anchor_dir
    fit_dir=options.fit_dir
    if not anchor_dir[-1] == "/":
        anchor_dir+="/"
    if not fit_dir[-1] == "/":
        fit_dir+="/"

    density_map_fn = args[3]
    resolution=float(args[4])
    spacing=args[5]
    threshold=args[6]
    origin=[float(args[7]),float(args[8]),float(args[9])]
    create_assembly_input_file(pdb_list,coarse_level,anchor_dir,fit_dir,asmb_name,
                               density_map_fn,resolution,spacing,threshold,
                               origin, options.asmb_input)

    create_alignment_param_file(asmb_name,coarse_level)
    create_fitting_param_file(asmb_name)

if __name__=="__main__":
    main()
