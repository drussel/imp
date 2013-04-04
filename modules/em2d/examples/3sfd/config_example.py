

# This class specifies the parameters used during the Monte Carlo optimization
class MonteCarloParams:
    def __init__(self):

        # Profile of the set of temperatures used. You can add as many as you
        # want. You can see that here the profile is a tempering
        self.temperatures = [30000,15000,10000,5000,1000]
        # Monte Carlo iterations to use for each value of the temperature.
        # The number of values must be the same as the number of temperatures.
        self.iterations = [200,200,200,200]
        # Maximum translation allowed for the components for each value of
        # the temperature
        self.max_translations = [20,15,10,5]
        # Same as above but with the rotations (in radians)
        self.max_rotations = [1,0.5,0.25,0.1]
        # The number of cycles. Is the number of times that the profile
        # of temperatures, iterations, translations and rotations is repeated
        # Yo can put as many as you want
        self.cycles = 3
        # This parameter indicates the probability for a component of doing
        #  a random movement instead of a relative movement. A value of 0.4
        # means that a component will do a
        # random move instead of moving to a relative position
        # respect to its receptor 40% of the time.
        # If no docking has been done between two structures, this parameter is
        # ignored.
        self.non_relative_move_prob = 0.4


# This class specifies the parameters used for the DOMINO sampling
class DominoSamplingPositions:
    def __init__(self):
        # This parameter is the file where the results of many different
        # Monte Carlo runs are. The starting solutions for DOMINO are read from
        # it. It must be in the SQLite database format, as generated by the
        # MC sampling.
        self.read = "monte_carlo_solutions.db"
        # The maximum number of solutions from the file in self.read to combine
        self.max_number = 50
        # criteria to order the solutions
        self.orderby = "em2d"
        # Here, all the parameters together mean that the 50 best solutions from monte_carlo_solutions.db according to the em2d score are fed to
        # DOMINO.

# This class is for technical parameters for DOMINO.
class DominoParams:
    def __init__(self, ):
        # Exploring all possible combinations of sampling positions
        # is memory intensive. self.heap_solutions is the number of
        #  solutions that you keep each merging step in DOMINO.
        # The larger the number, the better the space of combinations
        #  is explored, at the cost of larger running time
        self.heap_solutions = 2000

# This class is here to inject testing code in the modeling. Don't bother with
# it unless you know the code for the application well.
class TestParameters(object):
    def __init__(self, ):
        self.do_test = False
        self.test_fn_assembly = ""


# This class is used to measure the performance of the method with benchmark
# structures. Same as above, ignore or delete it if you are modeling.
class Benchmark:
    def __init__(self, ):
        self.fn_pdb_native = "3sfd.pdb"
        # parameters for measuring the ccc with the 3D map
        self.resolution = 8
        self.voxel_size = 1.5
        self.native_map_threshold = 0.25
        # The component from the model is aligned to the native component before
        # computing the measures for the benchmark
        self.native_component = "3sfdB"
        self.model_component = "3sfdB"


# this is the main class containing all the parameters of the experiment
class Experiment (object):

    def __init__(self,):

        # The name of the model. You use any name. It is not used by the logic
        # of the application
        self.model_name = ["3sfd"]
        # The name of each of the components (subunits) of the assembly. The
        # names of the subunits are heavily used by the code. Make sure that all
        # the names are different.
        self.names = ["3sfdA","3sfdB","3sfdC","3sfdD"]
        # The number of the pdbs containing the subunits. The name of the
        # files does not need to match the names of the subunits in self.names
        self.fn_pdbs = ["3sfdA.pdb","3sfdB.pdb","3sfdC.pdb","3sfdD.pdb"]
        # Some of the restraints (e.g., excluded volume), use a coarse
        # representation of the components of the assembly. This value is
        # the number of residues included in a bead of the coarse
        # representation
        self.n_residues = 10
        # The components that are going to be anchored. i.e., the components
        # that are going to be set at the origin and with a identity rotation.
        # First start with all False until you perform the initial dockings of
        # components, and the anchor the first receptor. You can override the
        # selection of the anchored component if you want. The only condition
        # is that the anchor can not move relative to other components.
        # The program will crash otherwise.
        self.anchor = [False,False,False,False]
        # This option means that the coordinates of a component are fixed.
        # It makes sense when using DOMINO. It keeps a component in its
        # actual position, ignoring all the possible positions obtained using
        # the Monte Carlo modelings.
        self.fixed =[ False,False,False,False]

        # Pair score restraint. This restraint indicates that 2 components of
        # the complex should be together. The order of the parameters for each
        # restraint is:
        #  component1 - the id of the first component
        #  component2 - the id of the second component
        #  name - The name for the restraint. Make sure that the restraint
        #       does not contain hyphens. SQLite does not like them. Do not
        # given the same name to different restraints.
        #  distance - The distance that the components should keep. It is the
        #       minimum distance between 2 particles of the complex. Distances
        # equal or below this value are not penalized by the scoring function
        # Weight - the weight for the restraint. The value of a restraint is
        #       multiplied by its weight before adding it to the total scoring
        #       function.
        # stddev - This is the "standard deviation". The pair score restraint
        # is an IMP.HarmonicUppperBound function. The stddev value is used to
        # approximate the values of the HarmonicUppperBound to those of a
        # Gaussian function of standard deviation stddev.

        # component1,component2,name,distance,weight,pairs,stddev
        self.pair_score_restraints =[ ["3sfdB","3sfdA","B_A",0,1,1,3.],
                                      ["3sfdB","3sfdC","B_C",0,1,1,3.],
                                      ["3sfdB","3sfdD","B_D",0,1,1,3.],
                                      ["3sfdD","3sfdC","D_C",0,1,1,3.]

                                    ]

        # Excluded volume restraint. Restraint preventing the components from
        # interpenetrating. This restraints sets an all agains all excluded
        # volument restraint. The meaning of the parameters is the same as above
        #distance, weight,pairs,stddev
        self.pairs_excluded_restraint = [0,1,0.1,2]

        # Geometric complementarity restraint:
        # the parameters are the ids of the 2 components, the name of the restraint
        # maximum distance - if 2 components are apart by more than this distance,
        #   the restraint resturns the value 0
        # max_penetration. It is maximum distance that 2 rigid bodies can interpenetrate
        #           before the restraint returns a infinte value. For values of
        #           the interpenetration lower than maximum penetration, the
        #           restraint returns increasing positive values.
        # The value of the restraint is negative only when the subunits show
        # some surface complementarity.
        # Note: There is a known issue with this restraint. Sometimes the program will crash
        # for no apparent reason, and seems to be structure specific. One of the first
        # to do in this case is to remove this restraints.
        # the weight parameter is the weight for the restraint.
        #component1, component2, max distance,max penetration,weight.
        self.complementarity_restraints = [ [ "3sfdB","3sfdA","cB_A",30,15,0.0001],
                                            [ "3sfdB","3sfdC","cB_C",30,15,0.0001],
                                            [ "3sfdB","3sfdD","cB_D",30,15,0.0001],
                                          ]

        # Em2DRestraint. Restraint measuring the similarity between a model and the
        # EM class averages. The parameters are:
        # the name of the restraint
        # the selection file containing the names of the images to use as
        # restraints.
        # The pixel size of the images
        # The resolution used to generate the projections of the model
        # The number of images used for the coarse search part of the em2d calculation.
        # The weight for the restraint
        # and the maximum score tolerated for the restraint. If the value for the
        # maximum score. 1000 is used here because the maximum value for the em2d score
        # is 1 and the weight of the restraint is 1000.
        self.em2d_restraints = [ ["em2d",  "em_images/images.sel", 1.5,5,20,1000,1000 ]
                                 ]

        # Cross-linking restraints. The parameters
        # the ids, chains and residue number of the subunits implied in the restraint
        # The maximum distance between residues tolerated. If the distance
        # is lower, the restraint has 0 value.
        # For the value of the stddev parameter, see the help for the
        # self.pair_score_restraints.
        # You can add a last parameter to the list, the maximum value of the scoring function
        # that you want to accept. It is only used during the DOMINO optimization
        # component1,chain1,residue1, component2, chain2, residue2, distance,weight, stddev, max_score
        self.xlink_restraints =  [ ["3sfdB","B",23,"3sfdA","A",456,30,100,2],
                                   ["3sfdB","B",241,"3sfdC","C",112,30,100,2],
                                   ["3sfdB","B",205,"3sfdD","D",37,30,100,2],
                                   ["3sfdB","B",177,"3sfdD","D",99,30,100,2],
                                   ["3sfdC","C",95,"3sfdD","D",132,30,100,2],
                                   ["3sfdC","C",9,"3sfdD","D",37,30,100,2],
                                   ["3sfdC","C",78,"3sfdD","D",128,30,100,2],
                                    ]
        # set the value to False of this parameter if you don't have the HEXDOCK docking program.
        # self.have_hexdock = False

        #
        self.dock_transforms =  [
                ["3sfdB","3sfdA","relative_positions_3sfdB-3sfdA.txt"],
                ["3sfdB","3sfdC","relative_positions_3sfdB-3sfdC.txt"],
                ["3sfdB","3sfdD","relative_positions_3sfdB-3sfdD.txt"],
                ["3sfdD","3sfdC","relative_positions_3sfdD-3sfdC.txt"],
                                 ]

        self.sampling_positions = DominoSamplingPositions()

        self.monte_carlo = MonteCarloParams()
        self.domino_params = DominoParams()

        self.test_opts = TestParameters()
        # benchmark
        self.benchmark = Benchmark()

        # results
        # the maximum number of solutions to produce
        self.n_solutions = 2000
        self.orderby = "em2d"