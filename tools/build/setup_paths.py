#!/usr/bin/python

"""Create the paths.cpp file for the base module so IMP can find data once
it is installed.
"""

import sys
from optparse import OptionParser
import os.path

template="""
#include <IMP/base/base_config.h>

IMPBASE_BEGIN_INTERNAL_NAMESPACE
 std::string imp_data_path="%(datapath)s";
 std::string imp_example_path="%(examplepath)s";
IMPBASE_END_INTERNAL_NAMESPACE
"""

parser = OptionParser()
parser.add_option("-d", "--datapath", dest="datapath",
                  help="The install data path.")
parser.add_option("-e", "--examplepath", dest="examplepath",
                  help="The install example path.")
parser.add_option("-o", "--output", dest="output",
                  help="Where to put the file.")




def main():
    (options, args) = parser.parse_args()
    data={}
    data["examplepath"]= options.examplepath
    data["datapath"]= options.datapath
    try:
        os.makedirs(os.path.split(options.output)[0])
    except:
        # already there
        pass
    open(options.output, "w").write(template%data)

if __name__ == '__main__':
    main()
