#!/bin/bash
#

cd probe2cbr
matlab -nosplash -nodesktop -r exe_sample_script
cd ..
java CBRCompiler probe2cbr/data/probe12sal.cbr.txt.descriptor probe2cbr/data/probe12sal.cbr.txt data/probe12sal.cbr

