#!/bin/sh -f
xv_path="/afs/ece.cmu.edu/support/xilinx/xilinx.release/Vivado-2015.2/Vivado/2015.2"
ExecStep()
{
"$@"
RETVAL=$?
if [ $RETVAL -ne 0 ]
then
exit $RETVAL
fi
}
ExecStep $xv_path/bin/xsim tb_behav -key {Behavioral:sim_1:Functional:tb} -tclbatch tb.tcl -log simulate.log
