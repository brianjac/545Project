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
ExecStep $xv_path/bin/xsim tb_behav -key {Behavioral:sim_1:Functional:tb} -tclbatch tb.tcl -view /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/tb_behav_01.wcfg -view /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/tb_mem_alu_ctrlflow.wcfg -log simulate.log
