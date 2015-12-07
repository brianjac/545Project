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
ExecStep $xv_path/bin/xelab -wto a06fa145c4ed449baa8a1e7c8a5f8e99 -m64 --debug typical --relax --mt 8 --include "../../../beryl.srcs/sources_1/imports/amber25" --include "../../../beryl.srcs/sources_1/imports/system" --include "../../../beryl.srcs/sources_1/imports/tb" -L xil_defaultlib -L xbip_utils_v3_0 -L xbip_pipe_v3_0 -L xbip_bram18k_v3_0 -L mult_gen_v12_0 -L blk_mem_gen_v8_2 -L unisims_ver -L unimacro_ver -L secureip --snapshot tb_behav xil_defaultlib.tb xil_defaultlib.glbl -log elaborate.log
