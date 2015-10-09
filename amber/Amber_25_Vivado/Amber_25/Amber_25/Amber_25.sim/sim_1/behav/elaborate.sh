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
ExecStep $xv_path/bin/xelab -wto 8c4eff9f186f4275bdba3e1f0fb2effe -m64 --debug typical --relax --mt 8 --include "../../../Amber_25.srcs/sources_1/imports/amber23" --include "../../../Amber_25.srcs/sources_1/imports/vlog/amber25" --include "../../../Amber_25.srcs/sources_1/imports/vlog/ethmac" --include "../../../Amber_25.srcs/sources_1/imports/vlog/system" --include "../../../Amber_25.srcs/sources_1/imports/vlog/tb" -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip --snapshot tb_behav xil_defaultlib.tb xil_defaultlib.glbl -log elaborate.log
