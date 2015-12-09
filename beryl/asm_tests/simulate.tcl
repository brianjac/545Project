open_project /home/DRRA/545Project/amber/Amber_25_Vivado/Amber_25/Amber_25/Amber_25.xpr
set_property -name {xsim.simulate.runtime} -value {10000000ns} -objects [current_fileset -simset]
launch_simulation
