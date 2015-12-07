proc start_step { step } {
  set stopFile ".stop.rst"
  if {[file isfile .stop.rst]} {
    puts ""
    puts "*** Halting run - EA reset detected ***"
    puts ""
    puts ""
    return -code error
  }
  set beginFile ".$step.begin.rst"
  set platform "$::tcl_platform(platform)"
  set user "$::tcl_platform(user)"
  set pid [pid]
  set host ""
  if { [string equal $platform unix] } {
    if { [info exist ::env(HOSTNAME)] } {
      set host $::env(HOSTNAME)
    }
  } else {
    if { [info exist ::env(COMPUTERNAME)] } {
      set host $::env(COMPUTERNAME)
    }
  }
  set ch [open $beginFile w]
  puts $ch "<?xml version=\"1.0\"?>"
  puts $ch "<ProcessHandle Version=\"1\" Minor=\"0\">"
  puts $ch "    <Process Command=\".planAhead.\" Owner=\"$user\" Host=\"$host\" Pid=\"$pid\">"
  puts $ch "    </Process>"
  puts $ch "</ProcessHandle>"
  close $ch
}

proc end_step { step } {
  set endFile ".$step.end.rst"
  set ch [open $endFile w]
  close $ch
}

proc step_failed { step } {
  set endFile ".$step.error.rst"
  set ch [open $endFile w]
  close $ch
}

set_msg_config -id {HDL 9-1061} -limit 100000
set_msg_config -id {HDL 9-1654} -limit 100000

start_step init_design
set rc [catch {
  create_msg_db init_design.pb
  debug::add_scope template.lib 1
  create_project -in_memory -part xc7vx485tffg1761-2
  set_property board_part xilinx.com:vc707:part0:1.2 [current_project]
  set_property design_mode GateLvl [current_fileset]
  set_property webtalk.parent_dir /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.cache/wt [current_project]
  set_property parent.project_path /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.xpr [current_project]
  set_property ip_repo_paths /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.cache/ip [current_project]
  set_property ip_output_repo /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.cache/ip [current_project]
  add_files -quiet /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.runs/synth_1/system.dcp
  read_xdc -prop_thru_buffers -ref clk_wiz_0 -cells inst /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_board.xdc
  set_property processing_order EARLY [get_files /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_board.xdc]
  read_xdc -ref clk_wiz_0 -cells inst /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xdc
  set_property processing_order EARLY [get_files /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xdc]
  read_xdc /afs/ece.cmu.edu/usr/wpe/Private/545/beryl/beryl.srcs/constrs_1/imports/drra/amber_constraints_fakeddr3.xdc
  link_design -top system -part xc7vx485tffg1761-2
  close_msg_db -file init_design.pb
} RESULT]
if {$rc} {
  step_failed init_design
  return -code error $RESULT
} else {
  end_step init_design
}

start_step opt_design
set rc [catch {
  create_msg_db opt_design.pb
  catch {write_debug_probes -quiet -force debug_nets}
  opt_design 
  write_checkpoint -force system_opt.dcp
  catch {report_drc -file system_drc_opted.rpt}
  close_msg_db -file opt_design.pb
} RESULT]
if {$rc} {
  step_failed opt_design
  return -code error $RESULT
} else {
  end_step opt_design
}

start_step place_design
set rc [catch {
  create_msg_db place_design.pb
  catch {write_hwdef -file system.hwdef}
  place_design 
  write_checkpoint -force system_placed.dcp
  catch { report_io -file system_io_placed.rpt }
  catch { report_utilization -file system_utilization_placed.rpt -pb system_utilization_placed.pb }
  catch { report_control_sets -verbose -file system_control_sets_placed.rpt }
  close_msg_db -file place_design.pb
} RESULT]
if {$rc} {
  step_failed place_design
  return -code error $RESULT
} else {
  end_step place_design
}

start_step route_design
set rc [catch {
  create_msg_db route_design.pb
  route_design 
  write_checkpoint -force system_routed.dcp
  catch { report_drc -file system_drc_routed.rpt -pb system_drc_routed.pb }
  catch { report_timing_summary -warn_on_violation -max_paths 10 -file system_timing_summary_routed.rpt -rpx system_timing_summary_routed.rpx }
  catch { report_power -file system_power_routed.rpt -pb system_power_summary_routed.pb }
  catch { report_route_status -file system_route_status.rpt -pb system_route_status.pb }
  catch { report_clock_utilization -file system_clock_utilization_routed.rpt }
  close_msg_db -file route_design.pb
} RESULT]
if {$rc} {
  step_failed route_design
  return -code error $RESULT
} else {
  end_step route_design
}

start_step write_bitstream
set rc [catch {
  create_msg_db write_bitstream.pb
  write_bitstream -force system.bit 
  catch { write_sysdef -hwdef system.hwdef -bitfile system.bit -meminfo system.mmi -ltxfile debug_nets.ltx -file system.sysdef }
  close_msg_db -file write_bitstream.pb
} RESULT]
if {$rc} {
  step_failed write_bitstream
  return -code error $RESULT
} else {
  end_step write_bitstream
}

