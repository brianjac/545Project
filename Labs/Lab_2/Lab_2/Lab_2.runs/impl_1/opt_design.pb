
L
Command: %s
53*	vivadotcl2

opt_design2default:defaultZ4-113h px
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
	xc7vx485t2default:defaultZ17-347h px
�
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
	xc7vx485t2default:defaultZ17-349h px
k
,Running DRC as a precondition to command %s
22*	vivadotcl2

opt_design2default:defaultZ4-22h px
O

Starting %s Task
103*constraints2
DRC2default:defaultZ18-103h px
M
Running DRC with %s threads
24*drc2
82default:defaultZ23-27h px
R
DRC finished with %s
272*project2
0 Errors2default:defaultZ1-461h px
a
BPlease refer to the DRC report (report_drc) for more information.
274*projectZ1-462h px
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.14 ; elapsed = 00:00:00.15 . Memory (MB): peak = 1915.352 ; gain = 13.027 ; free physical = 5651 ; free virtual = 178762default:defaulth px
^

Starting %s Task
103*constraints2&
Logic Optimization2default:defaultZ18-103h px
�

Phase %s%s
101*constraints2
1 2default:default27
#Generate And Synthesize Debug Cores2default:defaultZ18-101h px
;
Refreshing IP repositories
234*coregenZ19-234h px
�
 Loaded user IP repository '%s'.
1135*coregen2I
5/home/DRRA/545Project/Labs/Lab_2/Lab_2/Lab_2.cache/ip2default:defaultZ19-1700h px
�
"Loaded Vivado IP repository '%s'.
1332*coregen2f
R/afs/ece.cmu.edu/support/xilinx/xilinx.release/Vivado-2015.2/Vivado/2015.2/data/ip2default:defaultZ19-2313h px
�
Generating IP %s for %s.
1712*coregen2+
xilinx.com:ip:xsdbm:1.12default:default2

dbg_hub_CV2default:defaultZ19-3806h px
�
NRe-using generated and synthesized IP, "%s", from Vivado IP cache entry "%s".
146*	chipscope2+
xilinx.com:ip:xsdbm:1.12default:default2$
5a372d3d76a73b8e2default:defaultZ16-220h px
�
Generating IP %s for %s.
1712*coregen2)
xilinx.com:ip:ila:5.12default:default2

u_ila_0_CV2default:defaultZ19-3806h px
�
Generating IP %s for %s.
1712*coregen2)
xilinx.com:ip:ila:5.12default:default2

u_ila_1_CV2default:defaultZ19-3806h px
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.052default:default2
00:00:00.062default:default2
2017.9262default:default2
0.0002default:default2
55542default:default2
178032default:defaultZ17-722h px
T
BPhase 1 Generate And Synthesize Debug Cores | Checksum: 103cbe8cc
*commonh px
�

%s
*constraints2�
�Time (s): cpu = 00:05:01 ; elapsed = 00:05:07 . Memory (MB): peak = 2017.926 ; gain = 102.574 ; free physical = 5554 ; free virtual = 178032default:defaulth px
B
%Done setting XDC timing constraints.
35*timingZ38-35h px
f

Phase %s%s
101*constraints2
2 2default:default2
Retarget2default:defaultZ18-101h px
r
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px
H
Retargeted %s cell(s).
49*opt2
02default:defaultZ31-49h px
9
'Phase 2 Retarget | Checksum: 13f6b0239
*commonh px
�

%s
*constraints2�
�Time (s): cpu = 00:05:03 ; elapsed = 00:05:09 . Memory (MB): peak = 2125.949 ; gain = 210.598 ; free physical = 5540 ; free virtual = 177892default:defaulth px
r

Phase %s%s
101*constraints2
3 2default:default2(
Constant Propagation2default:defaultZ18-101h px
r
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px
F
Eliminated %s cells.
10*opt2
22default:defaultZ31-10h px
D
2Phase 3 Constant Propagation | Checksum: b9619647
*commonh px
�

%s
*constraints2�
�Time (s): cpu = 00:05:04 ; elapsed = 00:05:09 . Memory (MB): peak = 2125.949 ; gain = 210.598 ; free physical = 5535 ; free virtual = 177842default:defaulth px
c

Phase %s%s
101*constraints2
4 2default:default2
Sweep2default:defaultZ18-101h px
S
 Eliminated %s unconnected nets.
12*opt2
1902default:defaultZ31-12h px
d
1Inserted %s IBUFs to IO ports without IO buffers.100*opt2
22default:defaultZ31-140h px
R
!Eliminated %s unconnected cells.
11*opt2
82default:defaultZ31-11h px
6
$Phase 4 Sweep | Checksum: 11978f4b2
*commonh px
�

%s
*constraints2�
�Time (s): cpu = 00:05:04 ; elapsed = 00:05:10 . Memory (MB): peak = 2125.949 ; gain = 210.598 ; free physical = 5534 ; free virtual = 177842default:defaulth px
^

Starting %s Task
103*constraints2&
Connectivity Check2default:defaultZ18-103h px
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.02 ; elapsed = 00:00:00.02 . Memory (MB): peak = 2125.949 ; gain = 0.000 ; free physical = 5534 ; free virtual = 177842default:defaulth px
G
5Ending Logic Optimization Task | Checksum: 11978f4b2
*commonh px
�

%s
*constraints2�
�Time (s): cpu = 00:05:04 ; elapsed = 00:05:10 . Memory (MB): peak = 2125.949 ; gain = 210.598 ; free physical = 5534 ; free virtual = 177842default:defaulth px
>
,Implement Debug Cores | Checksum: 11eb43b42
*commonh px
;
)Logic Optimization | Checksum: 1e8f285c1
*commonh px
^

Starting %s Task
103*constraints2&
Power Optimization2default:defaultZ18-103h px
p
7Will skip clock gating for clocks with period < %s ns.
114*pwropt2
3.122default:defaultZ34-132h px
:
Applying IDT optimizations ...
9*pwroptZ34-9h px
<
Applying ODC optimizations ...
10*pwroptZ34-10h px


*pwropth px
b

Starting %s Task
103*constraints2*
PowerOpt Patch Enables2default:defaultZ18-103h px
�
�WRITE_MODE attribute of %s BRAM(s) out of a total of %s has been updated to save power.
    Run report_power_opt to get a complete listing of the BRAMs updated.
129*pwropt2
12default:default2
22default:defaultZ34-162h px
a
+Structural ODC has moved %s WE to EN ports
155*pwropt2
02default:defaultZ34-201h px
�
CNumber of BRAM Ports augmented: %s newly gated: %s Total Ports: %s
65*pwropt2
02default:default2
02default:default2
42default:defaultZ34-65h px
K
9Ending PowerOpt Patch Enables Task | Checksum: 11978f4b2
*commonh px
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.01 ; elapsed = 00:00:00 . Memory (MB): peak = 2189.957 ; gain = 0.000 ; free physical = 5487 ; free virtual = 177362default:defaulth px
G
5Ending Power Optimization Task | Checksum: 11978f4b2
*commonh px
�

%s
*constraints2�
�Time (s): cpu = 00:00:00.86 ; elapsed = 00:00:00.81 . Memory (MB): peak = 2189.957 ; gain = 64.008 ; free physical = 5487 ; free virtual = 177362default:defaulth px
W
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
412default:default2
02default:default2
02default:default2
02default:defaultZ4-41h px
Y
%s completed successfully
29*	vivadotcl2

opt_design2default:defaultZ4-42h px
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2 
opt_design: 2default:default2
00:05:052default:default2
00:05:112default:default2
2189.9572default:default2
296.6372default:default2
54872default:default2
177362default:defaultZ17-722h px
A
Writing placer database...
1603*designutilsZ20-1893h px
:
Writing XDEF routing.
211*designutilsZ20-211h px
G
#Writing XDEF routing logical nets.
209*designutilsZ20-209h px
G
#Writing XDEF routing special nets.
210*designutilsZ20-210h px
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2)
Write XDEF Complete: 2default:default2
00:00:00.062default:default2
00:00:00.022default:default2
2221.9652default:default2
0.0002default:default2
54842default:default2
177362default:defaultZ17-722h px
M
Running DRC with %s threads
24*drc2
82default:defaultZ23-27h px
�
#The results of DRC are in file %s.
168*coretcl2�
J/home/DRRA/545Project/Labs/Lab_2/Lab_2/Lab_2.runs/impl_1/ddr_drc_opted.rptJ/home/DRRA/545Project/Labs/Lab_2/Lab_2/Lab_2.runs/impl_1/ddr_drc_opted.rpt2default:default8Z2-168h px


End Record