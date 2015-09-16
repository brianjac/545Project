@echo off
set xv_path=C:\\Xilinx\\Vivado\\2015.2\\bin
call %xv_path%/xelab  -wto a9027b975f9b44f884bb0758270e305b -m64 --debug typical --relax --mt 2 -L secureip --snapshot top_behav xil_defaultlib.top -log elaborate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
