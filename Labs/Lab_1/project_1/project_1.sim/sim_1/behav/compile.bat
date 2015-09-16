@echo off
set xv_path=C:\\Xilinx\\Vivado\\2015.2\\bin
if "%errorlevel%"=="1" goto END
if "%errorlevel%"=="0" goto SUCCESS
:END
exit 1
:SUCCESS
exit 0
