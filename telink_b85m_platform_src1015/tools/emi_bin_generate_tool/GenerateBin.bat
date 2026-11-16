@echo off
REM ##################################################################################################################
REM ################################[Instructions for use]############################################################
REM STEP1: Modify TOOL_PATH_RLEASE_SDK to the installation path of TC32 IDE, be careful not to have spaces in the path,
REM        or surrounded by double quotes.
REM STEP2: Choose project to generate bins, 0: not excute, 1: excute.
REM STEP3: Modify the version number.
REM STEP4: Double click this file to run.
REM ##################################################################################################################

set TOOL_PATH_RLEASE_SDK=C:\TelinkSDK
set Project_Path_Rlease_SDK=..\..\
set EMI_compile_option=BQB_EMI_Demo

REM Choose project to generate bins, 0: not excute, 1: excute
set release_B80_flag=0
set release_B85_flag=0
set release_B87_flag=1
set release_B89_flag=0

REM Modify the version number
set B80_EMI_Release_Version=_V0001
set B85_EMI_Release_Version=_V0006
set B87_EMI_Release_Version=_V0003
set B89_EMI_Release_Version=_V0001

set chip_name=B85m
set release_version=_vxxxx
set cap_switch=_CapInternal
set power_mode=_PowerDCDC
set calibration_position=_CalibFlash
set program_mode=_ProgramFlashSRAM
set emi_bin_name=%chip_name%_EMI%cap_switch%%power_mode%%calibration_position%%program_mode%%release_version%

set Project_Path_B80=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B80
set Project_Path_B85=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B85
set Project_Path_B87=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B87
set Project_Path_B89=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B89


goto :MAIN


:TEST_COMPILE

if not exist %~dp0output mkdir %~dp0output

setlocal EnableDelayedExpansion


REM ###########################################################
REM B80 EMI
REM ###########################################################

set Project_Path_Compile=%Project_Path_B80%
set Test_Compile_CompileOption=B80_Driver_Demo

if %release_B80_flag% == 1 (

set chip_name=B80
set release_version=%B80_EMI_Release_Version%
set power_mode=_PowerLDO
set program_mode=_ProgramFlashSRAM

if not exist %~dp0output\!chip_name! mkdir %~dp0output\!chip_name!

REM set as EMI_DEMO
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_config.h TEST_DEMO EMI_DEMO

REM set as SRAM&FLASH program 
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\chip\!chip_name!\boot\cstartup_flash.S ALL_SRAM_CODE 1

call :B80_B89_Generate_Bin_Unit2 

)

REM ###########################################################
REM B85 EMI
REM ###########################################################

set Project_Path_Compile=%Project_Path_B85%
set Test_Compile_CompileOption=B85_Driver_Demo

if %release_B85_flag% == 1 (

set chip_name=B85
set release_version=%B85_EMI_Release_Version%
set power_mode=_PowerDCDC
set program_mode=_ProgramFlashSRAM
set calibration_position=_CalibFlash

if not exist %~dp0output\!chip_name! mkdir %~dp0output\!chip_name!

REM set as EMI_DEMO
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_config.h TEST_DEMO EMI_DEMO

REM set as SRAM&FLASH program 
call %~dp0.\tool st_node %Project_Path_Compile%\.cproject %~dp0config\config.ini B85m_Boot_Macro %EMI_compile_option%:-DMCU_STARTUP_FLASH:-DMCU_STARTUP_SRAM

call :B85m_Generate_Bin_Unit1

)

REM ###########################################################
REM B87 EMI
REM ###########################################################

set Project_Path_Compile=%Project_Path_B87%
set Test_Compile_CompileOption=B87_Driver_Demo

if %release_B87_flag% == 1 (

set chip_name=B87
set release_version=%B87_EMI_Release_Version%
set power_mode=_PowerDCDC
set program_mode=_ProgramFlashSRAM
set calibration_position=_CalibFlash

if not exist %~dp0output\!chip_name! mkdir %~dp0output\!chip_name!

REM set as EMI_DEMO
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_config.h TEST_DEMO EMI_DEMO

REM set as SRAM&FLASH program 
call %~dp0.\tool st_node %Project_Path_Compile%\.cproject %~dp0config\config.ini B85m_Boot_Macro %EMI_compile_option%:-DMCU_STARTUP_FLASH:-DMCU_STARTUP_SRAM

call :B87_Generate_Bin_Unit2

)

REM ###########################################################
REM B89 EMI
REM ###########################################################

set Project_Path_Compile=%Project_Path_B89%
set Test_Compile_CompileOption=B89_Driver_Demo

if %release_B89_flag% == 1 (

set chip_name=B89
set release_version=%B89_EMI_Release_Version%
set power_mode=_PowerDCDC
set program_mode=_ProgramFlashSRAM

if not exist %~dp0output\!chip_name! mkdir %~dp0output\!chip_name!

REM set as EMI_DEMO
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_config.h TEST_DEMO EMI_DEMO

REM set as SRAM&FLASH program 
call %~dp0.\tool st_node %Project_Path_Compile%\.cproject %~dp0config\config.ini B85m_Boot_Macro %EMI_compile_option%:-DMCU_STARTUP_FLASH:-DMCU_STARTUP_SRAM

call :B80_B89_Generate_Bin_Unit2

)

setlocal DisableDelayedExpansion
goto :END


:B85m_Generate_Bin_Unit1

REM Internal Cap
set cap_switch=_CapInternal
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_emi.c CLOSE_INTERNAL_CAP 0
set emi_bin_name=!chip_name!_EMI!cap_switch!!power_mode!!calibration_position!!program_mode!!release_version!

call %~dp0.\show_info "Compile %EMI_compile_option% !chip_name!" 1
call :COMPILE_PROCESS
copy %Project_Path_Compile%\%EMI_compile_option%\%EMI_compile_option%.bin %~dp0output\!chip_name!\!emi_bin_name!.bin
call %~dp0.\show_info "Generate !emi_bin_name!.bin successfully!" 4

REM External Cap
set cap_switch=_CapExternal
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_emi.c CLOSE_INTERNAL_CAP 1
set emi_bin_name=!chip_name!_EMI!cap_switch!!power_mode!!calibration_position!!program_mode!!release_version!

call %~dp0.\show_info "Compile %EMI_compile_option% !chip_name!" 1
call :COMPILE_PROCESS
copy %Project_Path_Compile%\%EMI_compile_option%\%EMI_compile_option%.bin %~dp0output\!chip_name!\!emi_bin_name!.bin
call %~dp0.\show_info "Generate !emi_bin_name!.bin successfully!" 4

goto :END


:B80_B89_Generate_Bin_Unit2

REM Calibration position Flash
set calibration_position=_CalibFlash
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_emi.c READ_OFFSET_CLIBRATION_OTP 0
call :B85m_Generate_Bin_Unit1

REM Calibration position OTP
set calibration_position=_CalibOTP
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\app_emi.c READ_OFFSET_CLIBRATION_OTP 1
call :B85m_Generate_Bin_Unit1

goto :END


:B87_Generate_Bin_Unit2

REM POWER DCDC
set power_mode=_PowerDCDC
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\main.c POWER_MODE_SELECT POWER_MODE_DCDC
call :B85m_Generate_Bin_Unit1

REM POWER LDO
set power_mode=_PowerLDO
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\main.c POWER_MODE_SELECT POWER_MODE_LDO
call :B85m_Generate_Bin_Unit1

REM POWER DCDC_LDO
set power_mode=_PowerDCDCLDO
call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\demo\vendor\%EMI_compile_option%\main.c POWER_MODE_SELECT POWER_MODE_DCDC_LDO
call :B85m_Generate_Bin_Unit1
goto :END


:COMPILE_PROCESS

if exist %Project_Path_Compile%\%EMI_compile_option% (rmdir /s /q %Project_Path_Compile%\%EMI_compile_option%)
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_Compile% %Test_Compile_CompileOption% %EMI_compile_option%
if exist %Project_Path_Compile%\%EMI_compile_option%\*.bin (
call %~dp0.\show_info "PASS" 2
) else (
call %~dp0.\show_info "FAIL" 2
pause
)

goto :END



:REMOVE_TEMP_FILE

if exist %~dp0.\~workspace (rmdir /s /q %~dp0.\~workspace)
if exist %~dp0.\customize_chip_profile (rmdir /s /q %~dp0.\customize_chip_profile)
if exist %~dp0.\~cfgTmp.txt (del /s /q /f %~dp0.\~cfgTmp.txt)
goto :END

:MAIN

call :REMOVE_TEMP_FILE
echo ------BEGIN(%date%  %time%)--------
call :TEST_COMPILE
echo ------END(%date%  %time%)---------
call :REMOVE_TEMP_FILE
pause

:END