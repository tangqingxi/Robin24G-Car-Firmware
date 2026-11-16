@echo off
REM ##################################################################################################################
REM ################################[Instructions for use]############################################################
REM STEP1: Modify TOOL_PATH_RLEASE_SDK to the installation path of TC32 IDE, be careful not to have spaces in the path.
REM STEP2: Modify release_B8x_flag, if you want to release the corresponding SDK, set it to 1, otherwise set it to 0.
REM STEP3: Modify exc_xxxxx_flag as required, where "mklib" is the compilation library, "rmopt" is the deletion of the 
REM        compilation option, "rmlnk" is the deletion of the file link in the project, "rmfile" is the deletion of the
REM        file, and "tstcmp" is the test compilation. Set to 1 to execute the item, otherwise not execute.
REM STEP4: Modify config/cfg.ini, rm_options is the compile option to be deleted, rm_links is the link of the file to 
REM        be deleted, rm_files is the file to be deleted, lib is the name of the file to be compiled into the library.
REM STEP5: Double click this file to run.
REM ##################################################################################################################

set TOOL_PATH_RLEASE_SDK=C:\TelinkIoTStudio
set Project_Path_Rlease_SDK=..\..\
set make_lib_compile_option=Debug_Demo
set release_B80_flag=0
set release_B80B_flag=0
set release_B85_flag=0
set release_B87_flag=0
set release_B89_flag=0
set release_TC321X_flag=0
set release_TC1211_flag=1
set release_tc122x_flag=0
set exc_mklib_flag=1
set exc_rmopt_flag=1
set exc_rmlnk_flag=1
set exc_rmfile_flag=1
set exc_tstcmp_flag=0

set Project_Path_B80=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B80
set Project_Path_B80B=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B80B
set Project_Path_B85=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B85
set Project_Path_B87=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B87
set Project_Path_B89=%Project_Path_Rlease_SDK%\project\tlsr_tc32\B89
set Project_Path_TC321X=%Project_Path_Rlease_SDK%\project\tlsr_tc32\TC321X
set Project_Path_TC1211=%Project_Path_Rlease_SDK%\project\tlsr_tc32\TC1211
set Project_Path_tc122x=%Project_Path_Rlease_SDK%\project\tlsr_tc32\tc122x

REM If you need to customize the B80 chip, please set the following parameters, 0: P0, 5: P5, 10: P10, 12: P12, 255:ingore
set B80_SDK_VERSION=255
set B80B_SDK_VERSION=255

goto :MAIN


:TEST_COMPILE
set Project_Path_Compile=%Project_Path_B80%
set Test_Compile_CompileOption=B80_Driver_Demo
if %release_B80_flag% == 1 (
call %~dp0.\show_info "Compile Test B80" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test B80 Completed!" 4
)

set Project_Path_Compile=%Project_Path_B80B%
set Test_Compile_CompileOption=B80B_Driver_Demo
if %release_B80B_flag% == 1 (
call %~dp0.\show_info "Compile Test B80B" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test B80B Completed!" 4
)

set Project_Path_Compile=%Project_Path_B85%
set Test_Compile_CompileOption=B85_Driver_Demo
if %release_B85_flag% == 1 (
call %~dp0.\show_info "Compile Test B85" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test B85 Completed!" 4
)

set Project_Path_Compile=%Project_Path_B87%
set Test_Compile_CompileOption=B87_Driver_Demo
if %release_B87_flag% == 1 (
call %~dp0.\show_info "Compile Test B87" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test B87 Completed!" 4
)

set Project_Path_Compile=%Project_Path_B89%
set Test_Compile_CompileOption=B89_Driver_Demo
if %release_B89_flag% == 1 (
call %~dp0.\show_info "Compile Test B89" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test B89 Completed!" 4
)

set Project_Path_Compile=%Project_Path_TC321X%
set Test_Compile_CompileOption=TC_PLATFORM_SDK_321X
if %release_TC321X_flag% == 1 (
call %~dp0.\show_info "Compile Test TC321X" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test TC321X Completed!" 4
)

set Project_Path_Compile=%Project_Path_TC1211%
set Test_Compile_CompileOption=TC_PLATFORM_SDK_1211
if %release_TC1211_flag% == 1 (
call %~dp0.\show_info "Compile Test TC1211" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test TC1211 Completed!" 4
)

set Project_Path_Compile=%Project_Path_tc122x%
set Test_Compile_CompileOption=TC_PLATFORM_SDK_122X
if %release_tc122x_flag% == 1 (
call %~dp0.\show_info "Compile Test tc122x" 1
call :COMPILE_PROCESS
call %~dp0.\show_info "Compile Test tc122x Completed!" 4
)

goto :END

:COMPILE_PROCESS
call %~dp0.\tool rd_options %Project_Path_Compile% .cproject %~dp0.\~cfgTmp.txt
for /f %%i in (%~dp0.\~cfgTmp.txt) do (
call %~dp0.\show_info "Compile %%i." 3
if exist %Project_Path_Compile%\%%i (rmdir /s /q %Project_Path_Compile%\%%i)
call compile %TOOL_PATH_RLEASE_SDK% %Project_Path_Compile% %Test_Compile_CompileOption% %%i
if exist %Project_Path_Compile%\%%i\*.bin (
call %~dp0.\show_info "PASS" 2
) else (
call %~dp0.\show_info "FAIL" 2
pause
)
if exist %Project_Path_Compile%\%%i (rmdir /s /q %Project_Path_Compile%\%%i)
)
goto :END

:MAKE_LIB
call %~dp0.\show_info "Make library" 1
REM if %release_B80_flag% == 1 (
REM copy %~dp0.\config\libB80.a %Project_Path_Rlease_SDK%\chip\B80\drivers\lib\src\libdriver.a
REM )
REM if %release_B85_flag% == 1 (
REM copy %~dp0.\config\libB85.a %Project_Path_Rlease_SDK%\chip\B85\drivers\lib\src\libdriver.a
REM )
REM if %release_B87_flag% == 1 (
REM copy %~dp0.\config\libB87.a %Project_Path_Rlease_SDK%\chip\B87\drivers\lib\src\libdriver.a
REM )
REM if %release_B89_flag% == 1 (
REM copy %~dp0.\config\libB89.a %Project_Path_Rlease_SDK%\chip\B89\drivers\lib\src\libdriver.a
REM )
set build_prj_Rlease_SDK=%make_lib_compile_option%

REM make libray for B80
set Project_Path_MakeLib_Complie=%Project_Path_B80%
set MakeLib_CompileOption=B80_Driver_Demo
if %release_B80_flag% == 1 (
REM B80
call %~dp0.\show_info "Compile B80 %build_prj_Rlease_SDK%." 3
REM call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\chip\B80\drivers\lib\src\pm.c OTP_PROTECTION_CODE_SELECT OTP_PROTECTION_CODE_S0
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\show_info "Generate libdriver.a" 3
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% B80 0
if %B80_SDK_VERSION% == 255 (
copy %~dp0..\release_lib\B80\libdriver.a %~dp0..\..\chip\B80\drivers\lib\libdriver_b80.a
)
) else (
goto :MAKE_LIB_ERROR
)
)

REM make libray for B80B
set Project_Path_MakeLib_Complie=%Project_Path_B80B%
set MakeLib_CompileOption=B80B_Driver_Demo
if %release_B80B_flag% == 1 (


call %~dp0.\show_info "Compile B80B %build_prj_Rlease_SDK%." 3
REM call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\chip\B80\drivers\lib\src\pm.c OTP_PROTECTION_CODE_SELECT OTP_PROTECTION_CODE_S0
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\show_info "Generate libdriver.a" 3
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% B80B 0



if %B80B_SDK_VERSION% == 255 (
copy %~dp0..\release_lib\B80B\libdriver.a %~dp0..\..\chip\B80\drivers\lib\libdriver_b80b.a
)
REM if %B80B_SDK_VERSION% == 1 (
REM copy %~dp0..\release_lib\B80B\libdriver_s1.a %~dp0..\..\chip\B80B\drivers\lib\src\libdriver.a
REM )
REM if %B80B_SDK_VERSION% == 2 (
REM copy %~dp0..\release_lib\B80B\libdriver_s2.a %~dp0..\..\chip\B80B\drivers\lib\src\libdriver.a
REM )
REM if %B80B_SDK_VERSION% == 3 (
REM copy %~dp0..\release_lib\B80B\libdriver_s3.a %~dp0..\..\chip\B80B\drivers\lib\src\libdriver.a
REM )


) else (
goto :MAKE_LIB_ERROR
)
)

REM make libray for B85
set Project_Path_MakeLib_Complie=%Project_Path_B85%
set MakeLib_CompileOption=B85_Driver_Demo
if %release_B85_flag% == 1 (
call %~dp0.\show_info "Compile B85 %build_prj_Rlease_SDK%." 3
REM call %~dp0.\tool st_macro %Project_Path_Rlease_SDK%\chip\B85\drivers\lib\src\pm.c SDK_VERSION_SELECT SDK_VERSION_CHECK
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% B85 1
copy %~dp0..\release_lib\B85\libdriver.a %~dp0..\..\chip\B85\drivers\lib\libdriver.a
) else (
goto :MAKE_LIB_ERROR
)
)

REM make libray for B87
set Project_Path_MakeLib_Complie=%Project_Path_B87%
set MakeLib_CompileOption=B87_Driver_Demo
if %release_B87_flag% == 1 (
call %~dp0.\show_info "Compile B87 %build_prj_Rlease_SDK%." 3
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% B87 1
copy %~dp0..\release_lib\B87\libdriver.a %~dp0..\..\chip\B87\drivers\lib\libdriver.a
) else (
goto :MAKE_LIB_ERROR
)
)

REM make libray for B89
set Project_Path_MakeLib_Complie=%Project_Path_B89%
set MakeLib_CompileOption=B89_Driver_Demo
if %release_B89_flag% == 1 (
call %~dp0.\show_info "Compile B89 %build_prj_Rlease_SDK%." 3
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% B89 1
copy %~dp0..\release_lib\B89\libdriver.a %~dp0..\..\chip\B89\drivers\lib\libdriver.a
) else (
goto :MAKE_LIB_ERROR
)
)

REM make libray for TC321X
set Project_Path_MakeLib_Complie=%Project_Path_TC321X%
set MakeLib_CompileOption=TC_PLATFORM_SDK_321X
if %release_TC321X_flag% == 1 (
call %~dp0.\show_info "Compile TC321X %build_prj_Rlease_SDK%." 3
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% TC321X 1
copy %~dp0..\release_lib\TC321X\libdriver.a %~dp0..\..\chip\TC321X\drivers\lib\libdriver_tc321x.a
) else (
goto :MAKE_LIB_ERROR
)
)

REM make libray for TC321X
set Project_Path_MakeLib_Complie=%Project_Path_TC1211%
set MakeLib_CompileOption=TC_PLATFORM_SDK_1211
if %release_TC1211_flag% == 1 (
call %~dp0.\show_info "Compile TC1211 %build_prj_Rlease_SDK%." 3
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% TC1211 1
copy %~dp0..\release_lib\TC1211\libdriver.a %~dp0..\..\chip\TC1211\drivers\lib\libdriver_tc1211.a
) else (
goto :MAKE_LIB_ERROR
)
)

REM make libray for tc122x
set Project_Path_MakeLib_Complie=%Project_Path_tc122x%
set MakeLib_CompileOption=TC_PLATFORM_SDK_122X
if %release_tc122x_flag% == 1 (
call %~dp0.\show_info "Compile tc122x %build_prj_Rlease_SDK%." 3
call %~dp0.\compile %TOOL_PATH_RLEASE_SDK% %Project_Path_MakeLib_Complie% %MakeLib_CompileOption% %build_prj_Rlease_SDK%
if exist %Project_Path_MakeLib_Complie%\%build_prj_Rlease_SDK%\*.bin (
call %~dp0.\show_info "PASS" 2
call %~dp0.\make_lib %TOOL_PATH_RLEASE_SDK% %Project_Path_Rlease_SDK% %build_prj_Rlease_SDK% tc122x 1
copy %~dp0..\release_lib\tc122x\libdriver.a %~dp0..\..\chip\tc122x\drivers\lib\libdriver_tc122x.a
) else (
goto :MAKE_LIB_ERROR
)
)

call %~dp0.\show_info "Make library Completed!" 4
goto :END

:MAKE_LIB_ERROR
call %~dp0.\show_info "Make library Failed!" 4
call %~dp0.\show_info "FAIL" 2
pause
exit
goto :END


:REMOVE_COMPILE_OPTIONS

call %~dp0.\show_info "Remove Compile Options" 1
if %release_B80_flag% == 1 (
call %~dp0.\show_info "Remove B80 Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_B80% %~dp0.\config\cfg.ini B80 .\.cproject
call %~dp0.\show_info "Remove B80 Compile Options Completed!" 4
)
if %release_B80B_flag% == 1 (
call %~dp0.\show_info "Remove B80B Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_B80B% %~dp0.\config\cfg.ini B80B .\.cproject
call %~dp0.\show_info "Remove B80B Compile Options Completed!" 4
)
if %release_B85_flag% == 1 (
call %~dp0.\show_info "Remove B85 Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_B85% %~dp0.\config\cfg.ini B85 .\.cproject
call %~dp0.\show_info "Remove B85 Compile Options Completed!" 4
)
if %release_B87_flag% == 1 (
call %~dp0.\show_info "Remove B87 Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_B87% %~dp0.\config\cfg.ini B87 .\.cproject
call %~dp0.\show_info "Remove B87 Compile Options Completed!" 4
)
if %release_B89_flag% == 1 (
call %~dp0.\show_info "Remove B89 Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_B89% %~dp0.\config\cfg.ini B89 .\.cproject
call %~dp0.\show_info "Remove B89 Compile Options Completed!" 4
)
if %release_TC321X_flag% == 1 (
call %~dp0.\show_info "Remove TC321X Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_TC321X% %~dp0.\config\cfg.ini TC321X .\.cproject
call %~dp0.\show_info "Remove TC321X Compile Options Completed!" 4
)
if %release_TC1211_flag% == 1 (
call %~dp0.\show_info "Remove TC1211 Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_TC321X% %~dp0.\config\cfg.ini TC1211 .\.cproject
call %~dp0.\show_info "Remove TC1211 Compile Options Completed!" 4
)
if %release_tc122x_flag% == 1 (
call %~dp0.\show_info "Remove tc122x Compile Options" 3
call %~dp0.\tool rm_options %Project_Path_tc122x% %~dp0.\config\cfg.ini tc122x .\.cproject
call %~dp0.\show_info "Remove tc122x Compile Options Completed!" 4
)
goto :END


:REMOVE_FILE_LINKS

call %~dp0.\show_info "Remove File Links" 1

if %release_B80_flag% == 1 (
call %~dp0.\show_info "Remove B80 File Links" 3
call %~dp0.\tool rm_links %Project_Path_B80% %~dp0.\config\cfg.ini B80 .\.project
call %~dp0.\show_info "Remove B80 File Links Completed!" 4
)
if %release_B80B_flag% == 1 (
call %~dp0.\show_info "Remove B80B File Links" 3
call %~dp0.\tool rm_links %Project_Path_B80B% %~dp0.\config\cfg.ini B80B .\.project
call %~dp0.\show_info "Remove B80B File Links Completed!" 4
)
if %release_B85_flag% == 1 (
call %~dp0.\show_info "Remove B85 File Links" 3
call %~dp0.\tool rm_links %Project_Path_B85% %~dp0.\config\cfg.ini B85 .\.project
call %~dp0.\show_info "Remove B85 File Links Completed!" 4
)
if %release_B87_flag% == 1 (
call %~dp0.\show_info "Remove B87 File Links" 3
call %~dp0.\tool rm_links %Project_Path_B87% %~dp0.\config\cfg.ini B87 .\.project
call %~dp0.\show_info "Remove B87 File Links Completed!" 4
)
if %release_B89_flag% == 1 (
call %~dp0.\show_info "Remove B89 File Links" 3
call %~dp0.\tool rm_links %Project_Path_B89% %~dp0.\config\cfg.ini B89 .\.project
call %~dp0.\show_info "Remove B89 File Links Completed!" 4
)
if %release_TC321X_flag% == 1 (
call %~dp0.\show_info "Remove TC321X File Links" 3
call %~dp0.\tool rm_links %Project_Path_TC321X% %~dp0.\config\cfg.ini TC321X .\.project
call %~dp0.\show_info "Remove TC321X File Links Completed!" 4
)
if %release_TC1211_flag% == 1 (
call %~dp0.\show_info "Remove TC1211 File Links" 3
call %~dp0.\tool rm_links %Project_Path_TC321X% %~dp0.\config\cfg.ini TC1211 .\.project
call %~dp0.\show_info "Remove TC1211 File Links Completed!" 4
)
if %release_tc122x_flag% == 1 (
call %~dp0.\show_info "Remove tc122x File Links" 3
call %~dp0.\tool rm_links %Project_Path_tc122x% %~dp0.\config\cfg.ini tc122x .\.project
call %~dp0.\show_info "Remove tc122x File Links Completed!" 4
)
goto :END

:REMOVE_FILES
call %~dp0.\show_info "Remove Files" 1
if %release_B80_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B80
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B80 chip\B80\drivers\driver.h
)
if %release_B80B_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B80B
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B80B chip\B80\drivers\driver.h
)
if %release_B85_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B85
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B85 chip\B85\drivers\driver.h
)
if %release_B87_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B87
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B87 chip\B87\drivers\driver.h
)
if %release_B89_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B89
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini B89 chip\B89\drivers\driver.h
)
if %release_TC321X_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini TC321X
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini TC321X chip\TC321X\drivers\driver.h
)
if %release_TC1211_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini TC1211
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini TC1211 chip\TC1211\drivers\driver.h
)

if %release_tc122x_flag% == 1 (
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini tc122x
call %~dp0.\tool rm_files %Project_Path_Rlease_SDK% %~dp0.\config\cfg.ini tc122x chip\tc122x\drivers\driver.h
)

call %~dp0.\show_info "Remove Files Completed!" 4
goto :END

:REMOVE_TEMP_FILE
if exist %~dp0.\~workspace (rmdir /s /q %~dp0.\~workspace)
if exist %~dp0.\customize_chip_profile (rmdir /s /q %~dp0.\customize_chip_profile)
if exist %~dp0.\~cfgTmp.txt (del /s /q /f %~dp0.\~cfgTmp.txt)
goto :END

:MAIN
call :REMOVE_TEMP_FILE
echo ------BEGIN(%date%  %time%)--------
if %exc_mklib_flag% == 1 (
call :MAKE_LIB
)
if %exc_rmopt_flag% == 1 (
call :REMOVE_COMPILE_OPTIONS
)
if %exc_rmlnk_flag% == 1 (
call :REMOVE_FILE_LINKS
)
if %exc_rmfile_flag% == 1 (
call :REMOVE_FILES
)
if %exc_tstcmp_flag% == 1 (
if exist %~dp0.\~workspace (rmdir /s /q %~dp0.\~workspace)
call :TEST_COMPILE
)
echo ------END(%date%  %time%)---------
call :REMOVE_TEMP_FILE
pause

:END