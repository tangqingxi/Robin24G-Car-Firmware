set tool_path_make_lib=%~1\opt\tc32\bin
set lib_obj_name_release=libdriver%~6.a
set chip_type=%~4
set rm_file_flag=%~5

goto :MAIN_MAKE_LIB



:MAIN_MAKE_LIB
set file_path=%~2\project\tlsr_tc32\%~4\%~3\drivers\lib\src
set lib_obj_name=lib%~4%~6.a
set lib_save_path=%~2\chip\%~4\drivers\lib\src
set lib_release_path=%~dp0..\release_lib\%~4
cd /d %~dp0
call read_cfg .\config\cfg.ini %~4 lib
call :PROCESS_MAKE_LIB

goto :END


:PROCESS_MAKE_LIB
set mk_lib_temp= 
setlocal EnableDelayedExpansion
for /f %%i in (.\~cfgTmp.txt) do (
set mk_lib_temp=!mk_lib_temp!%file_path%\%%i.o 
)

if exist %lib_release_path%\%lib_obj_name_release% (
del /s /q /f %lib_release_path%\%lib_obj_name_release%
)

REM echo !mk_lib_temp!
%tool_path_make_lib%\tc32-elf-ar.exe -r %file_path%\%lib_obj_name% !mk_lib_temp!
if exist %file_path%\%lib_obj_name% (
call %~dp0.\show_info "Generate %lib_obj_name_release% successfully." 
REM copy %file_path%\%lib_obj_name% %lib_save_path%\%lib_obj_name_release%
REM copy %file_path%\%lib_obj_name% %~dp0.\config\%lib_obj_name%
copy %file_path%\%lib_obj_name% %lib_release_path%\%lib_obj_name_release%
) else (
call %~dp0.\show_info "Generate %lib_obj_name_release% Failed." 
)

if %rm_file_flag% == 1 (

if exist %lib_release_path%\%lib_obj_name_release% (
for /f %%i in (%~dp0.\~cfgTmp.txt) do (
if exist %lib_save_path%\%%i.c del /s /q /f %lib_save_path%\%%i.c
)
)

)
cd /d %~dp0
setlocal DisableDelayedExpansion
goto :END


:END



