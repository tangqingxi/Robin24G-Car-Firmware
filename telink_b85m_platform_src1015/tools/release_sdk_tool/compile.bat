REM set TC32_ECLIPSE_BUILD=%~1\eclipse.exe -console -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild --launcher.suppressErrors
set TC32_ECLIPSE_BUILD=%~1\TelinkIoTStudio.exe -console -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild
set workspace=%~dp0.\~workspace\
set project_path=%~2
set project_name=%~3
set build_configuration_name=%~4

%TC32_ECLIPSE_BUILD% -data %workspace% -import %project_path% -cleanBuild %project_name%/%build_configuration_name%

cd /d %~dp0