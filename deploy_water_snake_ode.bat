@echo off
svn checkout "file:///FULLPATH/Water_Snake_ODE" temp_water_snake_ode

rem 2)
svnversion ./temp_water_snake_ode/ > svnversion.txt
set /p svnvers=<svnversion.txt
echo Currentversion of the project is %svnvers%
rem ":" cannot be used in a filename -> replace it

rem take the first part before ":"
set "rev_var=%svnvers::="^&REM #%
rem echo %rev_var%

rem take the second part after ":"
set "wc_var=%svnvers:*:=%"
rem echo %wc_var%

set svnversionname=rev%rev_var%wc%wc_var%
echo Generated name=%svnversionname%

del svnversion.txt

rem 3)
rem compress the checkout to a zip file
7z a -tzip water_snake_ode-%svnversionname% .\temp_water_snake_ode\*

rem 4)
rem upload the compressed file to robonodes
pscp -sftp -pw InsertPassword water_snake_ode-%svnversionname%.zip rened@robo9.cse.msu.edu:./workspace/


rem 5)
rem uncompress the file
echo cd workspace > puttyrobonodes9commands.txt
echo unzip water_snake_ode-%svnversionname%.zip -d water_snake_ode-%svnversionname%/ >> puttyrobonodes9commands.txt
rem start the launchscript
echo cd water_snake_ode-%svnversionname%/Experiments/ANN_Worm/ >> puttyrobonodes9commands.txt
rem make launch scripts executeable
echo chmod 755 ./launch_runs.sh >> puttyrobonodes9commands.txt
echo chmod 755 ./run_launcher.sh >> puttyrobonodes9commands.txt
echo chmod 755 ./validate_best.sh >> puttyrobonodes9commands.txt
echo chmod 755 ./evolutionary_progression_logging.sh >> puttyrobonodes9commands.txt
rem EXECUTE launch script in screen
rem http://unix.stackexchange.com/questions/47271/prevent-gnu-screen-from-terminating-session-once-executed-script-ends
rem -dm: starts screen in detached mode
rem ATTENTION set LIBPATH variable in .screenrc!!!!!!!!!!!!!!!!!!!!!!!
rem sh -c '...': running multiple commands
rem exec bash: prevent screen from terminating after script finished
echo screen -dm sh -c './launch_runs.sh 0 2 %svnversionname%; exec bash' >> puttyrobonodes9commands.txt

rem 6)
putty -ssh rened@robo9.cse.msu.edu -pw InsertPassword -m puttyrobonodes9commands.txt
rem del puttyrobonodes1commands.txt
echo started launch script on robonodes 10
del puttyrobonodes9commands.txt

rmdir /S /Q temp_water_snake_ode
del water_snake_ode-%svnversionname%.zip