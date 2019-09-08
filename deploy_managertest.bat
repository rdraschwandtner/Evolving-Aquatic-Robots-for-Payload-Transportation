@echo off
svn checkout "file:///FULLPATH/managertest_Water_Snake" temp_managertest_water_snake

rem 2)
svnversion ./temp_managertest_water_snake/ > svnversion.txt
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
7z a -tzip managertest-%svnversionname% .\temp_managertest_water_snake\*

rem 4)
rem upload the compressed file to robonodes
pscp -sftp -pw InsertPassword managertest-%svnversionname%.zip rened@robo1.cse.msu.edu:./workspace/

rem 5)
rem uncompress the file
echo cd workspace > puttyrobonodes1commands.txt
echo unzip managertest-%svnversionname%.zip -d managertest-%svnversionname%/ >> puttyrobonodes1commands.txt

echo cd managertest-%svnversionname%/Experiments/Spheretouch/ >> puttyrobonodes1commands.txt
rem make launch scripts executeable
echo chmod 755 ./launch_runs.sh >> puttyrobonodes1commands.txt
echo chmod 755 ./run_launcher.sh >> puttyrobonodes1commands.txt
echo chmod 755 ./evolutionary_progression_logging.sh >> puttyrobonodes1commands.txt
rem EXECUTE launch script in screen
rem http://unix.stackexchange.com/questions/47271/prevent-gnu-screen-from-terminating-session-once-executed-script-ends
rem -dm: starts screen in detached mode
rem ATTENTION set LIBPATH variable in .screenrc!!!!!!!!!!!!!!!!!!!!!!!
rem sh -c '...': running multiple commands
rem exec bash: prevent screen from terminating after script finished
echo screen -dm sh -c './launch_runs.sh 0 5 %svnversionname%; exec bash' >> puttyrobonodes1commands.txt

putty -ssh rened@robo1.cse.msu.edu -pw InsertPassword -m puttyrobonodes1commands.txt
del puttyrobonodes1commands.txt
echo started launch script on robonodes 1

rmdir /S /Q temp_managertest_water_snake
del managertest-%svnversionname%.zip