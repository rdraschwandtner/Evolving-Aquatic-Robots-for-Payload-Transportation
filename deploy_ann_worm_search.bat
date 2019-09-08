@echo off
svn checkout "file:///FULLPATH/behavior" temp_behavior

rem 2)
svnversion ./temp_behavior/ > svnversion.txt
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
7z a -tzip temp_behavior-%svnversionname% .\temp_behavior\*

rem 4)
rem upload the compressed file to robonodes
pscp -sftp -pw InsertPassword temp_behavior-%svnversionname%.zip rened@robo1.cse.msu.edu:./workspace/


rem 5)
rem uncompress the file
echo cd workspace > puttyrobonodes1commands.txt
echo unzip temp_behavior-%svnversionname%.zip -d behavior-%svnversionname%/ >> puttyrobonodes1commands.txt
rem start the launchscript
echo cd behavior-%svnversionname%/Experiments/ANN_Worm_Search/ >> puttyrobonodes1commands.txt
rem make launch scripts executeable
echo chmod 755 ./launch_runs.sh >> puttyrobonodes1commands.txt
echo chmod 755 ./run_launcher.sh >> puttyrobonodes1commands.txt
rem EXECUTE launch script in screen
rem http://unix.stackexchange.com/questions/47271/prevent-gnu-screen-from-terminating-session-once-executed-script-ends
rem -dm: starts screen in detached mode
rem ATTENTION set LIBPATH variable in .screenrc!!!!!!!!!!!!!!!!!!!!!!!
rem sh -c '...': running multiple commands
rem exec bash: prevent screen from terminating after script finished
echo screen -dm sh -c './launch_runs.sh 0 0 %svnversionname%; exec bash' >> puttyrobonodes1commands.txt

rem 6)
putty -ssh rened@robo1.cse.msu.edu -pw InsertPassword -m puttyrobonodes1commands.txt
rem del puttyrobonodes1commands.txt
echo started launch script on robonodes 1
del puttyrobonodes1commands.txt

rmdir /S /Q temp_behavior
del temp_behavior-%svnversionname%.zip