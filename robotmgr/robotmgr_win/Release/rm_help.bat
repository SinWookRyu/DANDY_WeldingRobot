: No log file Execute
@start	/b	dd2015_network_win.exe -q -port=2016 -verbose -dump
:@start	/b	taskexec_win.exe -loopback
:@start	/b	servocon_win.exe
robotmgr_win.exe /?
:robotmgr_win.exe /k /wi=1 /d /a job1 job2 job3%*
:@dandy_killer_win.exe -name=DD2015_NETWORK -code=0 -value=0

:RM KeyInput Enable Option '/key' or '/k'
:RM KeyInput Disable Option 'nokey' or '/nk'
:robotmgr_win.exe/nk %*

: Write log file Execute
:@start	/b	taskexec_win.exe > output_te.log
:@start	/b	servocon_win.exe > output_sc.log
:robotmgr_win.exe %* > output_rm.log

:protect dos window close
pause