: No log file Execute
:@start	/b	dd2015_network_win_d.exe -port=2016 -verbose -dump
:@start	/b	taskexec_win_d.exe -loopback
@start	/b	servocon_win_d.exe -i
robotmgr_win.exe /k /wi=1 /d /nv%*
:robotmgr_win_d.exe /k /wi=1 /d /a job1 job2 job3%*
:robotmgr_win_d.exe /k /wi=1 /nv /d%*
:@dandy_killer_win_d.exe -name=DD2015_NETWORK -code=0 -value=0


:RM KeyInput Enable Option '/key' or '/k'
:RM KeyInput Disable Option 'nokey' or '/nk'
:robotmgr_win_d.exe/nk %*

: Write log file Execute
:@start	/b	taskexec_win_d.exe > output_te.log
:@start	/b	servocon_win_d.exe > output_sc.log
:robotmgr_win_d.exe %* > output_rm.log

:protect dos window close
:pause