: No log file Execute
@start	/b	dd2015_network_win.exe -q -port=2016 -verbose -dump
:@start	/b	taskexec_win.exe -loopback -vga_line 2
@start	/b	taskexec_win.exe -vga_line 2
@start	/b	servocon_win.exe -e -l 10 -ntp -ns
robotmgr_win.exe /d /k /wi=3%*
@dandy_killer_win.exe -name=DD2015_NETWORK -code=0 -value=0

pause