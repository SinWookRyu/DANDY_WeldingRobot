: No log file Execute
:@start	/b	dd2015_network_win_d.exe -q -port=2016 -verbose -dump
:@start	/b	taskexec_win_d.exe -loopback -vga_line 2
@start	/b	taskexec_win_d.exe -vga_line 0
:@start	/b	servocon_win_d.exe -e -l 0 -ntp -ns
:robotmgr_win_d.exe /k /wi=3 /d %*
:@dandy_killer_win_d.exe -name=DD2015_NETWORK -code=0 -value=0

:protect dos window close
:pause