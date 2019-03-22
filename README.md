# DANDY Robot Controller Code
Open DANDY/DANDY_2015/DANDY_2015.sln using Visual Studio

Total Code are composed with 3 processes (robotmgr, servocon, taskexec)

3 processes are operated through IPC (message passing & shared memory)

Code can be executed under windows & QNX compatible

The location of code is 'DANDY/robotmgr/robotmgr_src/', 'DANDY/servocon/servocon_src/ ', 'DANDY/taskexec/taskexec_src/'

 - robotmgr: Robot Manager, scheduling the process sync, parameter management, network broker etc

 - taskexec: motion control, kinematics, i/o control calculation etc

 - servocon: ethercat interface, control command generation etc
