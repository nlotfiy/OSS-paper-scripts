unset mouse
set term wxt 0 size 1400, 1300 font ',14'
set multiplot layout 3,1 # This feature requires gnuplot v4.2 or later

datafile0 = 'TwoLinKrobotLaGrange.txt'

set lmarg 12
set grid
set xlabel 'Time, sec'

set ylabel 'Angular Position, rad'
plot datafile0 using 1:2  title 'Angle1'  with lines

plot datafile0 using 1:3 title 'Angle2' with lines
	
set ylabel 'Angular Velocity, rad/s'
plot datafile0 using 1:4 title 'Omega1' with lines,\
	datafile0 using 1:5 title 'Omega2' with lines
unset multiplot

set term wxt 1 size 1400, 1300 font ',14'
#set multiplot layout 1,1 # This feature requires gnuplot v4.2 or later

set xlabel 'x-position, m'
set ylabel 'y-position, m'
plot datafile0 using 8:9 title 'Joint2' with lines,\
	datafile0 using 12:13 title 'Traj. Circle' with lines

set term wxt 2 size 1400, 1300 font ',14'
set xlabel 'Time, sec'
set ylabel 'Angular Position, rad'
plot datafile0 using 1:2  title 'Angle1'  with lines,\
	datafile0 using 1:10  title 'AngleSet1'  with lines,\
	datafile0 using 1:3 title 'Angle2' with lines,\
	datafile0 using 1:11  title 'AngleSet2'  with lines

pause -1
exit gnuplot
#set mouse
#pause mouse keypress