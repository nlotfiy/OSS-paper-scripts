unset mouse
set term wxt 0 size 2000, 1800 font ',14'
set multiplot layout 3,1 # This feature requires gnuplot v4.2 or later

datafile0 = 'DCMotorOpenLoopData.txt'

set lmarg 12
set grid
set xlabel 'Time, sec'
set ylabel 'Speed, rad/s'
plot datafile0 using 1:3  title 'Rotor Speed'  with lines

set ylabel 'Angular Position, rad'
plot datafile0 using 1:2  title 'Angle'  with lines
	
set ylabel 'Current'
plot datafile0 using 1:4 title 'Current' with lines

unset multiplot

#pause -1
set mouse
pause mouse keypress