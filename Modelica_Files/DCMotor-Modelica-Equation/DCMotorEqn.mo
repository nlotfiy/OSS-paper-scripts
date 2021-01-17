model DCMotorEqn
  import SI = Modelica.SIunits;
  // Motor parameters
  parameter SI.Resistance R = 2.62 " Ohm";
  parameter SI.Voltage v = 24.0 " v";
  parameter SI.ElectricalTorqueConstant Kt = 0.66 "N.m/A";
  parameter SI.ElectricalTorqueConstant Kb = Kt "N.m/A, equal to Kt for consistent units";
  parameter SI.MomentOfInertia J = 0.0026 "kg.m^2";
  parameter SI.RotationalDampingConstant b = 0.01 "N.m.s/rad";
  parameter SI.Inductance L = 0.05 " H";
  // State variables
  SI.Angle theta(start = 0.0) "Shaft angle, rad";
  SI.AngularVelocity omega(start = 0.0) "Shaft angular velocity, rad/s";
  SI.Current i(start = 0.0) "Current in circuit, A";
equation
  der(theta) = omega;
  der(omega) = ((-b * omega) + Kt * i) / J;
  der(i) = ((-Kb * omega) - R * i + v) / L;
  annotation(
    experiment(StartTime = 0, StopTime = 0.5, Tolerance = 1e-06, Interval = 0.001));
end DCMotorEqn;
