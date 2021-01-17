model TwoLinkRobotEqnCntrl
  import SI = Modelica.SIunits;
  import CON = Modelica.Constants;
  // Parameters
  parameter SI.Length L1 = 0.25, L2 = L1 "m";
  parameter SI.Length r1 = L1 / 2.0, r2 = L2 / 2.0;
  parameter SI.Mass m1 = 0.5, m2 = m1 "kg";
  parameter SI.Acceleration g = 9.81 "m/s^2";
  parameter SI.RotationalDampingConstant b1 = 0.1, b2 = b1 "Nm-s/rad";
  // Derived parameters
  parameter SI.MomentOfInertia I1 = m1 * L1 ^ 2 / 12.0, I2 = m2 * L2 ^ 2 / 12.0;
  parameter Real alpha = I1 + I2 + m1 * r1 ^ 2 + m2 * (L1 ^ 2 + r2 ^ 2);
  parameter Real beta = m2 * L1 * r2;
  parameter Real delta = I2 + m2 * r2 ^ 2;
  Real[2, 2] M "mass matrix";
  Real[2, 2] C "Christoffel matrix";
  Real[2] G "Gravity term";
  parameter SI.Angle theta1_0 = 90.0 * CON.D2R, theta2_0 = 0.0 * CON.D2R;
  SI.Angle[2] theta(start = {theta1_0, theta2_0});
  parameter SI.Length xe0 = L1 * cos(theta1_0) + L2 * cos(theta1_0 + theta2_0) "Initial end effector location";
  parameter SI.Length ye0 = L1 * sin(theta1_0) + L2 * sin(theta1_0 + theta2_0);
  SI.AngularVelocity[2] omega(start = {0.0, 0.0});
  SI.Torque[2] tau;
  SI.Length[2] pos1, pos2 "Positions of joints";
  // Controller parameters
  //parameter Real setpoint1 = 92.0 * CON.D2R, setpoint2 = -2.0 * CON.D2R;
  parameter Real kp1 = 20.0, ki1 = 10.0, kd1 = 0.5, loLim1 = -20.0e10, hiLim1 = 20.0e10;
  parameter Real kp2 = 20.0, ki2 = 10.0, kd2 = 0.5, loLim2 = -20.0e10, hiLim2 = 20.0e10;
  parameter Real dt = 0.001;
  parameter Integer loopID1 = 1, loopID2 = 2;
  parameter Integer id1 = PIDCinit(loopID1, dt, kp1, ki1, kd1, loLim1, hiLim1) "id is a dummy variable -- not really needed except for syntax";
  parameter Integer id2 = PIDCinit(loopID2, dt, kp2, ki2, kd2, loLim2, hiLim2);
  Real cmd1, cmd2 "Controller command variables";
  Real err1, err2 "Controller errors";

  // Supervisory (path profile) parameters and variables
  //SI.Angle theta1p0, theta2p0 "Initial angle for the path";
  SI.Angle theta1Set, theta2Set;
  parameter SI.Time tSetup = 5.0, tHold = 1.0 "Time to get to initial setpoint";
  parameter SI.Length radTraj = 0.1 "Radius of circle";
  parameter SI.Length xCenter = 0.3, yCenter = 0.1 "Center of circle to be drawn";
  parameter SI.Length x0 = xCenter + radTraj, y0 = yCenter "Initial point for circle";
  SI.Length xTraj(start = xe0), yTraj(start = ye0) "Coordinates along the controlled trajectory";
  parameter SI.Time tCircle = 3.0;
  parameter SI.AngularVelocity omegaTraj0 = 2.0 * CON.pi / tCircle "Angular velocity for circular trajectory";
  SI.AngularVelocity omegaTraj;
  SI.Time tTraj "Time along the circular trajectory";  
  
  function PIDCinit
    input Integer loopID;
    input Real dt, kp, ki, kd, loLim, hiLim;
    output Integer ip "Index for this control loop -- should be same as loopID input";
  
    external "C" ip = PIDCinit(loopID, dt, kp, ki, kd, loLim, hiLim);
    annotation(
      Include = "#include \"c:\\Users\\x\\Documents\\AAAA-Stuff\\AAAA-Working\\Modelica Projects\\TwoLinkRobot\\PID_C.c\"");
  end PIDCinit;

  //Downside of using C is that the file 'included' in the annotation section must use the full file path and is thus not portable

  function PIDCstep
    input Integer ip;
    input Real val, setpoint "process value and setpoint";
    output Real mVal "Actuation (manipulated) variable output";
  
    external "C" mVal = PIDCstep(ip, val, setpoint);
  end PIDCstep;
equation
  M * der(omega) + C * omega + G = tau;
  der(theta) = omega;
  // Mass matrix
  M[1, 1] = alpha + 2.0 * beta * cos(theta[2]);
  M[1, 2] = delta + beta * cos(theta[2]);
  M[2, 1] = M[1, 2];
  M[2, 2] = delta;
  // Christoffel matrix
  C[1, 1] = (-beta * sin(theta[2]) * omega[2]) + b1;
  C[1, 2] = -beta * sin(theta[2]) * (omega[1] + omega[2]);
  C[2, 1] = beta * sin(theta[2]) * omega[1];
  C[2, 2] = b2;
  // Gravity
  G[1] = (m1 * r1 + m2 * L1) * g * cos(theta[1]) + m2 * r2 * g * cos(theta[1] + theta[2]);
  G[2] = m2 * r2 * g * cos(theta[1] + theta[2]);
  // Torque input
  tau[1] = cmd1 "use 0 here for open loop response";
  tau[2] = cmd2;
  // Controller errors
  err1 = theta1Set - theta[1];
  err2 = theta2Set - theta[2];
  pos1[1] = L1 * cos(theta[1]) "Compute joint positions";
  pos1[2] = L1 * sin(theta[1]);
  pos2[1] = L2 * cos(theta[2] + theta[1]) + pos1[1];
  pos2[2] = L2 * sin(theta[2] + theta[1]) + pos1[2];
algorithm
  when sample(0.0, dt) then
    // First do the supervisory control (compute path setpoints)
    tTraj := time - tSetup;
  
    if time < (tSetup - tHold) then
      // Move to start of circle
      xTraj := (time / ((tSetup - tHold) + 1.0e-6)) * (x0 - xe0) + xe0;
      yTraj := (time / ((tSetup - tHold) + 1.0e-6)) * (y0 -ye0) + ye0;
      (theta1Set, theta2Set) := InvKin(xTraj, yTraj, L1, L2);
      omegaTraj := 0.0;
    elseif time < tSetup then
      // Hold position (dwell)
      (theta1Set, theta2Set) := InvKin(x0, y0, L1, L2);
      xTraj := x0;
      yTraj := y0;
      omegaTraj := 0.0;
    else
      // Draw circle
      omegaTraj := omegaTraj0;
      xTraj := radTraj * cos(omegaTraj * tTraj) + xCenter;
      yTraj := radTraj * sin(omegaTraj * tTraj) + yCenter;
      (theta1Set, theta2Set) := InvKin(xTraj, yTraj, L1, L2);
    end if;
    
    cmd1 := PIDCstep(loopID1, theta[1], theta1Set);
    cmd2 := PIDCstep(loopID2, theta[2], theta2Set);
  end when;
  annotation(
    experiment(StartTime = 0, StopTime = 7.99, Tolerance = 1e-06, Interval = 0.01598));
end TwoLinkRobotEqnCntrl;
