model TwoLinkRevoluteCtrl
  import SI = Modelica.SIunits;
  import CON = Modelica.Constants;
  
  inner Modelica.Mechanics.MultiBody.World world(axisDiameter = diam1 / 5, axisLength = len1)  annotation(
    Placement(visible = true, transformation(origin = {-90, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed1 annotation(
    Placement(visible = true, transformation(origin = {-86, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute Revolute1(cylinderDiameter = diam1 * 2, cylinderLength = diam1 * 2,n = {0, 0, 1}, phi(start = theta1_0), useAxisFlange = true) annotation(
    Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Link1(density(displayUnit = "kg/m3"), diameter = diam1, r = {len1, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-16, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute Revolute2(cylinderDiameter = diam1 * 2, cylinderLength = diam1 * 2, phi(start = theta2_0), useAxisFlange = true)  annotation(
    Placement(visible = true, transformation(origin = {18, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Link2(diameter = diam2, r = {len2, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  parameter SI.Angle theta1_0 = 90.0 * CON.D2R, theta2_0 = 0.0 * CON.D2R;
  parameter SI.Length xe0 = len1 * cos(theta1_0) + len2 * cos(theta1_0 + theta2_0) "Initial end effector location";
  parameter SI.Length ye0 = len1 * sin(theta1_0) + len2 * sin(theta1_0 + theta2_0);
  constant Real pi = CON.pi;
  parameter SI.Mass m1 = 0.5, m2 = m1;
  parameter SI.Length len1 = 0.25, len2 = len1 "Length of pendulum rod";
  parameter SI.Density densSteel = 7700.0 "Density of steel, kg/m^3";
  parameter SI.Length diam1 = 2.0 * sqrt(m1/(len1 * pi * densSteel)) "Compute diameter assuming cylindrical rod";
  parameter SI.Length diam2 = 2.0 * sqrt(m2/(len2 * pi * densSteel));
  parameter SI.RotationalDampingConstant b1 = 0.1, b2 = b1 "N.m.s/rad";
  
  PID pid1(kp = 30.0, ki = 10.0, kd = 0.5, loLim = -10.0, hiLim = 10.0, dt = 0.005) "Controllers -- PID on angle";
  PID pid2(kp = 30.0, ki = 10.0, kd = 0.5, loLim = -10.0, hiLim = 10.0, dt = 0.005);

// Supervisory (path profile) parameters and variables
  SI.Angle theta1Set, theta2Set;
  parameter SI.Time tSetup = 5.0, tHold = 1.0 "Time to get to initial setpoint";
  parameter SI.Length radTraj = 0.1 "Radius of circle";
  parameter SI.Length xCenter = 0.2, yCenter = 0.2 "Center of circle to be drawn";
  parameter SI.Length x0 = xCenter + radTraj, y0 = yCenter "Initial point for circle";
  SI.Length xTraj, yTraj "Coordinates along the controlled trajectory";
  parameter SI.Time tCircle = 4.0;
  parameter SI.AngularVelocity omegaTraj0 = 2.0 * CON.pi / tCircle "Angular velocity for circular trajectory";
  SI.AngularVelocity omegaTraj;
  SI.Time tTraj "Time along the circular trajectory";
  
  Modelica.Mechanics.Rotational.Components.Damper damper1(d = b1)  annotation(
    Placement(visible = true, transformation(origin = {-48, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper2(d = b2)  annotation(
    Placement(visible = true, transformation(origin = {22, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque2 torque1 annotation(
    Placement(visible = true, transformation(origin = {-52, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque2 torque2 annotation(
    Placement(visible = true, transformation(origin = {20, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

equation
  tTraj = time - tSetup;
  
  if time < (tSetup - tHold) then
    xTraj = (time / ((tSetup - tHold) + 1.0e-6)) * (x0 - xe0) + xe0;
    yTraj = (time / ((tSetup - tHold) + 1.0e-6)) * (y0 -ye0) + ye0;
    (theta1Set, theta2Set) = InvKin(xTraj, yTraj, len1, len2);
    omegaTraj = 0.0;
  elseif time < tSetup then
    (theta1Set, theta2Set) = InvKin(x0, y0, len1, len2);
    xTraj = x0;
    yTraj = y0;
    omegaTraj = 0.0;
  
  else
    omegaTraj = omegaTraj0;
    xTraj = radTraj * cos(omegaTraj * tTraj) + xCenter;
    yTraj = radTraj * sin(omegaTraj * tTraj) + yCenter;
    (theta1Set, theta2Set) = InvKin(xTraj, yTraj, len1, len2);
  end if;
    
  pid1.sp = theta1Set;
  pid1.y = Revolute1.angle;
  torque1.tau = pid1.m;
  pid2.sp = theta2Set;
  pid2.y = Revolute2.angle;
  torque2.tau = pid2.m;

  connect(fixed1.frame_b, Revolute1.frame_a) annotation(
    Line(points = {{-76, 0}, {-62, 0}, {-62, 0}, {-62, 0}}, color = {95, 95, 95}));
  connect(Revolute1.frame_b, Link1.frame_a) annotation(
    Line(points = {{-42, 0}, {-26, 0}, {-26, 0}, {-26, 0}}));
  connect(Link1.frame_b, Revolute2.frame_a) annotation(
    Line(points = {{-6, 0}, {8, 0}, {8, 0}, {8, 0}}));
  connect(Revolute2.frame_b, Link2.frame_a) annotation(
    Line(points = {{28, 0}, {42, 0}, {42, 0}, {42, 0}}, color = {95, 95, 95}));
  connect(damper1.flange_a, Revolute1.support) annotation(
    Line(points = {{-58, 24}, {-62, 24}, {-62, 10}, {-58, 10}, {-58, 10}}));
  connect(damper1.flange_b, Revolute1.axis) annotation(
    Line(points = {{-38, 24}, {-38, 24}, {-38, 10}, {-52, 10}, {-52, 10}}));
  connect(damper2.flange_a, Revolute2.support) annotation(
    Line(points = {{12, 24}, {6, 24}, {6, 10}, {12, 10}, {12, 10}}));
  connect(damper2.flange_b, Revolute2.axis) annotation(
    Line(points = {{32, 24}, {32, 24}, {32, 10}, {18, 10}, {18, 10}}));
  connect(torque1.flange_a, Revolute1.support) annotation(
    Line(points = {{-62, 48}, {-66, 48}, {-66, 10}, {-58, 10}, {-58, 10}}));
  connect(torque1.flange_b, Revolute1.axis) annotation(
    Line(points = {{-42, 48}, {-34, 48}, {-34, 10}, {-52, 10}, {-52, 10}}));
  connect(torque2.flange_a, Revolute2.support) annotation(
    Line(points = {{10, 48}, {0, 48}, {0, 10}, {12, 10}, {12, 10}}));
  connect(torque2.flange_b, Revolute2.axis) annotation(
    Line(points = {{30, 48}, {38, 48}, {38, 10}, {18, 10}, {18, 10}}));
  annotation(
    uses(Modelica(version = "3.2.3")),
    experiment(StartTime = 0, StopTime = 13, Tolerance = 1e-06, Interval = 0.026));
end TwoLinkRevoluteCtrl;
