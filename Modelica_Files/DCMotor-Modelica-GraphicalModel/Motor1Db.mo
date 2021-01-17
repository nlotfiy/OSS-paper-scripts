model Motor1Db
  import SI = Modelica.SIunits;
  // Motor parameters
  parameter SI.Resistance R = 2.62 " Ohm";
  parameter SI.Voltage v = 24.0 " v";
  parameter SI.ElectricalTorqueConstant Kt = 0.66 "N.m/A";
  parameter SI.MomentOfInertia J = 0.0026 "kg.m^2";
  parameter SI.RotationalDampingConstant b = 0.01 "N.m.s/rad";
  parameter SI.Inductance L = 0.05 " H";
  Modelica.Electrical.Analog.Basic.EMF emf(k = Kt) annotation(Placement(visible = true, transformation(origin = {0, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R)  annotation(Placement(visible = true, transformation(origin = {-28, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = J, phi(fixed = true, start = 0), w(fixed = true, start = 0))  annotation(Placement(visible = true, transformation(origin = {34, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L, i(fixed = true))  annotation(Placement(visible = true, transformation(origin = {-28, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(Placement(visible = true, transformation(origin = {-72, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage1(V = v)  annotation(Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = b)  annotation(
    Placement(visible = true, transformation(origin = {58, 12}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Fixed RotationalFixed annotation(
    Placement(visible = true, transformation(origin = {74, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ground1.p, constantVoltage1.n) annotation(Line(points = {{-72, -20}, {-50, -20}, {-50, -10}, {-50, -10}, {-50, -10}}, color = {0, 0, 255}));
  connect(constantVoltage1.n, inductor1.p) annotation(Line(points = {{-50, -10}, {-50, -10}, {-50, -20}, {-38, -20}, {-38, -20}}, color = {0, 0, 255}));
  connect(constantVoltage1.p, resistor1.p) annotation(Line(points = {{-50, 10}, {-50, 10}, {-50, 20}, {-38, 20}, {-38, 20}}, color = {0, 0, 255}));
  connect(inductor1.n, emf.n) annotation(Line(points = {{-18, -20}, {0, -20}, {0, -12}, {0, -12}, {0, -12}}, color = {0, 0, 255}));
  connect(emf.flange, inertia1.flange_a) annotation(
    Line(points = {{10, -2}, {24, -2}}));
  connect(resistor1.n, emf.p) annotation(Line(points = {{-18, 20}, {0, 20}, {0, 8}}, color = {0, 0, 255}));
  connect(damper.flange_a, inertia1.flange_b) annotation(
    Line(points = {{48, 12}, {44, 12}, {44, -2}}));
  connect(damper.flange_b, RotationalFixed.flange) annotation(
    Line(points = {{68, 12}, {74, 12}, {74, 0}}));
  annotation(uses(Modelica(version = "3.2.3")), experiment(StartTime = 0, StopTime = 0.5, Tolerance = 1e-06, Interval = 0.001));
end Motor1Db;
