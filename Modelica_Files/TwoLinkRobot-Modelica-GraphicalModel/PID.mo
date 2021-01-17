class PID
  import CON = Modelica.Constants;

  parameter Real kp = 1.0, ki = 0.0, kd = 0.0;
  parameter Real dt = 0.02 "Sample time";
  parameter Real loLim = -CON.inf, hiLim = CON.inf;
  Real y "target variable";
  Real sp "setpoint";
  Real err(fixed = true) "error";
  Real preErr(start = 0.0, fixed = true);
  Real integ(start = 0.0, fixed = true) "Integrator stored value";
  Real deriv(fixed = true) "Derivative term";
  Real m "output (manipulated) variable";
algorithm
  when sample(0.0, dt) then
    preErr := err;
    err := sp - y;
    integ := integ + ki * err * dt;
    deriv := kd * (err - preErr) / dt;
    m := kp * err + integ + deriv;
    m := max(m, loLim) "Output saturation limits";
    m := min(m, hiLim);
  end when;
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end PID;
