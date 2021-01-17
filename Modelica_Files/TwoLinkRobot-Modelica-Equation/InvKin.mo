function InvKin
  import CON = Modelica.Constants;
  
  input Real x "Desired position of end effector";
  input Real y;
  input Real L1 "Lengths of the links";
  input Real L2;
  output Real theta1;
  output Real theta2;
protected
  Real elbow = -1.0 "Elbow up (-1)/down(+1)";
  Real D = (x^2 + y^2 - L1^2 - L2^2) / (2.0 * L1 * L2);
  Real th;
algorithm
  theta2 := elbow * atan2(sqrt(1.0 - D^2), D);
  th := atan2(y, x);
  if (th < (-CON.pi / 2.0)) then
    th := th + 2.0 * CON.pi;
  end if;
  
  theta1 := th - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)); 
end InvKin;
