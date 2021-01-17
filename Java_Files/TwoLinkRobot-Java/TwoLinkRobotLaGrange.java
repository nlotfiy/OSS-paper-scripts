package twolinkrobot;

import PID.PIDController;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.hipparchus.ode.*;
import org.hipparchus.ode.nonstiff.*;

/**
 * This file runs the robot with a controller. The main() method in the 
 * TwoLinkRobotEquations file will run the open loop system
 * @author DM Auslander, 2020-July-1
 */
public class TwoLinkRobotLaGrange 
{
    public static void main(String[] args) 
    {
        TwoLinkRobotLaGrange tlrg = new TwoLinkRobotLaGrange();
        tlrg.RunTwoLink();
    }
    
    public void RunTwoLink()
    {
        //DormandPrince853Integrator(double minStep, double maxStep, double scalAbsoluteTolerance, double scalRelativeTolerance)
        //ODEIntegrator odeInt = new DormandPrince853Integrator(1.0e-8, 100.0, 1.0e-6, 1.0e-6);
        ODEIntegrator odeInt = new DormandPrince54Integrator(1.0e-8, 100.0, 1.0e-6, 1.0e-6);
        PrintWriter pw = OpenWriteFile("TwoLinkRobotLaGrange.txt");
        TwoLinkRobotEquations tlr = new TwoLinkRobotEquations();  // Instantiate the
            // ODE equations class
        OrdinaryDifferentialEquation ode = tlr;
        double t = 0.0, tf = 11.0;  // Running time, sec
        // States: 0,1 - theta; 2,3 - omega
        double [] x0 = {90.0 * Math.PI / 180.0, 0.0 * Math.PI / 180.0, 0.0, 0.0};
        tlr.setInitialConditions(x0);
        double [] x = new double[4];

        for(int i = 0; i < 4; i++)x[i] = x0[i];
        ODEState state = new ODEState(t, x0);

        
        double dtLog = tf / 500.0, tNextLog = 0.0;
        double dtControl = 0.005, tNextControl = 0.0;
        double tNextSim;
        
        // Set up controllers
        PIDController pid1 = new PIDController(/*kp*/ 20.0, /*ki*/ 40.0, /*kd*/ 2.0,
            /*deltaT*/ dtControl, /*mMin*/ -1.0e10, /*mMax*/ 1.0e10, /*intVal*/ 0.0, /*windupProtect*/ true);
        PIDController pid2 = new PIDController(/*kp*/ 20.0, /*ki*/ 40.0, /*kd*/ 0.1,
            /*deltaT*/ dtControl, /*mMin*/ -1.0e10, /*mMax*/ 1.0e10, /*intVal*/ 0.0, /*windupProtect*/ true);
        double [] m = new double[2]; // Controller output
        double xTraj = 0.0, yTraj = 0.0; // Trajectory coordinates
        //Initial end effector location;
        double xe0 = tlr.L1 * Math.cos(x0[0]) + tlr.L2 * Math.cos(x0[0] + x0[1]); 
        double ye0 = tlr.L1 * Math.sin(x0[0]) + tlr.L2 * Math.sin(x0[0] + x0[1]);
        double angleSet1 = -91.0 * Math.PI / 180.0, angleSet2 = 0.0;
        double tSetup = 5.0, tHold = 1.0;  // Time to get to initial point on circle trajectory
        double guard = 1.0e-10; // To guard against floating point round-off error
        
        // Compute starting location for the circle trajectory
        double radTraj = 0.1;//0.1; // Radius of trajectory circle
        double tCircle = 3.0;
        double omegaCirc = 2.0 * Math.PI / tCircle;  // Angular speed on trajectory
        double xCenter =  0.3, yCenter = 0.1; //Center of circle to be drawn
        double xCirc0 = xCenter + radTraj, yCirc0 = yCenter; //Initial point for circle
        double [] angSet0 = new double[2];

        // Print initial state
        // pos1[] <x,y> of joint 1, pos2[] <x,y> of joint 2
        double [] pos1 = tlr.GetJoint1Position();
        double [] pos2 = tlr.GetJoint2Position();
        pw.format("%g %g %g %g %g %g %g %g %g\n", t, x[0], x[1], x[2], x[3],
                pos1[0], pos1[1], pos2[0], pos2[1]);                
        
        while(t <= tf)
        {
            if(t >= (tNextControl - guard))
            {
                tNextControl += dtControl;
                if(t <= (tSetup - tHold))
                {
                    // Move from initial position to beginning of circle
                    // pos1[] <x,y> of joint 1, pos2[] <x,y> of joint 2
                    xTraj = (t / ((tSetup - tHold) + 1.0e-6)) * (xCirc0 - xe0) + xe0;
                    yTraj = (t / ((tSetup - tHold) + 1.0e-6)) * (yCirc0 -ye0) + ye0;
                    angSet0 = InvKin(xTraj, yTraj, tlr.L1, tlr.L2);
                    angleSet1 = angSet0[0];
                    angleSet2 = angSet0[1];
                }
                else if(t <= tSetup)
                {
                    // Leave the setpoints as they are for "hold" period
                }
                else
                {
                    xTraj = radTraj * Math.cos(omegaCirc * (t - tSetup)) + xCenter;
                    yTraj = radTraj * Math.sin(omegaCirc * (t - tSetup)) + yCenter;
                    double [] angSet = InvKin(xTraj, yTraj, tlr.L1, tlr.L2);
                    angleSet1 = angSet[0];
                    angleSet2 = angSet[1];
                }
                // States: 0,1 - theta; 2,3 - omega
                m[0] = pid1.PIDStep(t, x[0], angleSet1);
                m[1] = pid2.PIDStep(t, x[1], angleSet2);
                tlr.SetTorque(m);
            }
            
            if(t >= (tNextLog - guard))
            {
                tNextLog += dtLog;
                tlr.ComputePositions();
                pos1 = tlr.GetJoint1Position();
                pos2 = tlr.GetJoint2Position();
                pw.format("%g %g %g %g %g %g %g %g %g %g %g %g %g\n", t, x[0], x[1], x[2], x[3],
                        pos1[0], pos1[1], pos2[0], pos2[1], angleSet1, angleSet2,
                        xTraj, yTraj);                
            }
           
            tNextSim = Math.min(tNextLog, tNextControl); // Simulate to next event
            ODEStateAndDerivative finalState = odeInt.integrate(ode, state, tNextSim);
            x = finalState.getPrimaryState();
            t = tNextSim;
            state = new ODEState(t, x); // setup for next iteration                       
        }
        pw.close();
    }

    // Inverse Kinematics
    public double [] InvKin(double x, double y, double L1, double L2)
    {
      double elbow = 1.0;  // Elbow up (-1.0) or down (+1.0)
      double D = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
      if(Math.abs(D) > 1.0)
      {
          System.out.println(" abs(D) > 1 ... exiting");
          System.exit(21);
      }
      double theta2 = elbow * Math.atan2(Math.sqrt(1.0 - D * D), D);
      double th  = Math.atan2(y, x);
      if(th < 0.0)th += 2.0 * Math.PI;

      double theta1 = th - Math.atan2(L2 * Math.sin(theta2), L1 + L2 * Math.cos(theta2));
      double [] tht = {theta1, theta2};
      return tht;
    }
    
    static private PrintWriter OpenWriteFile(String name)
    {
        PrintWriter dataFile = null;
        try
        {
            FileWriter fW = new FileWriter (name);
            dataFile = new PrintWriter ( fW );
        }
        catch(IOException e)
        {
            System.out.println("<OpenWriteFile> IO Error " + e);
            System.exit(1);  // File error -- quit
        }
        return dataFile;
    }        
}
