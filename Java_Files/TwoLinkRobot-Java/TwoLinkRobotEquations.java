package twolinkrobot;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.hipparchus.ode.OrdinaryDifferentialEquation;
//import java.lang.Math.*;
import org.hipparchus.ode.ODEIntegrator;
import org.hipparchus.ode.ODEState;
import org.hipparchus.ode.ODEStateAndDerivative;
import org.hipparchus.ode.nonstiff.DormandPrince54Integrator;

/**
 *
 * @author x
 */
public class TwoLinkRobotEquations implements OrdinaryDifferentialEquation 
{
    double L1 = 0.25, L2 = L1; // Link lengths, m;
    double r1 = L1 / 2.0, r2 = L2 / 2.0; // Radius-arm
    double m1 = 0.5, m2 = m1; //kg
    double g = 9.81; //m/s^2
    double b1 = 0.1, b2 = b1; // damping coefficients, Nm-s/rad

    // Derived parameters
    double I1 = m1 * L1 * L1 / 12.0, I2 = m2 * L2 * L2 / 12.0;
    double alpha = I1 + I2 + m1 * r1 * r1 + m2 * (L1 * L2 + r2 * r2);
    double beta = m2 * L1 * r2;
    double delta = I2 + m2 * r2 * r2;
    
    double [][] M = new double[2][2]; // Mass matrix
    double [][] C = new double[2][2]; // Christoffel matrix
    double [] G = new double[2];  // Gravity term
    
    double [] theta0 = {90.0 * Math.PI / 180.0, -90.0 * Math.PI / 180.0}; // Joint angles, rad
    double [] theta = new double[2];
    double [] omega0 = {0.0, 0.0}; // Joint angular velocities, rad/s
    double [] omega = new double[2];
    double [] tau = new double[2];  // Joint torques

    double [] pos1 = new double[2]; // Joint positions (for display purposes,
    double [] pos2 = new double[2]; // not used in calculation)
    
    double [] x = new double[4];  // State and intial state
    double [] x0 = new double[4];
    double [] dxdt = new double[4];
    
    double [] cmd = {0.0, 0.0}; // Torque command
    
    public TwoLinkRobotEquations()
    {
        // States: 0,1 - theta; 2,3 - omega
        x0[0] = theta0[0]; // Set initial state
        x0[1] = theta0[1];
        x0[2] = omega0[0];
        x0[3] = omega0[1];
        for(int i = 0; i <2; i++)theta[i] = theta0[i];
        for(int i = 0; i <2; i++)omega[i] = omega0[i];
        ComputePositions();
    }
    
    public void setInitialConditions(double [] ic)
    {
        for(int i = 0; i < 4; i++)x0[i] = ic[i];
        for(int i = 0; i < 2; i++)theta0[i] = ic[i];
        for(int i = 0; i < 2; i++)omega0[i] = ic[i + 2];
        for(int i = 0; i <2; i++)theta[i] = theta0[i];
        for(int i = 0; i <2; i++)omega[i] = omega0[i];
        ComputePositions();
    }
    
    public void ComputePositions()
    {
        pos1[0] = L1 * Math.cos(theta[0]);
        pos1[1] = L1 * Math.sin(theta[0]);
        pos2[0] = L2 * Math.cos(theta[1] + theta[0]) + pos1[0];
        pos2[1] = L2 * Math.sin(theta[1] + theta[0]) + pos1[1];
    }
    
    public double [] GetJoint1Position()
    {
        return pos1;
    }
    
    public double [] GetJoint2Position()
    {
        return pos2;
    }
    
    public double [] GetInitialState()
    {
        return x0;
    }
    
    public void SetTorque(double [] torq)
    {
        for(int i = 0; i < 2; i++)cmd[i] = torq[i];
    }
    
    @Override
    public int getDimension() 
    {
        return 4;  // 2 angles, 2 angular velocities
    }

    @Override
    public void init(double t0, double[] y0, double finalTime) 
    {
        OrdinaryDifferentialEquation.super.init(t0, y0, finalTime); //To change body of generated methods, choose Tools | Templates.
        // This is optional -- nothing to do for this system
    }

    @Override
    public double[] computeDerivatives(double t, double[] x) 
    {
        theta[0] = x[0]; // Copy state variables to local variables
        theta[1] = x[1]; 
        ComputePositions();
        omega[0] = x[2];
        omega[1] = x[3];
        
        dxdt[0] = omega[0];  // d theta/dt = omega
        dxdt[1] = omega[1];
        
        // Be careful of the comparison of equations here -- Java uses 0-based indexing
        // while Modelica uses 1-based indexing!!
        // Mass matrix
        M[0][0] = alpha + 2.0 * beta * Math.cos(theta[1]); // m(1,1), etc.!
        M[0][1] = delta + beta * Math.cos(theta[1]);
        M[1][0] = M[0][1];
        M[1][1] = delta;
        
        // Christoffel matrix
        C[0][0] = -beta * Math.sin(theta[1]) * omega[1] + b1;
        C[0][1] = -beta * Math.sin(theta[1]) * (omega[0] + omega[1]);
        C[1][0] = beta * Math.sin(theta[1]) * omega[0];
        C[1][1] = b2;
        
        // Gravity
        G[0] = (m1 * r1 + m2 * L1) * g * Math.cos(theta[0]) + m2 * r2 * g * Math.cos(theta[0] + theta[1]);
        G[1] = m2 * r2 * g * Math.cos(theta[0] + theta[1]);

        // Torques
        tau[0] = cmd[0];
        tau[1] = cmd[1];
        
        double detM = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        dxdt[2] = (M[0][1] * (G[1] - tau[1] + C[1][0] * omega[0] + C[1][1] * omega[1]) -
                M[1][1] * (G[0] - tau[0] + C[0][0] * omega[0] + C[0][1] * omega[1])) / detM;
        dxdt[3] = (M[1][0] * (G[0] - tau[0] + C[0][0] * omega[0] + C[0][1] * omega[1]) -
                M[0][0] * (G[1] - tau[1] + C[1][0] * omega[0] + C[1][1] * omega[1])) / detM;
                
        return dxdt;
    }
    
    // This main() method will produce an open-loop simulation -- use the main()
    // in TwoLinkRobotLaGrange for a closed loop simulation
    public static void main(String[] args) 
    {
        //DormandPrince853Integrator(double minStep, double maxStep, double scalAbsoluteTolerance, double scalRelativeTolerance)
        //ODEIntegrator odeInt = new DormandPrince853Integrator(1.0e-8, 100.0, 1.0e-6, 1.0e-6);
        ODEIntegrator odeInt = new DormandPrince54Integrator(1.0e-8, 100.0, 1.0e-6, 1.0e-6);
        PrintWriter pw = OpenWriteFile("TwoLinkRobotLaGrange.txt");
        TwoLinkRobotEquations tlr = new TwoLinkRobotEquations();  // Instantiate the
            // ODE equations class
        OrdinaryDifferentialEquation ode = tlr;
        double t = 0.0, tf = 10.0;  // Running time, sec
        // States: 0,1 - theta; 2,3 - omega
        double [] x0 = {0.0 * Math.PI / 180.0, 0.0 * Math.PI / 180.0, 0.0, 0.0};
        tlr.setInitialConditions(x0);

        double [] x = new double[4];

        for(int i = 0; i < 4; i++)x[i] = x0[i];
        ODEState state = new ODEState(t, x0);

        // Print initial state
        double [] pos1 = tlr.GetJoint1Position();
        double [] pos2 = tlr.GetJoint2Position();
        pw.format("%g %g %g %g %g %g %g %g %g\n", t, x[0], x[1], x[2], x[3],
                pos1[0], pos1[1], pos2[0], pos2[1]);                
        
        double dtLog = tf / 500.0, tNextLog = 0.0;
        double tNextSim;
        
        while(t <= tf)
        {
            if(t >= tNextLog)
            {
                tNextLog += dtLog;
                pos1 = tlr.GetJoint1Position();
                pos2 = tlr.GetJoint2Position();
                pw.format("%g %g %g %g %g %g %g %g %g\n", t, x[0], x[1], x[2], x[3],
                        pos1[0], pos1[1], pos2[0], pos2[1]);                
            }
            
            tNextSim = tNextLog;
            ODEStateAndDerivative finalState = odeInt.integrate(ode, state, tNextSim);
            x = finalState.getPrimaryState();
            t = tNextSim;
            state = new ODEState(t, x); // setup for next iteration                       
        }
        pw.close();
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
