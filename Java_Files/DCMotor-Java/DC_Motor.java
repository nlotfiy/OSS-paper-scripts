package dcmotor;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.hipparchus.ode.ODEIntegrator;
import org.hipparchus.ode.ODEState;
import org.hipparchus.ode.ODEStateAndDerivative;
import org.hipparchus.ode.OrdinaryDifferentialEquation;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;

    public class DC_Motor implements OrdinaryDifferentialEquation 
    {
        private double theta = 0.0;  // Angle, rad
        private double omega = 0.0;  // Angular speed, rad/s
        private double J = 0.0026;  // inertia, kg.m^2
        private double b = 0.01;  // damping constant N.m.s/rad
        // Note: back-emf constant and torque constant are actually the same quantity
        private double Kt = 0.66;  // Torque constant, N.m/A
        private double Kb = Kt;  // back-emf constant, V/(rad/s); 
                // same as Kt with consistent units
        private double R = 2.62;  // Resistance, Ohm
        private double L = 0.05;  // Inductance, H
        private double torq = 0.0; // Torque, N.m
        private double v = 0.0;  // Applied voltage, v
        private double vBack = 0.0;  // Back-emf voltage, v
        private double ic = 0.0;  // Current, A

        public DC_Motor() {
        }
        
        public void SetVoltage(double v)
        {
            this.v = v;
        }

        @Override
        public int getDimension() {
            return 3;
        }

        @Override
        public double[] computeDerivatives(double t, double[] y) 
        {
            // ODE: d angle / dt = omega; d omega/dt = (torque - b omega)/J
            // States: 0 - angle; 1 - omega; 2 - current
            omega = y[1]; // Angular velocity of shaft
            ic = y[2]; // Current in circuit
            vBack = Kb * omega;
            torq = Kt * ic;
            double [] dydt = new double[3];
            // Construct derivatives of states
            dydt[0] = omega; // d theta / dt = omegga
            dydt[1] = (torq - b * omega) / J;  // d omega / dt = (torque - damping) / inertia
            dydt[2] = (v - vBack - R * ic) / L;  // di/dt = (voltage - back-emf - R * current) / L
            return dydt; 
        }
        
        // Test case to simulate open loop motor behavior
        public static void main(String[] args) 
        {
            ODEIntegrator dp853 = new DormandPrince853Integrator(1.0e-8, 100.0, 1.0e-10, 1.0e-10);
            DC_Motor dcm = new DC_Motor();
            OrdinaryDifferentialEquation ode = dcm;

            // States: 0 - angle; 1 - omega; 2 - current
            double [] y0 = new double [] {0.0, 0.0, 0.0};
            double [] y = new double[3];
            for(int i = 0; i < 3; i++)y[i] = y0[i];
            double volt = 24.0; // Applied voltage, v
            ODEState state = new ODEState(0.0, y0);
            double t = 0.0, tf = 0.5;
            double dtLog = tf / 200.0, tNextLog = 0.0;
            double tNextSim = 0.0;
            double guard = 1.0e-10;  // Guard against floating point round-off error

            PrintWriter pw = OpenWriteFile("DCMotorOpenLoopData.txt");

            // States: 0 - angle; 1 - omega; 2 - current
            dcm.SetVoltage(volt);  // Voltage applied to motor drive cicuit

            // Print initial state
            pw.format("%g %g %g %g %g %n", t, y0[0], y0[1], y[2], volt);

            while(t <= tf)
            {
                if(t >= (tNextLog - guard))
                {
                    tNextLog += dtLog;
                    pw.format("%g %g %g %g %g %n", t, y[0], y[1], y[2], volt);
                }

                tNextSim = tNextLog; // Simulate until next event
                ODEStateAndDerivative finalState = dp853.integrate(ode, state, tNextSim);
                y = finalState.getPrimaryState();

                t = tNextSim;
                state = new ODEState(t, y); // setup for next iteration           
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
