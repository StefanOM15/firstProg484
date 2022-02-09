//============================================================================
// SimplePend.cs Defines a class for simulating a simple pendulum
//============================================================================
using System;

namespace Sim
{
    public class SimplePend
    {
        private double len = 1.1; // pendulum length
        private double g = 9.81; // gravitation field strength
        int n = 2;              // number of states
        private double[] x; // array of states
        public double[] f; // right side of equation array

        //--------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------
        public SimplePend()
        {
           x = new double[n];
           f = new double[n];

           x[0] = 1.0;
           x[1] = 0.0;
        }

        //--------------------------------------------------------------------
        // step: perform one integration step via Euler's Method`
        //--------------------------------------------------------------------
        public void step(double dt)
        {
            rhsFunc(x,f);

            for (int i=0;i<n;i++)
            {
                x[i] = x[i] + f[i] * dt;
            }
            //Console.WriteLine($"{f[0].ToString()}  {f[1].ToString()}");
        }

        //--------------------------------------------------------------------
        // rhsFunc: function to calculate rhs of pendulum ODEs
        //--------------------------------------------------------------------
        public void rhsFunc(double[] st, double[] ff)
        {
            ff[0] = st[1];
            ff[1] = -g/len * Math.Sin(st[0]);
        }

        //--------------------------------------------------------------------
        // rk4: function to calculate the next value of pendulum using fourth 
        //      order runge-kutta 
        //--------------------------------------------------------------------

        public void rk4(double dt)
        {
            double[] k = new double[4];
            double[] c = new double[2];

            
            for(int i = 0;i<n;i++)
            {
                rhsFunc(x,f);
                k[0] = f[i];

                c[1] = x[1];
                
                c[0] = x[0]+0.5*k[0]*dt;
                rhsFunc(c,f);
                k[1] = f[i];

                c[0] = x[0]+0.5*k[1]*dt;
                rhsFunc(c,f);
                k[2] = f[i];

                c[0] = x[0]+k[2]*dt;
                rhsFunc(c,f);
                k[3] = f[i];
                
                x[i] = x[i] + (dt/6)*(k[0]+2*k[1]+2*k[2]+k[3]);
            }
        }

        //--------------------------------------------------------------------
        // Getters and Setters
        //--------------------------------------------------------------------
        public double L
        {
            get {return(len);}
            
            set
            {
                if (value > 0.0)
                    len = value;
            }
        }    

        public double G
        {
            get {return(g);}
            
            set
            {
                if (value >= 0.0)
                    g = value;
            }
        }

        public double theta
        {
            get {return x[0];}
            
            set {x[0] = value;}
        } 

         public double thetaDot
        {
            get {return x[1];}
            
            set {x[1] = value;}
        }       
    }

}