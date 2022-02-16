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
            double[][] k = new double[4][];
            k[0] = new double[n];
            k[1] = new double[n];
            k[2] = new double[n];
            k[3] = new double[n];
            double[] xi = new double[n];


            rhsFunc(x,k[0]);
            for(int i = 0;i<n;i++)
            {
                xi[i] = x[i]+0.5*k[0][i]*dt;
            }

            rhsFunc(xi,k[1]);
            for(int i = 0;i<n;i++)
            {
                xi[i] = x[i]+0.5*k[1][i]*dt;
            }

            rhsFunc(xi,k[2]);
            for(int i = 0;i<n;i++)
            {
                xi[i] = x[i]+k[2][i]*dt;
            }

            rhsFunc(xi,k[3]);
            for(int i = 0;i<n;i++)
            {
                x[i] = x[i] + (dt/6.0)*(k[0][i]+2.0*k[1][i]+2.0*k[2][i]+k[3][i]);
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