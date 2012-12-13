using System;
using System.Drawing;
using OpenTK;

namespace NBody
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			var n = new NBodySystem (100);

			var numSteps = 10000;
			var stepSize = 100;
			for (int i = 0; i < numSteps; i += stepSize) {
				Console.WriteLine("Saving step {0}/{1}", i, numSteps);
				n.integrateStep(stepSize);
				n.SaveImage();
			}
		}
	}

	
	public class NBodySystem {
		public Vector3d[] position;
		public Vector3d[] velocity;
		public Vector3d[] newPosition;
		public Vector3d[] newVelocity;
		public double[] mass;
		const double G = 6.6738e-11;

		double maxSize;
		public int step;

		public NBodySystem(int n) {
			var rand = new Random ();
			maxSize = 1000.0;
			var maxVel = 0.1;
			var maxMass = 1000000.0;
			position = new Vector3d[n];
			velocity = new Vector3d[n];
			newPosition = new Vector3d[n];
			newVelocity = new Vector3d[n];
			mass = new double[n];
			step = 0;

			// Make one big mass to keep things together
			position[0] = new Vector3d(500, 500, 500);
			velocity[0] = Vector3d.Zero;
			mass[0] = 1e13;
			for (int i = 1; i < n; i++) {
				var m = rand.NextDouble() * maxMass;
				var x = rand.NextDouble() * maxSize;
				var y = rand.NextDouble() * maxSize;
				var z = rand.NextDouble() * maxSize;

				var vx = (rand.NextDouble() * maxVel) - (maxVel / 2);
				var vy = (rand.NextDouble() * maxVel) - (maxVel / 2);
				var vz = (rand.NextDouble() * maxVel) - (maxVel / 2);

				position[i] = new Vector3d(x, y, z);
				velocity[i] = new Vector3d(vx, vy, vz);
				mass[i] = m;
			}
		}

		public void integrateStep(double dt) {
			for(int i = 0; i < position.Length; i++) {
				integrateStepForObject(i, dt);
			}
			Vector3d[] tempP = position;
			Vector3d[] tempV = velocity;
			position = newPosition;
			velocity = newVelocity;
			newPosition = tempP;
			newVelocity = tempV;

			step += 1;
		}

		public void integrateStepForObject(int index, double dt) {
			var p0 = position[index];
			var v0 = velocity[index];
			var m = mass[index];
			var a = allForcesOn(index) / m;
			var p1 = p0 + (v0 * dt) + 0.5*a*(dt*dt);

			newPosition[index] = p1;
			newVelocity[index] = v0+a;
		}

		Vector3d allForcesOn(int index) {
			var m1 = mass[index];
			var accumulatedForce = Vector3d.Zero;
			for(int i = 0; i < position.Length; i++) {
				var m2 = mass[i];
				var rVec = position[index] - position[i];
				var rSquared = rVec.LengthSquared;
				if(rSquared == 0) {
					continue;
				}
				var f = G * (m1 + m2) / rSquared;
				rVec.Normalize();
				accumulatedForce += rVec * f;
			}
			return accumulatedForce;
		}

		public void SaveImage() {
			var img = new Bitmap((int)maxSize, (int) maxSize);
			for(int i = 0; i < position.Length; i++) {
				var x = (int) position[i].X;
				var y = (int) position[i].Y;
				if(x >= 0 && x < maxSize && y >= 0 && y < maxSize) {
					//Console.WriteLine ("Writing body {0} at {1},{2}", i, x, y);
					img.SetPixel(x, y, Color.White);
				}
			}
			img.Save(String.Format("nbody-{0:00000000}.png", step), System.Drawing.Imaging.ImageFormat.Png);
		}
	}
}
