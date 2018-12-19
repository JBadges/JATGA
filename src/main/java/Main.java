import java.io.File;
import java.io.FileWriter;
import math.Point;
import trajectory.RobotConstraints;
import trajectory.Trajectory;
import trajectory.TrajectoryGenerator;

public class Main {

  public static void main(String[] args) {
    RobotConstraints rc = new RobotConstraints(3, 3, 3, 0.55926);
    double sTime = System.currentTimeMillis();
    Trajectory trajs = new TrajectoryGenerator().generate(rc, false, new Point(0, 0, 0), new Point(10, 0, 0));
    System.out.printf("Took %.2fs to generate the trajectories\n", (System.currentTimeMillis() - sTime) / 1000.0);

    // try {
    //   FileWriter fw = new FileWriter(new File("test.csv"));
    //   for(double t = 0; t < trajs.lastKey(); t += 0.02) {
    //     fw.write(trajs.getInterpolated(t).toString());
    //   }
    // } catch (Exception e) {}
    // QuinticHermiteSpline spline = new QuinticHermiteSpline(new Point(0, 0, 0), new Point(100, 100, 0));
    // {
    //   double startTime = System.currentTimeMillis();
    //   double dt = 1e-5;
    //   double dist = 0;
    //   for(double t = 0; t < 1; t+=dt) {
    //     dist += spline.getPoint(t).distance(spline.getPoint(t+dt));
    //   }
    //   System.out.println(System.currentTimeMillis() - startTime);
    //   System.out.println(dist);
    // }
    // {
    //   double startTime = System.currentTimeMillis();
    //   double dt = 1e-5;
    //   double arcLength = 0;
    //   for(double t = 0; t < 1; t+=dt) {
    //     arcLength += Math.sqrt(spline.dx(t) * spline.dx(t) + spline.dy(t) * spline.dy(t)) * dt;
    //   }
    //   System.out.println(System.currentTimeMillis() - startTime);
    //   System.out.println(arcLength);
    // }
    // {
    //   double startTime = System.currentTimeMillis();
    //   double dt = 1e-5;
    //   double integrand = 0;
    //   double last_integrand = Math.sqrt(spline.dx(0) * spline.dx(0) + spline.dy(0) * spline.dy(0)) * dt;
    //   double arc_length = 0;
    //   for(double t = 0; t < 1; t+=dt) {
    //     integrand = Math.sqrt(spline.dx(t) * spline.dx(t) + spline.dy(t) * spline.dy(t)) * dt;
    //     arc_length += (integrand + last_integrand) / 2;
    //     last_integrand = integrand;
    //   }
    //   System.out.println(System.currentTimeMillis() - startTime);
    //   System.out.println(arc_length);
    // }
  }

}
