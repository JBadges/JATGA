import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import math.Point;
import math.spline.QuinticHermiteSpline;
import trajectory.RobotConstraints;
import trajectory.TrajectoryGenerator;
import trajectory.TrajectoryPoint;

public class Main {

  public static void main(String[] args) throws IOException {
    

    RobotConstraints rc = new RobotConstraints(3, 3, 3, 0.55926);
    double sTime = System.currentTimeMillis();
    TrajectoryPoint[][] trajs = new TrajectoryGenerator().generate(rc, false, new Point(0, 0, 0), new Point(10, 0, 2), new Point(5, 20, 1), new Point(10, 23, 0));
    System.out.printf("Took %.2fs to generate the trajectories", (System.currentTimeMillis() - sTime) / 1000.0);
    BufferedWriter newPathWriter = null;
    List<Point> genPath = new ArrayList<>();
    genPath.add(new Point(0, 0));
    double heading = 0;
    for (int i = 0; i < trajs[0].length; i++) {
      TrajectoryPoint left = trajs[0][i];
      TrajectoryPoint right = trajs[1][i];
      double Dl = left.getVelocity() * left.getTimeStep();
      double Dr = right.getVelocity() * right.getTimeStep();
      double newX = genPath.get(genPath.size() - 1).getX();
      double newY = genPath.get(genPath.size() - 1).getY();
      if (Math.abs(Dl - Dr) < 1.0e-10) { // basically going straight
        double distT = (Dr + Dl) / 2.0;
        newX += Math.cos(heading) * distT;
        newY += Math.sin(heading) * distT;
      } else {
        double headingChange = (Dr - Dl) / rc.wheelbase;
        double radius = rc.wheelbase * (Dl + Dr) / (2 * (Dr - Dl));
        newX += radius * Math.sin(headingChange + heading) - radius * Math.sin(heading);
        newY -= radius * Math.cos(headingChange + heading) - radius * Math.cos(heading);
        heading += headingChange;
      }
      genPath.add(new Point(newX, newY, heading));
    }
    try {
      newPathWriter = new BufferedWriter(new FileWriter("generatedPath.csv"));
    } catch (IOException e) {
      e.printStackTrace();
    }
    try {
      newPathWriter.write("x, y, heading \n");
    } catch (IOException e1) {
      // TODO Auto-generated catch block
      e1.printStackTrace();
    }
    for (int i = 0; i < genPath.size(); i++) {
      try {
        newPathWriter
            .write(genPath.get(i).getX() + ", " + genPath.get(i).getY() + ", " + genPath.get(i).getHeading() + "\n");
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

    try {
      newPathWriter.flush();
      newPathWriter.close();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    QuinticHermiteSpline spline = new QuinticHermiteSpline(new Point(0, 0, 0), new Point(100, 100, 0));
    {
      double startTime = System.currentTimeMillis();
      double dt = 1e-5;
      double dist = 0;
      for(double t = 0; t < 1; t+=dt) {
        dist += spline.getPoint(t).distance(spline.getPoint(t+dt));
      }
      System.out.println(System.currentTimeMillis() - startTime);
      System.out.println(dist);
    }
    {
      double startTime = System.currentTimeMillis();
      double dt = 1e-5;
      double arcLength = 0;
      for(double t = 0; t < 1; t+=dt) {
        arcLength += Math.sqrt(spline.dx(t) * spline.dx(t) + spline.dy(t) * spline.dy(t)) * dt;
      }
      System.out.println(System.currentTimeMillis() - startTime);
      System.out.println(arcLength);
    }
    {
      double startTime = System.currentTimeMillis();
      double dt = 1e-5;
      double integrand = 0;
      double last_integrand = Math.sqrt(spline.dx(0) * spline.dx(0) + spline.dy(0) * spline.dy(0)) * dt;
      double arc_length = 0;
      for(double t = 0; t < 1; t+=dt) {
        integrand = Math.sqrt(spline.dx(t) * spline.dx(t) + spline.dy(t) * spline.dy(t)) * dt;
        arc_length += (integrand + last_integrand) / 2;
        last_integrand = integrand;
      }
      System.out.println(System.currentTimeMillis() - startTime);
      System.out.println(arc_length);
    }
  }

}
