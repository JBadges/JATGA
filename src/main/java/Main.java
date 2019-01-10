import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.Map.Entry;

import math.Point;
import trajectory.DriveState;
import trajectory.RobotConstraints;
import trajectory.State;
import trajectory.Trajectory;
import trajectory.TrajectoryGenerator;

public class Main {

  public static final double METERS_PER_INCH = 0.0254;

  public static void main(String[] args) {
    ArrayList<Double> times = new ArrayList<>();
    double dt = 0.02;
    RobotConstraints rc = new RobotConstraints(5.1816, 3, 3, 0.55926);
    TrajectoryGenerator tg = new TrajectoryGenerator();
    double sTime = System.currentTimeMillis();
    Trajectory trajs = tg.generate(rc, false, 
      new Point(0, 0, 0),
      new Point(10, 10, Math.PI),
      new Point(20, -20, Math.PI*2),
      new Point(30, 30, 0)
    );
    // Trajectory trajs = tg.generate(rc, false, 
    //   new Point(0, 0, 0),
    //   new Point(1, 0, 0)
    // );
    // Trajectory trajs = tg.generate(rc, false, new Point(0, 0, 0), new Point(4, 4, Math.PI/2));
    System.out.printf("Took %.0fms to generate the trajectories\n", (System.currentTimeMillis() - sTime));
    System.out.printf("It will take %.2fs to follow the path\n", trajs.lastKey());
    saveTrajToFile(trajs, "ttest", dt);
    saveXYToFile(trajs, "line", dt, rc, 0);

    // trajs = tg.generate(rc, false, new Point(95.28 * METERS_PER_INCH, 0, 0), new Point(220.25 * METERS_PER_INCH, 0, 0));
    // System.out.printf("Took %.2fs to generate the trajectories\n", (System.currentTimeMillis() - sTime) / 1000.0);
    // System.out.printf("It will take %.2fs to follow the path\n", trajs.lastKey());
    // times.add(trajs.lastKey());
    // saveXYToFile(trajs, "forwardHatch", dt, rc, 0);

    // trajs = new TrajectoryGenerator().generate(rc, true, new Point(0, 0, Math.PI), new Point(-(133.13-27.4) * METERS_PER_INCH, 229.13 * METERS_PER_INCH, Math.PI));
    // System.out.printf("Took %.2fs to generate the trajectories\n", (System.currentTimeMillis() - sTime) / 1000.0);
    // System.out.printf("It will take %.2fs to follow the path\n", trajs.lastKey());
    // times.add(trajs.lastKey());
    // saveXYToFile(trajs, "hatchToHP", dt, rc, Math.PI);

    // trajs = new TrajectoryGenerator().generate(rc, false, new Point(0, 0, 0), new Point((201.13 + 40.50) * METERS_PER_INCH, -(133.13-27.4) * METERS_PER_INCH, -Math.PI/2));
    // System.out.printf("Took %.2fs to generate the trajectories\n", (System.currentTimeMillis() - sTime) / 1000.0);
    // System.out.printf("It will take %.2fs to follow the path\n", trajs.lastKey());
    // times.add(trajs.lastKey());
    // saveXYToFile(trajs, "HpToCargo", dt, rc, 0);

    // trajs = new TrajectoryGenerator().generate(rc, true, new Point(0, 0, Math.PI/2), new Point(-(201.13 + 40.50) * METERS_PER_INCH, 40 * METERS_PER_INCH, Math.PI));
    // System.out.printf("Took %.2fs to generate the trajectories\n", (System.currentTimeMillis() - sTime) / 1000.0);
    // System.out.printf("It will take %.2fs to follow the path\n", trajs.lastKey());
    // times.add(trajs.lastKey());
    // saveXYToFile(trajs, "CargoToBall", dt, rc, Math.PI/2);

    // trajs = new TrajectoryGenerator().generate(rc, false, new Point(0, 0, Math.PI/2), new Point((201.13 + 40.50 + 21.75) * METERS_PER_INCH, -(133.13-27.4) * METERS_PER_INCH, -Math.PI/2));
    // System.out.printf("Took %.2fs to generate the trajectories\n", (System.currentTimeMillis() - sTime) / 1000.0);
    // System.out.printf("It will take %.2fs to follow the path\n", trajs.lastKey());
    // times.add(trajs.lastKey());
    // saveXYToFile(trajs, "BallToCargo", dt, rc, Math.PI/2);

    // System.out.println(times);
    double totalTime = 0;
    for(double d : times) {
      totalTime += d;
    }

    // System.out.printf("It will take %.2fs to follow the entire path\n", totalTime);


    // trajs = new TrajectoryGenerator().generate(rc, false, new Point(0, 0, Math.PI/2), new Point(1, 0,-Math.PI/2));
    // System.out.printf("Took %.2fs to generate the trajectories\n", (System.currentTimeMillis() - sTime) / 1000.0);
    // saveXYToFile(trajs, "headingTest", dt, rc, Math.PI/2);

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

  public static void saveXYToFile(Trajectory traj, String name, double dt, RobotConstraints rc, double initHeading) {
    List<Point> genPath = new ArrayList<>();
    genPath.add(new Point(0, 0, initHeading));
    double heading = initHeading;
    double dT = 0.002;
    for (double t = 0; t < traj.lastEntry().getKey(); t += dT) {
      State left = traj.getInterpolated(t).getLeftDrive();
      State right = traj.getInterpolated(t).getRightDrive();
      double Dl = left.getVelocity() * dT;
      double Dr = right.getVelocity() * dT;
      double newX = genPath.get(genPath.size() - 1).getX();
      double newY = genPath.get(genPath.size() - 1).getY();
      if (Math.abs(Dl - Dr) < 1.0e-10) { // basically going straight
        double avgDist = (Dr + Dl) / 2.0;
        newX += Math.cos(heading) * avgDist;
        newY += Math.sin(heading) * avgDist;
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
      FileWriter fw = new FileWriter(new File(name + ".csv"));
      fw.write("x, y, h\n");
      for(Point p : genPath) {
        fw.write(p.getX() + ", " + p.getY() + ", " + p.getHeading() + "\n");
      }
    } catch (Exception e) {}
  }

  public static void saveTrajToFile(Trajectory traj, String name, double dt) {
     try {
      FileWriter fw = new FileWriter(new File(name + ".csv"));
      Set<Entry<Double, DriveState>> set = traj.entrySet();
      // for(double t = 0; t < traj.lastKey(); t += dt) {
      //   fw.write(traj.getInterpolated(t).getLeftDrive() + ", " + traj.getInterpolated(t).getRightDrive() + ", " + traj.getInterpolated(t).getHeading() + "\n");
      // }
      for(Entry<Double, DriveState> ds : set) {
        fw.write(ds.getValue().getLeftDrive() + ", " + ds.getValue().getRightDrive() + ", " + ds.getValue().getHeading() + "\n");
      }
      fw.close();
    } catch (Exception e) {}
  }

}
