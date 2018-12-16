package trajectory;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;
import java.util.Map.Entry;

import math.Point;
import math.spline.Path;
import math.spline.QuinticHermiteSpline;

public class TrajectoryGenerator {

  private int pointsPerPath = 100_000;
  private double dI = 1e-6;

  public Trajectory generate(RobotConstraints rc, boolean reversed, Point... points) {
    List<QuinticHermiteSpline> splines = new ArrayList<>();
    for (int i = 1; i < points.length; i++) {
      splines.add(new QuinticHermiteSpline(points[i - 1], points[i]));
    }
    return generate(new Path(splines), reversed, rc);
  }

  public Trajectory generate(Path path, boolean reversed, RobotConstraints rc) {
    ArrayList<ArrayList<Double>> avgVelocities = maxAcheivableVelocityForPath(path, rc);
    return generateTrajectory(path, reversed, avgVelocities);
  }

  private Trajectory generateTrajectory(Path path, boolean reversed, ArrayList<ArrayList<Double>> velocites) {
    sumT.clear();
    Trajectory traj = new Trajectory();
    traj.put(0.0, new DriveState(new State(0, 0, 0), new State(0, 0, 0), 0));

    double dD = path.getTotalDistance() / pointsPerPath;

    for (int i = 1; i < velocites.get(0).size(); i++) {
      double avgVel = (velocites.get(0).get(i) + velocites.get(1).get(i)) / 2;
      double prevAvgVel = (velocites.get(0).get(i - 1) + velocites.get(1).get(i - 1)) / 2;
      double dT = dD / ((avgVel + prevAvgVel) / 2);
      double leftAcceleration = (velocites.get(0).get(i) - velocites.get(0).get(i - 1)) / dT;
      double rightAcceleration = (velocites.get(1).get(i) - velocites.get(1).get(i - 1)) / dT;
      double time = traj.lastEntry().getValue().getLeftDrive().getTime() + dT;
      traj.put(time, new DriveState(new State(time, velocites.get(0).get(i), leftAcceleration),
          new State(time, velocites.get(1).get(i), rightAcceleration), velocites.get(2).get(i)));
    }

    for (Entry<Double, DriveState> dds : traj.entrySet()) {
      DriveState ds = dds.getValue();
      ds.setHeading(Point.normalize(ds.getHeading(), Math.PI));
      ds.setHeading(Point.normalize(ds.getHeading(), Math.PI));
    }
    boolean changed;
    List<Double> keyList = new ArrayList<>(traj.keySet());
    do {
      changed = false;
      for (int i = 0; i < keyList.size()-1; i++) {
        DriveState nextDs = traj.get(keyList.get(i+1));
        DriveState ds = traj.get(keyList.get(i));
        if (Math.abs(nextDs.getHeading() - ds.getHeading()) > Math.PI / 2.0) {
          double sign = nextDs.getHeading() - ds.getHeading() > 0 ? 1 : -1;
          nextDs.setHeading(nextDs.getHeading() - (Math.PI * sign));
          changed = true;
        }
      }
    } while (changed);

    if (reversed) {
      for (Entry<Double, DriveState> dds : traj.entrySet()) {
        DriveState ds = dds.getValue();
        ds.getLeftDrive().setVelocity(-ds.getLeftDrive().getVelocity());
        ds.getLeftDrive().setAcceleration(-ds.getLeftDrive().getAcceleration());
        ds.getRightDrive().setVelocity(-ds.getRightDrive().getVelocity());
        ds.getRightDrive().setAcceleration(-ds.getRightDrive().getAcceleration());
        State temp = ds.getLeftDrive();
        ds.setLeftDrive(ds.getRightDrive());
        ds.setRightDrive(temp);
      }
    }

    String filename = "test";

    if (!(filename == null || filename == "")) {
      FileWriter fw = null;
      try {
        fw = new FileWriter(new File(filename + ".csv"));
        fw.flush();
        fw.write("Heading,\n");
        for (Entry<Double, DriveState> dds : traj.entrySet()) {
          fw.write(dds.getValue().getHeading()+",\n");
        }
        fw.close();
      } catch (Exception e) {

      }
    }

    return traj;
  }

  private ArrayList<ArrayList<Double>> maxAcheivableVelocityForPath(Path path, RobotConstraints rc) {
    // vi=sqrt(vo+2ad)
    ArrayList<Double> avgVelocities = new ArrayList<>();
    ArrayList<ArrayList<Double>> velocities = new ArrayList<ArrayList<Double>>(2);
    velocities.add(new ArrayList<Double>());
    velocities.add(new ArrayList<Double>());
    velocities.add(new ArrayList<Double>());
    ArrayList<Double> ts = new ArrayList<>();

    double dD = path.getTotalDistance() / pointsPerPath;

    avgVelocities.add(0.0);
    ts.add(0.0);
    for (int i = 1; i < pointsPerPath; i++) {
      double t = sumToT(path, dD * i);
      ts.add(t);
      double lastVelocity = avgVelocities.get(i - 1);
      double accelerateableVelocity = Math.sqrt(lastVelocity + 2 * rc.maxAcceleration * dD);
      double maxAcheiveableVelocityForCurvature = maximumAverageVelocityAtPoint(rc, path, t);
      avgVelocities.add(Math.min(accelerateableVelocity, maxAcheiveableVelocityForCurvature));
    }

    avgVelocities.set(avgVelocities.size() - 1, 0.0);
    for (int i = avgVelocities.size() - 2; i >= 0; i--) {
      double t = sumToT(path, dD * i);
      double lastVelocity = avgVelocities.get(i + 1);
      double accelerateableVelocity = Math.sqrt(lastVelocity + 2 * rc.maxAcceleration * dD);
      double maxAcheiveableVelocityForCurvature = maximumAverageVelocityAtPoint(rc, path, t);
      avgVelocities.set(i, Math.min(accelerateableVelocity, maxAcheiveableVelocityForCurvature));
    }

    for (int i = 0; i < ts.size(); i++) {
      double curvature = path.get((int) (ts.get(i).doubleValue())).getCurvature(ts.get(i) % 1);
      double radius = 1 / curvature;

      if (Double.isFinite(radius)) {
        velocities.get(0)
            .add((avgVelocities.get(i) * 2) / ((radius + rc.wheelbase / 2) / (radius - rc.wheelbase / 2) + 1));
        velocities.get(1).add(avgVelocities.get(i) * 2 - velocities.get(0).get(velocities.get(0).size() - 1));
      } else {
        velocities.get(0).add(avgVelocities.get(i));
        velocities.get(1).add(avgVelocities.get(i));
      }
      velocities.get(2).add(path.get((int) (ts.get(i).doubleValue())).getHeading(ts.get(i) % 1));
    }

    return velocities;
  }

  private double maximumAverageVelocityAtPoint(RobotConstraints constraints, Path path, double t) {
    // Constraints
    // left and right velocites <= Vmax | (Vleft + Vright) / 2 = Vmax
    // Vleft / Vright = (r+w)/(r-w) | Vleft = (r+w)/(r-w) * Vright
    // (Vright - Vleft) / w = Vangular

    double radius = 1 / path.get((int) t).getCurvature(t % 1);
    if (!Double.isFinite(radius)) {
      return constraints.maxVelocity;
    }
    return (Math.abs(radius) * constraints.maxVelocity) / (Math.abs(radius) + constraints.wheelbase / 2);
  }

  public void setDI(double i) {
    dI = i;
  }

  public void setPointsPerPath(int ppp) {
    pointsPerPath = ppp;
  }

  static TreeMap<Double, Double> sumT = new TreeMap<>();

  private double sumToT(Path splines, double distance) {
    double i = 0;
    double dist = 0;

    if (!sumT.isEmpty()) {
      dist = sumT.lastEntry().getKey();
      i = sumT.lastEntry().getValue();
    }

    while (dist < distance && i + dI < splines.size()) {
      dist += Math.sqrt(splines.get((int) i).dx(i % 1) * splines.get((int) i).dx(i % 1)
          + splines.get((int) i).dy(i % 1) * splines.get((int) i).dy(i % 1)) * dI;
      i += dI;
    }
    sumT.put(dist, i);

    return i;
  }

}