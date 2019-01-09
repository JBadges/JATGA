package trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;

import math.Point;
import math.spline.Path;
import math.spline.QuinticHermiteSpline;

public class TrajectoryGenerator {

  private int pointsPerPath = 7_500;
  private double dI = 1e-4;
  private Trajectory traj;
  private double dD;

  public Trajectory generate(RobotConstraints rc, boolean reversed, Point... points) {
    //Set ponts to start at 0,0,0
    if(points[0].getX() != 0 || points[0].getY() != 0 || points[0].getHeading() != 0) {
        Point start = points[0];
        Point[] newPoints = new Point[points.length];
        for(int i = 0; i < points.length; i++) {
          newPoints[i] = new Point(points[i].getX() - start.getX(), points[i].getY() - start.getY(), points[i].getHeading());
        }
      //Rotate points from init heading
      for(int i = 0; i < points.length; i++) {
        double x = points[i].getX() * Math.cos(start.getHeading()) - points[i].getY() * Math.sin(start.getHeading());
        double y = points[i].getY() * Math.cos(start.getHeading()) + points[i].getX() * Math.sin(start.getHeading());
        double heading = points[i].getHeading() - start.getHeading();
        newPoints[i].setX(x);
        newPoints[i].setY(y);
        newPoints[i].setHeading(heading);
      }
    }

    List<QuinticHermiteSpline> splines = new ArrayList<>();
    for (int i = 1; i < points.length; i++) {
      splines.add(new QuinticHermiteSpline(points[i - 1], points[i]));
    }
    return generate(new Path(splines), reversed, rc);
  }

  public Trajectory generate(Path path, boolean reversed, RobotConstraints rc) {
    double time = System.currentTimeMillis();
    ArrayList<ArrayList<Double>> avgVelocities = maxAcheivableVelocityForPath(path, rc);
    System.out.println("Time for maxAcheivableVelocityForPath : " + (System.currentTimeMillis() - time));
    time = System.currentTimeMillis();
    traj = generateTrajectory(path, reversed, rc, avgVelocities);
    System.out.println("Time for generateTrajectory : " + (System.currentTimeMillis() - time));
    return traj;
  }

  private Trajectory generateTrajectory(Path path, boolean reversed, RobotConstraints rc, ArrayList<ArrayList<Double>> velocites) {
    Trajectory traj = new Trajectory();
    traj.put(0.0, new DriveState(new State(0, 0, 0, rc.maxAcceleration), new State(0, 0, 0, rc.maxAcceleration), 0));

    double leftDist = 0;
    double rightDist = 0;
    double leftCur;
    double leftNext;
    double rightCur;
    double rightNext;

    double sTime = System.currentTimeMillis();
    for (int i = 0; i < velocites.get(0).size()-1; i++) {
      leftCur = velocites.get(0).get(i);
      leftNext = velocites.get(0).get(i+1);
      rightCur = velocites.get(1).get(i);
      rightNext = velocites.get(1).get(i+1);
      double prevAvgVel = (leftCur + rightCur) / 2;
      double dT = dD / ((((leftNext + rightNext) / 2) + prevAvgVel)/2);
      double leftChangeInDist = leftCur * dT;
      leftDist += leftChangeInDist;
      double rightChangeInDist = rightCur * dT;
      rightDist += rightChangeInDist;
      double leftAcceleration = (leftNext - leftCur) / dT;
      double rightAcceleration = (rightNext - rightCur) / dT;
      double time = traj.lastEntry().getValue().getLeftDrive().getTime() + dT;
      traj.put(time, new DriveState(new State(time, leftDist, leftCur, leftAcceleration),
          new State(time, rightDist, rightCur, rightAcceleration), velocites.get(2).get(i)));
    }
    System.out.println("Time for traj : " + (System.currentTimeMillis() - sTime));
    // for (Entry<Double, DriveState> dds : traj.entrySet()) {
    //   DriveState ds = dds.getValue();
    //   ds.setHeading(Point.normalize(ds.getHeading(), Math.PI));
    //   ds.setHeading(Point.normalize(ds.getHeading(), Math.PI));
    // }
    
    sTime = System.currentTimeMillis();
    boolean changed;
    List<Double> keyList = new ArrayList<>(traj.keySet());
    do {
      changed = false;
      for (int i = 0; i < keyList.size()-1; i++) {
        DriveState nextDs = traj.get(keyList.get(i+1));
        DriveState ds = traj.get(keyList.get(i));
        double nextHeading = nextDs.getHeading();
        double curHeading = ds.getHeading();
        if (Math.abs(nextHeading - curHeading) > Math.PI / 2.0) {
          double sign = nextHeading - curHeading > 0 ? 1 : -1;
          nextDs.setHeading(nextHeading - ( Math.PI * sign));
          changed = true;
        }
      }
    } while (changed);
    System.out.println("Time for heading : " + (System.currentTimeMillis() - sTime));

    if (reversed) {
      for (Entry<Double, DriveState> dds : traj.entrySet()) {
        DriveState ds = dds.getValue();
        ds.getLeftDrive().setPosition(-ds.getLeftDrive().getPosition());
        ds.getLeftDrive().setVelocity(-ds.getLeftDrive().getVelocity());
        ds.getLeftDrive().setAcceleration(-ds.getLeftDrive().getAcceleration());
        ds.getRightDrive().setPosition(-ds.getRightDrive().getPosition());
        ds.getRightDrive().setVelocity(-ds.getRightDrive().getVelocity());
        ds.getRightDrive().setAcceleration(-ds.getRightDrive().getAcceleration());
        State temp = ds.getLeftDrive();
        ds.setLeftDrive(ds.getRightDrive());
        ds.setRightDrive(temp);
      }
    }
    return traj;
  }

  private ArrayList<ArrayList<Double>> maxAcheivableVelocityForPath(Path path, RobotConstraints rc) {
    lastDistance = -1;
    lastT = -1;

    // vi=sqrt(vo^2+2ad)
    ArrayList<Double> avgVelocities = new ArrayList<>();
    ArrayList<ArrayList<Double>> velocities = new ArrayList<ArrayList<Double>>();
    velocities.add(new ArrayList<Double>());
    velocities.add(new ArrayList<Double>());
    velocities.add(new ArrayList<Double>());

    ArrayList<Double> ts = new ArrayList<>();
    double size = pointsPerPath * path.size();
    dD = path.getTotalDistance() / size;
    
    avgVelocities.add(0.0);
    ts.add(0.0);
    
    for (int i = 1; i < size; i++) {
      double t = sumToT(path, dD * i);
      ts.add(t);
      double lastVelocity = avgVelocities.get(i - 1);
      double accelerateableVelocity = Math.sqrt(lastVelocity * lastVelocity + 2 * rc.maxAcceleration * dD);
      double maxAcheiveableVelocityForCurvature = maximumAverageVelocityAtPoint(rc, path, t);
      avgVelocities.add(Math.min(accelerateableVelocity, maxAcheiveableVelocityForCurvature));
    }

    avgVelocities.set(avgVelocities.size() - 1, 0.0);
    for (int i = avgVelocities.size() - 2; i >= 0; i--) {
      double lastVelocity = avgVelocities.get(i + 1);
      double accelerateableVelocity = Math.sqrt(lastVelocity * lastVelocity + 2 * rc.maxAcceleration * dD);
      avgVelocities.set(i, Math.min(accelerateableVelocity, avgVelocities.get(i)));
    }

    for (int i = 0; i < ts.size(); i++) {
      double curvature = path.get((int) (ts.get(i).doubleValue())).getCurvature(ts.get(i) % 1);
      double radius = 1 / curvature;

      if (Double.isFinite(radius)) {
        velocities.get(0).add((avgVelocities.get(i) * 2) / ((radius + rc.wheelbase / 2) / (radius - rc.wheelbase / 2) + 1));
        velocities.get(1).add(avgVelocities.get(i) * 2 - velocities.get(0).get(i));
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

  static double lastDistance = -1;
  static double lastT = -1;

  private double sumToT(Path splines, double distance) {
    double i = 0;
    double dist = 0;

    if (lastDistance != -1) {
      dist = lastDistance;
      i = lastT;
    }

    while (dist < distance && i + dI < splines.size()) {
      dist += Math.sqrt(splines.get((int) i).dx(i % 1) * splines.get((int) i).dx(i % 1) + splines.get((int) i).dy(i % 1) * splines.get((int) i).dy(i % 1)) * dI;
      i += dI;
    }
    
    lastDistance = dist;
    lastT = i;

    return i;
  }

}