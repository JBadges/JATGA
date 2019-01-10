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
    double[][] avgVelocities = maxAcheivableVelocityForPath(path, rc);
    System.out.println("Time for maxAcheivableVelocityForPath : " + (System.currentTimeMillis() - time));
    time = System.currentTimeMillis();
    traj = generateTrajectory(path, reversed, rc, avgVelocities);
    System.out.println("Time for generateTrajectory : " + (System.currentTimeMillis() - time));
    return traj;
  }

  private Trajectory generateTrajectory(Path path, boolean reversed, RobotConstraints rc, double[][] velocites) {
    Trajectory traj = new Trajectory();
    traj.put(0.0, new DriveState(new State(0, 0, 0, rc.maxAcceleration), new State(0, 0, 0, rc.maxAcceleration), 0));

    double leftDist = 0;
    double rightDist = 0;
    double leftCur;
    double leftNext;
    double rightCur;
    double rightNext;

    double sTime = System.currentTimeMillis();
    for (int i = 0; i < velocites[0].length-1; i++) {
      leftCur = velocites[0][i];
      leftNext = velocites[0][i+1];
      rightCur = velocites[1][i];
      rightNext = velocites[1][i+1];
      double curAvgVel = (leftCur + rightCur) / 2;
      double nextAvgVel = (leftNext + rightNext) / 2;
      double dT = dD / ((curAvgVel + nextAvgVel)/2);
      leftDist += leftCur * dT;
      rightDist += rightCur * dT;
      double leftAcceleration = (leftNext - leftCur) / dT;
      double rightAcceleration = (rightNext - rightCur) / dT;
      double time = traj.lastEntry().getValue().getLeftDrive().getTime() + dT;

      traj.put(time, new DriveState(new State(time, leftDist, leftCur, leftAcceleration),
          new State(time, rightDist, rightCur, rightAcceleration), velocites[2][i]));
    }
    System.out.println("Time for traj : " + (System.currentTimeMillis() - sTime));
    
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

  private double[][] maxAcheivableVelocityForPath(Path path, RobotConstraints rc) {
    lastDistance = -1;
    lastT = -1;
    double size = pointsPerPath * path.size();
    dD = path.getTotalDistance() / size;

    // vf=sqrt(vo^2+2ad)
    double[] avgVelocities = new double[(int) size];
    double[][] velocities = new double[3][(int) size];

    ArrayList<Double> ts = new ArrayList<>();
    
    avgVelocities[0] = 0.0;
    ts.add(0.0);
    
    double sTime = System.currentTimeMillis();
    for (int i = 1; i < avgVelocities.length; i++) {
      double t = sumToT(path, dD * i);
      ts.add(t);
      double lastVelocity = avgVelocities[i - 1];
      double accelerateableVelocity = Math.sqrt(lastVelocity * lastVelocity + 2 * rc.maxAcceleration * dD);
      double maxAcheiveableVelocityForCurvature = maximumAverageVelocityAtPoint(rc, lastVelocity, ts.get(i-1), path, t);
      avgVelocities[i] = Math.min(accelerateableVelocity, maxAcheiveableVelocityForCurvature);
    }
    System.out.println("Time for forward prop: " + (System.currentTimeMillis() - sTime));

    sTime = System.currentTimeMillis();
    avgVelocities[avgVelocities.length-1] = 0.0;
    for (int i = avgVelocities.length - 2; i > 0; i--) {
      double lastVelocity = avgVelocities[i+1];
      double accelerateableVelocity = Math.sqrt(lastVelocity * lastVelocity + 2 * rc.maxAcceleration * dD);
      avgVelocities[i] = Math.min(accelerateableVelocity, avgVelocities[i]);
      avgVelocities[i] = Math.min(maximumAverageVelocityAtPoint(rc, lastVelocity, ts.get(i+1), path, ts.get(i)), avgVelocities[i]);
    }
    System.out.println("Time for backwards prop: " + (System.currentTimeMillis() - sTime));

    sTime = System.currentTimeMillis();
    for (int i = 0; i < ts.size(); i++) {
      double curvature = path.get((int) (ts.get(i).doubleValue())).getCurvature(ts.get(i) % 1);
      double radius = 1 / curvature;

      if (Double.isFinite(radius)) {
        velocities[0][i] = ((avgVelocities[i] * 2) / ((radius + rc.wheelbase / 2) / (radius - rc.wheelbase / 2) + 1));
        velocities[1][i] = avgVelocities[i] * 2 - velocities[0][i];
      } else {
        velocities[0][i] = avgVelocities[i];
        velocities[1][i] = avgVelocities[i];
      }
      velocities[2][i] = path.get((int) (ts.get(i).doubleValue())).getHeading(ts.get(i) % 1);
    }
    System.out.println("Time for velocities: " + (System.currentTimeMillis() - sTime));

    return velocities;
  }

  private double maximumAverageVelocityAtPoint(RobotConstraints constraints, double lastVel, double lastT, Path path, double t) {
    // Constraints
    //  |left vel | <= Vmax & |right vel | <= Vmax 
    // | (Vleft + Vright) / 2 | <= Vmax
    // Vleft / Vright = (r+w)/(r-w) | Vleft = (r+w)/(r-w) * Vright
    // (Vright - Vleft) / w = Vangular
    // |left accel | <= Amax & |right accel | <= Amax

    double topAvgVel = -1;
    double topLeftVel = 0;
    double topRightVel = 0;
    double lastLeftVel = 0;
    double lastRightVel = 0;


    double radius = 1 / path.get((int) t).getCurvature(t % 1);
    if (!Double.isFinite(radius)) {
      topAvgVel = constraints.maxVelocity;
      topLeftVel = constraints.maxVelocity;
      topRightVel = constraints.maxVelocity;
    } else {
      topAvgVel = (Math.abs(radius) * constraints.maxVelocity) / (Math.abs(radius) + constraints.wheelbase / 2);
      topLeftVel = ((topAvgVel * 2) / ((radius + constraints.wheelbase / 2) / (radius - constraints.wheelbase / 2) + 1));
      topRightVel = topAvgVel * 2 - topLeftVel;
    }

    double lastRadius = 1 / path.get((int) lastT).getCurvature(lastT % 1);
    double dLeft;
    double dRight;
    if (!Double.isFinite(lastRadius)) {
      lastLeftVel = lastVel;
      lastRightVel = lastVel;
      dLeft = dD;
      dRight = dD;
    } else {
      lastLeftVel = (lastVel * 2) / ((lastRadius + constraints.wheelbase / 2) / (lastRadius - constraints.wheelbase / 2) + 1);
      lastRightVel = lastVel * 2 - lastLeftVel;
      dLeft = (1+constraints.wheelbase/(2*lastRadius))*dD;
      dRight = (1-constraints.wheelbase/(2*lastRadius))*dD;
    }
    
    double maxLeft = Math.sqrt(lastLeftVel * lastLeftVel + 2 * constraints.maxAcceleration * dLeft);
    double maxRight = Math.sqrt(lastRightVel * lastRightVel + 2 * constraints.maxAcceleration * dRight);
    if(Math.abs(maxLeft) < Math.abs(topLeftVel) || Math.abs(maxRight) < Math.abs(topRightVel)) {
      double scalar = Math.min(Math.abs(maxRight)/Math.abs(topRightVel),Math.abs(maxLeft)/Math.abs(topLeftVel));
      topLeftVel *= scalar;
      topRightVel *= scalar;
    }

    return (topLeftVel + topRightVel)/2;
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