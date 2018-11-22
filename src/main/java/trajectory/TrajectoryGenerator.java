package trajectory;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.TreeMap;

import math.LinearRegression;
import math.Point;
import math.spline.Path;
import math.spline.QuinticHermiteSpline;

public class TrajectoryGenerator {

  private static double pointsPerPath = 100_000;
  private static double dI = 1e-6;

  public static TrajectoryPoint[][] generate(RobotConstraints rc, boolean isReversed, Point... path) {
    List<QuinticHermiteSpline> p = new ArrayList<>();
    for (int i = 1; i < path.length; i++) {
      p.add(new QuinticHermiteSpline(path[i - 1], path[i]));
    }
    return generate(rc, isReversed, "", new Path(p));
  }

  public static TrajectoryPoint[][] generate(RobotConstraints rc, boolean isReversed, String filename, Point... path) {
    List<QuinticHermiteSpline> p = new ArrayList<>();
    for (int i = 1; i < path.length; i++) {
      p.add(new QuinticHermiteSpline(path[i - 1], path[i]));
    }
    return generate(rc, isReversed, filename, new Path(p));
  }

  public static TrajectoryPoint[][] generate(RobotConstraints rc, boolean isReversed, String filename, Path path) {
    VelocityDistanceTPoint[] v = TrajectoryGenerator.generateAverageTrajectoryVsDistance(rc, path);
    VelocityTime[] vt = TrajectoryGenerator.generateAverageTrajectoryVsTime(v);
    VelocityTime[][] skids = TrajectoryGenerator.getSkidVelocities(rc, vt, path);
    VelocityTime[][] traj = TrajectoryGenerator.getNormalizedSkidVelocities(skids, 20 / 1000.0);
    System.out.println("generate");

    TrajectoryPoint[][] followableTrajectory = new TrajectoryPoint[2][traj[0].length];
    final double dT = traj[0][1].time - traj[0][0].time;
    followableTrajectory[0][0] = new TrajectoryPoint(0, 0, 0, 0, 0, dT);
    followableTrajectory[1][0] = new TrajectoryPoint(0, 0, 0, 0, 0, dT);
    double initHeading = path.get((int) traj[0][0].t).getHeading(traj[0][0].t % 1);
    for (int i = 1; i < followableTrajectory[0].length; i++) {
      followableTrajectory[0][i] = new TrajectoryPoint(i * dT,
          followableTrajectory[0][i - 1].getPosition() + traj[0][i].velocity * dT, traj[0][i].velocity,
          (traj[0][i].velocity - followableTrajectory[0][i - 1].getVelocity()) / dT,
          path.get((int) traj[0][i].t).getHeading(traj[0][i].t % 1) - initHeading, dT);
      followableTrajectory[1][i] = new TrajectoryPoint(i * dT,
          followableTrajectory[1][i - 1].getPosition() + traj[1][i].velocity * dT, traj[1][i].velocity,
          (traj[1][i].velocity - followableTrajectory[1][i - 1].getVelocity()) / dT,
          path.get((int) traj[1][i].t).getHeading(traj[1][i].t % 1) - initHeading, dT);
    }
    followableTrajectory[0][followableTrajectory[0].length - 1]
        .setHeading(followableTrajectory[0][followableTrajectory[0].length - 2].getHeading());
    followableTrajectory[1][followableTrajectory[1].length - 1]
        .setHeading(followableTrajectory[1][followableTrajectory[1].length - 2].getHeading());
    if (isReversed) {
      TrajectoryPoint[] temp = followableTrajectory[0];
      followableTrajectory[0] = followableTrajectory[1];
      followableTrajectory[1] = temp;
      for (int i = 0; i < followableTrajectory[0].length; i++) {
        followableTrajectory[0][i].setPosition(-followableTrajectory[0][i].getPosition());
        followableTrajectory[0][i].setVelocity(-followableTrajectory[0][i].getVelocity());
        followableTrajectory[0][i].setAcceleration(-followableTrajectory[0][i].getAcceleration());
        followableTrajectory[1][i].setPosition(-followableTrajectory[1][i].getPosition());
        followableTrajectory[1][i].setVelocity(-followableTrajectory[1][i].getVelocity());
        followableTrajectory[1][i].setAcceleration(-followableTrajectory[1][i].getAcceleration());
      }
      // //Invert heading trajectory
      // for(int i = 0; i < followableTrajectory[0].length/2; i++) {
      // double tempHeading = followableTrajectory[0][i].getHeading();
      // followableTrajectory[0][i].setHeading(followableTrajectory[0][followableTrajectory[0].length-i-1].getHeading());
      // followableTrajectory[0][followableTrajectory[0].length-i-1].setHeading(tempHeading);
      // followableTrajectory[1][i].setHeading(followableTrajectory[1][followableTrajectory[1].length-i-1].getHeading());
      // followableTrajectory[1][followableTrajectory[1].length-i-1].setHeading(tempHeading);
      // }
      // double initH = followableTrajectory[0][0].getHeading();
      // for(int i = 0; i < followableTrajectory[0].length; i++) {
      // followableTrajectory[0][i].setHeading(followableTrajectory[0][i].getHeading()
      // - initH);
      // followableTrajectory[1][i].setHeading(followableTrajectory[1][i].getHeading()
      // - initH);
      // }
    }

    for (int i = 0; i < followableTrajectory[0].length; i++) {
      followableTrajectory[0][i].setHeading(Point.normalize(followableTrajectory[0][i].getHeading(), Math.PI));
      followableTrajectory[1][i].setHeading(Point.normalize(followableTrajectory[1][i].getHeading(), Math.PI));
    }
    boolean changed;
    int count = 0;
    do {
      count++;
      changed = false;
      for (int i = 0; i < followableTrajectory[0].length - 1; i++) {
        if (Math.abs(followableTrajectory[0][i + 1].getHeading() - followableTrajectory[0][i].getHeading()) > Math.PI
            / 2.0) {
          double sign = followableTrajectory[0][i + 1].getHeading() - followableTrajectory[0][i].getHeading() > 0 ? 1
              : -1;
          followableTrajectory[0][i + 1].setHeading(followableTrajectory[0][i + 1].getHeading() - (Math.PI * sign));
          changed = true;
        }
        if (followableTrajectory[1][i + 1].getHeading() - followableTrajectory[1][i].getHeading() > Math.PI / 2.0) {
          double sign = followableTrajectory[1][i + 1].getHeading() - followableTrajectory[1][i].getHeading() > 0 ? 1
              : -1;
          followableTrajectory[1][i + 1].setHeading(followableTrajectory[1][i + 1].getHeading() - (Math.PI * sign));
          changed = true;
        }
      }
    } while (changed);
    System.out.println("Continuized path " + count + " times");

    if (!(filename == null || filename == "")) {
      FileWriter fw = null;
      try {
        fw = new FileWriter(new File("Left_" + filename + ".csv"));
        fw.flush();
        fw.write("Time, Pos, Vel, Acc, Heading, Timestep \n");
        for (int i = 0; i < followableTrajectory[0].length; i++) {
          fw.write(followableTrajectory[0][i].toString());
        }
        fw.close();
      } catch (Exception e) {

      }
      try {
        fw = new FileWriter(new File("Right_" + filename + ".csv"));
        fw.flush();
        fw.write("Time, Pos, Vel, Acc, Heading, Timestep \n");
        for (int i = 0; i < followableTrajectory[1].length; i++) {
          fw.write(followableTrajectory[1][i].toString());
        }
        fw.close();
      } catch (Exception e) {

      }
    }
    return followableTrajectory;
  }

  public static VelocityTime[][] getNormalizedSkidVelocities(VelocityTime[][] skidTraj, double wantedDt) {
    System.out.println("getNormalizedSkidVelocities");
    // Sum all the skidTraj dt until just under dt - linreg for the dt wanted cap at
    // next seg loop and repeat
    int loopTo = (int) (skidTraj[0][skidTraj[0].length - 1].time / wantedDt);

    VelocityTime[][] normVt = new VelocityTime[2][loopTo + 1];
    for (int i = 0; i < normVt[0].length; i++) {
      normVt[0][i] = new VelocityTime();
      normVt[1][i] = new VelocityTime();
    }

    double[] dtArr = new double[skidTraj[0].length - 1];
    for (int ind = 1; ind < dtArr.length; ind++) {
      dtArr[ind - 1] = skidTraj[0][ind].time - skidTraj[0][ind - 1].time;
    }

    int dtIndex = 0;
    int loopCounter = 0;
    while (loopCounter < loopTo) {
      double dtSum = 0;
      int startIndex = dtIndex;
      while (dtSum < wantedDt) {
        dtSum += dtArr[dtIndex];
        dtIndex++;
      }
      dtSum -= dtArr[dtIndex];
      dtIndex--;

      int endIndex = dtIndex;
      double vL[] = new double[endIndex - startIndex];
      double tL[] = new double[endIndex - startIndex];
      double vR[] = new double[endIndex - startIndex];
      double tR[] = new double[endIndex - startIndex];
      for (int k = startIndex; k < endIndex; k++) {
        vL[k - startIndex] = skidTraj[0][k].velocity;
        tL[k - startIndex] = skidTraj[0][k].time;
        vR[k - startIndex] = skidTraj[1][k].velocity;
        tR[k - startIndex] = skidTraj[1][k].time;
      }
      LinearRegression lrL = new LinearRegression(tL, vL);
      normVt[0][loopCounter].time = loopCounter * wantedDt;
      normVt[0][loopCounter].velocity = lrL.predict((loopCounter) * wantedDt);
      normVt[0][loopCounter].t = skidTraj[0][endIndex].t;
      LinearRegression lrR = new LinearRegression(tR, vR);
      normVt[1][loopCounter].time = loopCounter * wantedDt;
      normVt[1][loopCounter].velocity = lrR.predict((loopCounter) * wantedDt);
      normVt[1][loopCounter].t = skidTraj[1][endIndex].t;

      normVt[0][loopCounter].velocity = Double.isFinite(normVt[0][loopCounter].velocity)
          ? normVt[0][loopCounter].velocity
          : 0;
      normVt[1][loopCounter].velocity = Double.isFinite(normVt[1][loopCounter].velocity)
          ? normVt[1][loopCounter].velocity
          : 0;

      loopCounter++;
    }

    normVt[0][loopCounter].time = normVt[0][loopCounter - 1].time + wantedDt;
    normVt[1][loopCounter].time = normVt[1][loopCounter - 1].time + wantedDt;

    // FileWriter fw = null;
    // try {
    // fw = new FileWriter(new File("normLeft.csv"));
    // fw.flush();
    // fw.write("Time, Left Vel, Right Vel, Vel avg \n");
    // for (int id = 0; id < normVt[0].length; id++) {
    // fw.write(normVt[0][id].time + ", " + normVt[0][id].velocity + ", " +
    // normVt[1][id].velocity + ", "
    // + (normVt[0][id].velocity + normVt[1][id].velocity) / 2 + ",\n");
    // }
    // fw.close();
    // } catch (Exception e) {
    //
    // }

    return normVt;
  }

  public static VelocityTime[][] getSkidVelocities(RobotConstraints constraints, VelocityTime[] vt, Path path) {
    System.out.println("getSkidVelocities");
    VelocityTime[][] leftAndRight = new VelocityTime[2][vt.length];

    for (int i = 0; i < leftAndRight[0].length; i++) {
      leftAndRight[0][i] = new VelocityTime();
      leftAndRight[1][i] = new VelocityTime();
    }

    for (int i = 0; i < vt.length; i++) {
      double curvature = path.get((int) (vt[i].t)).getCurvature(vt[i].t % 1);
      double radius = 1 / curvature;
      double leftVelocity;
      double rightVelocity;

      if (Double.isFinite(radius)) {
        leftVelocity = (vt[i].velocity * 2)
            / ((radius + constraints.wheelbase / 2) / (radius - constraints.wheelbase / 2) + 1);
        rightVelocity = vt[i].velocity * 2 - leftVelocity;
      } else {
        leftVelocity = vt[i].velocity;
        rightVelocity = vt[i].velocity;
      }

      leftAndRight[0][i].velocity = leftVelocity;
      leftAndRight[0][i].time = vt[i].time;
      leftAndRight[0][i].t = vt[i].t;
      leftAndRight[1][i].velocity = rightVelocity;
      leftAndRight[1][i].time = vt[i].time;
      leftAndRight[1][i].t = vt[i].t;
    }

    leftAndRight[0][0].time = 0;
    leftAndRight[0][leftAndRight[0].length - 1].time = leftAndRight[0][leftAndRight[0].length - 2].time;
    leftAndRight[0][0].velocity = 0;
    leftAndRight[1][0].time = 0;
    leftAndRight[1][leftAndRight[1].length - 1].time = leftAndRight[1][leftAndRight[1].length - 2].time;
    leftAndRight[1][0].velocity = 0;

    // FileWriter fw = null;
    // try {
    // fw = new FileWriter(new File("left&right.csv"));
    // fw.flush();
    // fw.write("Time, Left Vel, Right Vel, Vel avg \n");
    // for(int i = 0; i < leftAndRight[0].length; i++) {
    // fw.write(leftAndRight[0][i].time + ", " + leftAndRight[0][i].velocity + ", "
    // + leftAndRight[1][i].velocity + ", " + (leftAndRight[0][i].velocity +
    // leftAndRight[1][i].velocity)/2 + ",\n");
    // }
    // fw.close();
    // } catch (Exception e) {

    // }
    return leftAndRight;
  }

  public static VelocityTime[] generateAverageTrajectoryVsTime(VelocityDistanceTPoint[] avgTrajVsDist) {
    System.out.println("generateAverageTrajectoryVsTime");
    VelocityTime[] vt = new VelocityTime[avgTrajVsDist.length];

    for (int i = 0; i < vt.length; i++) {
      vt[i] = new VelocityTime();
    }

    for (int i = 1; i < avgTrajVsDist.length; i++) {
      double changeInDistance = avgTrajVsDist[i].distance - avgTrajVsDist[i - 1].distance;
      double changeInTime = changeInDistance / avgTrajVsDist[i].velocity;
      vt[i].velocity = changeInDistance / changeInTime;
      vt[i].time = vt[i - 1].time + changeInTime;
      vt[i].t = avgTrajVsDist[i].t;
    }
    // FileWriter fw = null;
    // try {
    // fw = new FileWriter(new File("veltime.csv"));
    // fw.flush();
    // fw.write("Velocity, Time, \n");
    // for(int i = 0; i < vt.length; i++) {
    // fw.write(vt[i].velocity + "," + vt[i].time + ",\n");
    // }
    // fw.close();
    // } catch (Exception e) {

    // }
    return vt;
  }

  /**
   * Idea from poofs
   * https://docs.google.com/presentation/d/1xjtQ5m3Ay4AYxS_SfloF2n_vWZnCU25aXZuu9A59xPY/pub?start=false&loop=false&delayms=3000&slide=id.g998959438_1_338
   * uses forward and backward propogation to calculate the fastest your robot can
   * be at any point in time given robot constraints
   * 
   * @param constraints
   * @param path
   * @return
   */
  public static VelocityDistanceTPoint[] generateAverageTrajectoryVsDistance(RobotConstraints constraints, Path path) {
    System.out.println("generateAverageTrajectoryVsDistance");
    double dD = path.getTotalDistance() / pointsPerPath;
    // Vf^2 = Vi^2 + 2*a*d
    // Vf = sqrt(Vi^2+2*a*d)
    VelocityDistanceTPoint[] maxVelocity = new VelocityDistanceTPoint[(int) (path.getTotalDistance() / dD)];
    for (int i = 0; i < maxVelocity.length; i++) {
      maxVelocity[i] = new VelocityDistanceTPoint();
    }

    // Forward Propagation
    for (int i = 1; i < maxVelocity.length; i++) {
      double currentVelocity = maxVelocity[i - 1].velocity;
      double maxReachable = Math.sqrt(currentVelocity * currentVelocity + 2 * constraints.maxAcceleration * dD);
      double maxConstrained = Math.min(maxReachable, maximumAverageVelocityAtPoint(constraints, path, i * dD, true));
      maxVelocity[i].velocity = maxConstrained;
      maxVelocity[i].distance = i * dD;
      maxVelocity[i].t = sumToT(path, maxVelocity[i].distance, true);
    }

    // End at zero velocity
    maxVelocity[maxVelocity.length - 1].velocity = 0;

    // Backwards Propagation
    for (int i = maxVelocity.length - 2; i >= 0; i--) {
      double currentVelocity = maxVelocity[i + 1].velocity;
      double maxReachable = Math.sqrt(currentVelocity * currentVelocity + 2 * constraints.maxAcceleration * dD);
      double maxConstrained = Math.min(maxReachable, maxVelocity[i].velocity);
      maxVelocity[i].velocity = maxConstrained;
      maxVelocity[i].distance = i * dD;
      maxVelocity[i].t = sumToT(path, maxVelocity[i].distance, false);
    }
    // FileWriter fw = null;
    // try {
    // fw = new FileWriter(new File("veldist.csv"));
    // fw.flush();
    // fw.write("Velocity, Distance, \n");
    // for(int i = 0; i < maxVelocity.length; i++) {
    // fw.write(maxVelocity[i].velocity + "," + maxVelocity[i].distance + ",\n");
    // }
    // fw.close();
    // } catch (Exception e) {

    // }
    return maxVelocity;
  }

  private static double maximumAverageVelocityAtPoint(RobotConstraints constraints, Path path, double distance,
      boolean increasing) {
    // Constraints
    // left and right velocites <= Vmax | (Vleft + Vright) / 2 = Vmax
    // Vleft / Vright = (r+w)/(r-w) | Vleft = (r+w)/(r-w) * Vright
    // (Vright - Vleft) / w = Vangular

    double i = sumToT(path, distance, increasing);
    double radius = 1 / path.get((int) i).getCurvature(i % 1);
    if (!Double.isFinite(radius)) {
      return constraints.maxVelocity;
    }
    return (Math.abs(radius) * constraints.maxVelocity) / (Math.abs(radius) + constraints.wheelbase / 2);
  }

  public static void setDI(double i) {
    dI = i;
  }

  public static void setPointsPerPath(double ppp) {
    pointsPerPath = ppp;
  }

  static TreeMap<Double, Double> sumT = new TreeMap<>();

  private static double sumToT(Path splines, double distance, boolean increasing) {
    double i = 0;
    double dist = 0;

    if (increasing && !sumT.isEmpty()) {
      dist = sumT.lastEntry().getKey();
      i = sumT.lastEntry().getValue();
    } else if (!sumT.isEmpty()) {
      Entry<Double, Double> entry = sumT.pollLastEntry();
      dist = entry.getKey();
      i = entry.getValue();
    }
    while (dist < distance && i + dI < splines.size()) {
      Point a = splines.get((int) Math.floor(i)).getPoint(i % 1);
      i += dI;
      Point b = splines.get((int) Math.floor(i)).getPoint(i % 1);
      dist += a.distance(b);
    }
    if (increasing) {
      sumT.put(dist, i);
    }
    return i;
  }

}

class VelocityDistanceTPoint {
  double velocity;
  double distance;
  double t;
}

class VelocityTime {
  double velocity;
  double time;
  double t;
}
