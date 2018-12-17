import static org.junit.Assert.assertEquals;
import static org.junit.Assume.assumeThat;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;

import org.apache.commons.math3.util.MathUtils;
import org.hamcrest.Matchers;
import org.junit.Test;

import math.Point;
import trajectory.RobotConstraints;
import trajectory.State;
import trajectory.Trajectory;
import trajectory.TrajectoryGenerator;

public class TrajectoryTest {

  @Test
  public void straightPathAccuracy() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 0) };
    Trajectory trajs = new TrajectoryGenerator().generate(rc, false, pathA);
    List<Point> genPath = testPath(trajs, rc);

    // Accurate to 1 cm
    assertEquals(10, genPath.get(genPath.size() - 1).getX(), 1e-2);

    Point[] pathB = { new Point(0, 0, 0), new Point(10, 0, 0) };
    trajs = new TrajectoryGenerator().generate(rc, true, pathB);

    genPath = testPath(trajs, rc);
    // Accurate to 1 cm
    assertEquals(-10, genPath.get(genPath.size() - 1).getX(), 1e-2);

    Point[] pathC = { new Point(0, 0, Math.PI / 2), new Point(0, 10, Math.PI / 2) };
    trajs = new TrajectoryGenerator().generate(rc, false, pathC);

    genPath = testPath(trajs, rc);
    // Accurate to 1 cm
    assertEquals(10, genPath.get(genPath.size() - 1).getX(), 1e-2);

    Point[] pathD = { new Point(0, 0, Math.PI / 2), new Point(0, 10, Math.PI / 2) };
    trajs = new TrajectoryGenerator().generate(rc, true, pathD);

    genPath = testPath(trajs, rc);
    // Accurate to 1 cm
    assertEquals(-10, genPath.get(genPath.size() - 1).getX(), 1e-2);
  }

  @Test
  public void headingTest() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 0) };
    Trajectory trajs = new TrajectoryGenerator().generate(rc, false, pathA);

    List<Point> genPath = testPath(trajs, rc);
    double finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1/100 of a radian
    assertEquals(0, finalHeading, 1e-2);

    Point[] pathB = { new Point(0, 0, 0), new Point(4, 10, Math.PI / 2) };
    trajs = new TrajectoryGenerator().generate(rc, true, pathB);

    genPath = testPath(trajs, rc);
    finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1/100 of a radian
    assertEquals(Math.PI / 2, finalHeading, 1e-2);

    Point[] pathC = { new Point(0, 0, 0), new Point(0, 10, Math.PI / 3), new Point(15, 20, Math.PI / 2),
        new Point(5, 10, Math.PI) };
    trajs = new TrajectoryGenerator().generate(rc, true, pathC);

    genPath = testPath(trajs, rc);
    finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1/100 of a radian
    assertEquals(Math.PI, finalHeading, 1e-2);

    Point[] pathD = { new Point(0, 0, Math.PI), new Point(0, 10, Math.PI / 2), new Point(15, 20, 0),
        new Point(5, 10, Math.PI) };
    trajs = new TrajectoryGenerator().generate(rc, true, pathD);

    genPath = testPath(trajs, rc);
    finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1/100 of a radian
    assertEquals(0, MathUtils.normalizeAngle(trajs.lastEntry().getValue().getHeading(), Math.PI), 1e-2);
    assertEquals(0, finalHeading, 1e-2);
  }

  @Test
  public void totalTest() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 2), new Point(5, 20, 1), new Point(10, 23, 0) };
    Trajectory trajs = new TrajectoryGenerator().generate(rc, false, pathA);

    List<Point> genPath = testPath(trajs, rc);
    double finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1 cm
    assertEquals(10, genPath.get(genPath.size() - 1).getX(), 1e-2);
    // Accurate to 1 cm
    assertEquals(23, genPath.get(genPath.size() - 1).getY(), 1e-2);
    // Accurate to 1/100 of a radian
    assertEquals(0, finalHeading > Math.PI ? 0 : finalHeading, 1e-2);
  }

  private List<Point> testPath(Trajectory trajs, RobotConstraints rc) {
    List<Point> genPath = new ArrayList<>();
    genPath.add(new Point(0, 0, 0));
    double heading = 0;
    double dT = 0.002;
    for (double t = 0; t < trajs.lastEntry().getKey(); t += dT) {
      State left = trajs.getInterpolated(t).getLeftDrive();
      State right = trajs.getInterpolated(t).getRightDrive();
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
    return genPath;
  }

  @Test
  public void constraints() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 2), new Point(5, 20, 1), new Point(10, 23, 0) };
    Trajectory traj = new TrajectoryGenerator().generate(rc, false, pathA);

    testConstraints(traj, rc);

    rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathB = { new Point(0, 0, 0), new Point(10, 3, 2), new Point(5, 20, -1), new Point(10, 23, 0) };
    traj = new TrajectoryGenerator().generate(rc, false, pathB);

    testConstraints(traj, rc);
  }

  private void testConstraints(Trajectory traj, RobotConstraints rc) {
    double dT = 0.002;
    for(double t = 0; t < traj.lastKey(); t += dT) {
      State left = traj.getInterpolated(t).getLeftDrive();
      assumeThat(Math.abs(left.getVelocity()), Matchers.lessThanOrEqualTo(rc.maxVelocity));
      assumeThat(Math.abs(left.getAcceleration()), Matchers.lessThanOrEqualTo(rc.maxAcceleration));

      State right = traj.getInterpolated(t).getRightDrive();
      assumeThat(Math.abs(right.getVelocity()), Matchers.lessThanOrEqualTo(rc.maxVelocity));
      assumeThat(Math.abs(right.getAcceleration()), Matchers.lessThanOrEqualTo(rc.maxAcceleration));

      assumeThat(Math.abs((right.getVelocity()-left.getVelocity())/rc.wheelbase), Matchers.lessThanOrEqualTo(rc.maxAngular));
    }
  }

}