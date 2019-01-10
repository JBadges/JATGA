import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.MathUtils;
import org.junit.Test;

import math.Point;
import trajectory.RobotConstraints;
import trajectory.State;
import trajectory.Trajectory;
import trajectory.TrajectoryGenerator;

import static org.hamcrest.Matchers.lessThanOrEqualTo;
import static org.hamcrest.MatcherAssert.assertThat;

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

    Point[] pathE = { new Point(0, 0, 0), new Point(10, 0, 0) };
    trajs = new TrajectoryGenerator().generate(rc, true, pathE);
    assertEquals(-10, trajs.lastEntry().getValue().getLeftDrive().getPosition(), 1e-2);
    assertEquals(-10, trajs.lastEntry().getValue().getRightDrive().getPosition(), 1e-2);
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
  public void initalHeadingTest() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, Math.PI/2), new Point(10, 0, 0) };
    Trajectory trajs = new TrajectoryGenerator().generate(rc, false, pathA);

    List<Point> genPath = testPath(trajs, rc, Math.PI/2);
    // Accurate to 1 cm
    assertEquals(10, genPath.get(genPath.size() - 1).getX(), 1e-2);
    assertEquals(0, genPath.get(genPath.size() - 1).getY(), 1e-2);

    Point[] pathB = { new Point(0, 0, Math.PI), new Point(0, 10, Math.PI / 2), new Point(15, 20, 0),
      new Point(5, 10, Math.PI) };
    trajs = new TrajectoryGenerator().generate(rc, false, pathB);

    genPath = testPath(trajs, rc, Math.PI);
    // Accurate to 1 cm
    assertEquals(5, genPath.get(genPath.size() - 1).getX(), 1e-2);
    assertEquals(10, genPath.get(genPath.size() - 1).getY(), 1e-2);
  }

  @Test
  public void totalTest() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 2), new Point(5, 10, 1), new Point(6, 15, 0) };
    Trajectory trajs = new TrajectoryGenerator().generate(rc, false, pathA);

    List<Point> genPath = testPath(trajs, rc);
    double finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1 cm
    assertEquals(6, genPath.get(genPath.size() - 1).getX(), 1e-2);
    // Accurate to 1 cm
    assertEquals(15, genPath.get(genPath.size() - 1).getY(), 1e-2);
    // Accurate to 1/100 of a radian
    assertEquals(0, finalHeading > Math.PI ? finalHeading - Math.PI*2 : finalHeading, 1e-2);
  }

  @Test
  public void constraints() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 2), new Point(5, 20, 1), new Point(10, 23, 0) };
    Trajectory traj = new TrajectoryGenerator().generate(rc, false, pathA);

    testConstraints(traj, rc);

    Point[] pathB = { new Point(0, 0, 0), new Point(10, 3, 2), new Point(5, 20, -1), new Point(10, 23, 0) };
    traj = new TrajectoryGenerator().generate(rc, false, pathB);

    testConstraints(traj, rc);

    traj = new TrajectoryGenerator().generate(rc, false, new Point(0, 0, 0),
    new Point(10, 10, Math.PI),
    new Point(20, -20, Math.PI*2),
    new Point(30, 30, 0));

    testConstraints(traj, rc);
  }

  private void testConstraints(Trajectory traj, RobotConstraints rc) {
    double dT = 0.002;
    for(double t = 0; t < traj.lastKey(); t += dT) {
      State left = traj.getInterpolated(t).getLeftDrive();
      assertThat(Math.abs(left.getVelocity()), lessThanOrEqualTo(rc.maxVelocity * 1.05));
      assertThat(Math.abs(left.getAcceleration()), lessThanOrEqualTo(rc.maxAcceleration * 1.05));

      State right = traj.getInterpolated(t).getRightDrive();
      assertThat(Math.abs(right.getVelocity()), lessThanOrEqualTo(rc.maxVelocity * 1.05));
      assertThat(Math.abs(right.getAcceleration()), lessThanOrEqualTo(rc.maxAcceleration * 1.05));

      assertThat(Math.abs((right.getVelocity()-left.getVelocity())/rc.wheelbase), lessThanOrEqualTo(rc.maxAngular * 1.05));    
    }
  }

  private List<Point> testPath(Trajectory trajs, RobotConstraints rc) {
    return testPath(trajs, rc, 0);
  }

  private List<Point> testPath(Trajectory trajs, RobotConstraints rc, double initHeading) {
    List<Point> genPath = new ArrayList<>();
    genPath.add(new Point(0, 0, initHeading));
    double heading = initHeading;
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
  
}