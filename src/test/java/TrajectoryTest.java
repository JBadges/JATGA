import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.MathUtils;
import org.junit.Test;

import math.Point;
import trajectory.RobotConstraints;
import trajectory.TrajectoryGenerator;
import trajectory.TrajectoryPoint;

public class TrajectoryTest {

  @Test
  public void straightPathAccuracy() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 0) };
    TrajectoryPoint[][] trajs = TrajectoryGenerator.generate(rc, false, pathA);

    List<Point> genPath = testPath(trajs, rc);
    // Accurate to 1 cm
    assertEquals(10, genPath.get(genPath.size() - 1).getX(), 1e-2);

    Point[] pathB = { new Point(0, 0, 0), new Point(10, 0, 0) };
    trajs = TrajectoryGenerator.generate(rc, true, pathB);

    genPath = testPath(trajs, rc);
    // Accurate to 1 cm
    assertEquals(-10, genPath.get(genPath.size() - 1).getX(), 1e-2);

    Point[] pathC = { new Point(0, 0, Math.PI / 2), new Point(0, 10, Math.PI / 2) };
    trajs = TrajectoryGenerator.generate(rc, false, pathC);

    genPath = testPath(trajs, rc);
    // Accurate to 1 cm
    assertEquals(10, genPath.get(genPath.size() - 1).getX(), 1e-2);

    Point[] pathD = { new Point(0, 0, Math.PI / 2), new Point(0, 10, Math.PI / 2) };
    trajs = TrajectoryGenerator.generate(rc, true, pathD);

    genPath = testPath(trajs, rc);
    // Accurate to 1 cm
    assertEquals(-10, genPath.get(genPath.size() - 1).getX(), 1e-2);
  }

  @Test
  public void headingTest() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 0) };
    TrajectoryPoint[][] trajs = TrajectoryGenerator.generate(rc, false, pathA);

    List<Point> genPath = testPath(trajs, rc);
    double finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1/100 of a radian
    assertEquals(0, finalHeading, 1e-2);

    Point[] pathB = { new Point(0, 0, 0), new Point(0, 10, Math.PI / 2) };
    trajs = TrajectoryGenerator.generate(rc, true, pathB);

    genPath = testPath(trajs, rc);
    finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1/100 of a radian
    assertEquals(Math.PI / 2, finalHeading, 1e-2);

    Point[] pathC = { new Point(0, 0, 0), new Point(0, 10, Math.PI / 2), new Point(15, 20, 0),
        new Point(5, 10, Math.PI) };
    trajs = TrajectoryGenerator.generate(rc, true, pathC);

    genPath = testPath(trajs, rc);
    finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1/100 of a radian
    assertEquals(Math.PI, finalHeading, 1e-2);
  }

  @Test
  public void totalTest() {
    RobotConstraints rc = new RobotConstraints(3, 5, 2, 1);
    Point[] pathA = { new Point(0, 0, 0), new Point(10, 0, 2), new Point(5, 20, 1), new Point(10, 23, 0) };
    TrajectoryPoint[][] trajs = TrajectoryGenerator.generate(rc, false, pathA);

    List<Point> genPath = testPath(trajs, rc);
    double finalHeading = MathUtils.normalizeAngle(genPath.get(genPath.size() - 1).getHeading(), Math.PI);
    // Accurate to 1 cm
    assertEquals(10, genPath.get(genPath.size() - 1).getX(), 1e-2);
    // Accurate to 1 cm
    assertEquals(23, genPath.get(genPath.size() - 1).getY(), 1e-2);
    // Accurate to 1/100 of a radian
    assertEquals(0, finalHeading, 1e-2);
  }

  private List<Point> testPath(TrajectoryPoint[][] trajs, RobotConstraints rc) {
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
    return genPath;
  }

}