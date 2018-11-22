import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import math.Point;
import trajectory.RobotConstraints;
import trajectory.TrajectoryGenerator;
import trajectory.TrajectoryPoint;

public class Main {

	public static void main(String[] args) throws IOException {
		RobotConstraints rc = new RobotConstraints(3, 3, 3, 0.55926);
        double sTime = System.currentTimeMillis();
        TrajectoryGenerator.setPointsPerPath(100_000);
        TrajectoryGenerator.setDI(1e-6);
        TrajectoryPoint[][] trajs = TrajectoryGenerator.generate(rc, false,
        		new Point(0,0,0), new Point(3.3, 1.12912, 0));
		System.out.printf("Took %.2fs to generate the trajectories", (System.currentTimeMillis() - sTime)/1000.0);
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
                newPathWriter.write(genPath.get(i).getX() + ", " + genPath.get(i).getY() + ", " + genPath.get(i).getHeading() + "\n");
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
	}
	
}
