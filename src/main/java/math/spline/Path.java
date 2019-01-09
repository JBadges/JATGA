package math.spline;

import java.util.List;

import math.Point;

public class Path {

	private List<QuinticHermiteSpline> path;
	private final double dT = 3e-4;
	private double distance = -1;

	public Path(List<QuinticHermiteSpline> path) {
		this.path = path;
	}

	public Point getPoint(double i) {
		return path.get((int) i).getPoint(i % 1);
	}

	public QuinticHermiteSpline get(int i) {
		return path.get(i);
	}

	public int size() {
		return path.size();
	}

	public double getTotalDistance() {
		if(distance != -1) {
			return distance;
		}
		double dist = 0;
		for (double t = 0; t < path.size(); t += dT) {
			dist += Math.sqrt(path.get((int)t).dx(t%1) * path.get((int)t).dx(t%1) + path.get((int)t).dy(t%1) * path.get((int)t).dy(t%1)) * dT;
		}
		distance = dist;
		return dist;
	}

}
