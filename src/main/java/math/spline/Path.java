package math.spline;

import java.util.List;

import math.Point;

public class Path {

	private List<QuinticHermiteSpline> path;
	private final double dI = 0.00001;

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
		double dist = 0;
		for (double i = 0; i + dI < path.size(); i += dI) {
			Point a = path.get((int) i).getPoint(i % 1);
			Point b = path.get((int) (i + dI)).getPoint((i + dI) % 1);
			dist += a.distance(b);
		}
		return dist;
	}

	public Point getPointAtDistance(double distance) {
		double i = 0;
		for (double dist = 0; i + dI < path.size() && dist < distance; i += dI) {
			Point a = path.get((int) i).getPoint(i % 1);
			Point b = path.get((int) (i + dI)).getPoint((i + dI) % 1);
			dist += a.distance(b);
		}
		return path.get((int) i).getPoint(i % 1);
	}
}
