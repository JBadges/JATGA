package math;

/**
 * Line
 */
public class Line {

    private double m;
    private double b;

    public Line(double x1, double y1, double x2, double y2) {
        this.m = (y1-y2)/(x1-x2);
        this.b = y1-m*x1;
    }

    public double get(double x) {
        return m*x + b;
    }
}