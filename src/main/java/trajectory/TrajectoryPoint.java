package trajectory;

public class TrajectoryPoint {

	private double time;
	private double position;
	private double velocity;
	private double acceleration;
	private double heading;
	private double timeStep;
	
	public TrajectoryPoint(double time, double position, double velocity, double acceleration, double heading, double timeStep) {
		this.time = time;
		this.position = position;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.heading = heading;
		this.timeStep = timeStep;
	}

	public double getTime() {
		return time;
	}

	public void setTime(double time) {
		this.time = time;
	}

	public double getPosition() {
		return position;
	}

	public void setPosition(double position) {
		this.position = position;
	}

	public double getVelocity() {
		return velocity;
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	public double getAcceleration() {
		return acceleration;
	}

	public void setAcceleration(double acceleration) {
		this.acceleration = acceleration;
	}

	public double getHeading() {
		return heading;
	}

	public void setHeading(double heading) {
		this.heading = heading;
	}

	public double getTimeStep() {
		return timeStep;
	}

	public void setTimeStep(double timeStep) {
		this.timeStep = timeStep;
	}
	
	@Override
	public String toString() {
		return time + ", " + position + ", " + velocity + ", " + acceleration + ", " + heading + ", " + timeStep + ",\n";
	}
	
}
