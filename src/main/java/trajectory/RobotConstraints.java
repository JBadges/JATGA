package trajectory;

public class RobotConstraints {

  public final double maxVelocity;
  public final double maxAngular;
  public final double maxAcceleration;
  public final double wheelbase;

  public RobotConstraints(double maxVelocity, double maxAngular, double maxAcceleration, double wheelbase) {
    this.maxVelocity = maxVelocity;
    this.maxAngular = maxAngular;
    this.maxAcceleration = maxAcceleration;
    this.wheelbase = wheelbase;
  }

}
