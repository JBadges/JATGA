package trajectory;

/**
 * State
 */
public class State {

    private double time;
    private double velocity;
    private double acceleration;

    public State(double time, double velocity, double acceleration) {
        this.time = time;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
    
    public double getTime() {
        return this.time;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public double getVelocity() {
        return this.velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getAcceleration() {
        return this.acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    @Override
    protected State clone() {
        return new State(time, velocity, acceleration);
    }

}