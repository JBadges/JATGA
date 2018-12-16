package trajectory;

/**
 * State
 */
public class State {

    private double time;
    private double position;
    private double velocity;
    private double acceleration;

    public State(double time, double position, double velocity, double acceleration) {
        this.time = time;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
    
    public double getTime() {
        return this.time;
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
        return new State(time, position, velocity, acceleration);
    }

    @Override
    public String toString() {
        return time + ", " + position + ", " + velocity + ", " + acceleration + ", "; 
    }

}