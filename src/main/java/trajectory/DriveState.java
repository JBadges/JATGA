package trajectory;

/**
 * DriveState
 */
public class DriveState {

    State leftDrive;
    State rightDrive;
    double heading;
    

    public DriveState(State leftDrive, State rightDrive, double heading) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.heading = heading;
    }

    public State getLeftDrive() {
        return this.leftDrive;
    }

    public void setLeftDrive(State leftDrive) {
        this.leftDrive = leftDrive;
    }

    public State getRightDrive() {
        return this.rightDrive;
    }

    public void setRightDrive(State rightDrive) {
        this.rightDrive = rightDrive;
    }

    public double getHeading() {
        return this.heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    @Override
    protected DriveState clone() {
        return new DriveState(leftDrive.clone(), rightDrive.clone(), heading);
    }

}