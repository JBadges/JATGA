package trajectory;

import java.util.TreeMap;

import math.Line;

/**
 * InterpolatableTreeMap
 */
public class Trajectory extends TreeMap<Double, DriveState> {

    public Trajectory() {
        super();
    }
    
    public DriveState put(Double d, DriveState s) {
        return super.put(d, s);
    }

    public DriveState getInterpolated(Double time) {
        if(time == 0.0) {
            return new DriveState(new State(0,0,0,0), new State(0, 0, 0, 0), 0);
        }
        DriveState gotval = get(time);
        if (gotval == null) {
            /** Get surrounding keys for interpolation */
            Double topBound = ceilingKey(time);
            Double bottomBound = floorKey(time);

            /**
             * If attempting interpolation at ends of tree, return the nearest data point
             */
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            /** Get surrounding values for interpolation */
            DriveState topElem = get(topBound);
            DriveState bottomElem = get(bottomBound);

            double leftAdjPosition = new Line(bottomBound, bottomElem.getLeftDrive().getPosition(), topBound, topElem.getLeftDrive().getPosition()).get(time);
            double leftAdjVel = new Line(bottomBound, bottomElem.getLeftDrive().getVelocity(), topBound, topElem.getLeftDrive().getVelocity()).get(time);
            double leftAdjAcc = new Line(bottomBound, bottomElem.getLeftDrive().getAcceleration(), topBound, topElem.getLeftDrive().getAcceleration()).get(time);
            State leftState = new State(time, leftAdjPosition, leftAdjVel, leftAdjAcc);

            double rightAdjPosition = new Line(bottomBound, bottomElem.getRightDrive().getPosition(), topBound, topElem.getRightDrive().getPosition()).get(time);
            double rightAdjVel = new Line(bottomBound, bottomElem.getRightDrive().getVelocity(), topBound, topElem.getRightDrive().getVelocity()).get(time);
            double rightAdjAcc = new Line(bottomBound, bottomElem.getRightDrive().getAcceleration(), topBound, topElem.getRightDrive().getAcceleration()).get(time);
            State rightState = new State(time, rightAdjPosition, rightAdjVel, rightAdjAcc);

            double adjHeading = new Line(bottomBound, bottomElem.getHeading(), topBound, topElem.getHeading()).get(time);

            return new DriveState(leftState, rightState, adjHeading);
        } else {
            return gotval;
        }
    }
    

}