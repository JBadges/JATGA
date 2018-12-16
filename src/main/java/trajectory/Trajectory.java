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
            
            double leftAdjVel = new Line(topBound, topElem.getLeftDrive().getVelocity(), bottomBound, bottomElem.getLeftDrive().getVelocity()).get(time);
            double leftAdjAcc = new Line(topBound, topElem.getLeftDrive().getAcceleration(), bottomBound, bottomElem.getLeftDrive().getAcceleration()).get(time);
            State leftState = new State(time, leftAdjVel, leftAdjAcc);

            double rightAdjVel = new Line(topBound, topElem.getRightDrive().getVelocity(), bottomBound, bottomElem.getRightDrive().getVelocity()).get(time);
            double rightAdjAcc = new Line(topBound, topElem.getRightDrive().getAcceleration(), bottomBound, bottomElem.getRightDrive().getAcceleration()).get(time);
            State rightState = new State(time, rightAdjVel, rightAdjAcc);

            double adjHeading = new Line(topBound, topElem.getHeading(), bottomBound, bottomElem.getHeading()).get(time);

            return new DriveState(leftState, rightState, adjHeading);
        } else {
            return gotval;
        }
    }
    

}