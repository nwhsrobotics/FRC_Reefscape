package frc.robot.util;

public class SwerveUtils {

    /**
     * Steps a value towards a target with a specified step size.
     *
     * @param _current  The current or starting value.  Can be positive or negative.
     * @param _target   The target value the algorithm will step towards.  Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        } else if (_target < _current) {
            return _current - _stepsize;
        } else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     *
     * @param _current  The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target   The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);

        if (difference <= _stepsize) {
            return _target;
        } else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2 * Math.PI - _target < _stepsize || _target + 2 * Math.PI - _current < _stepsize) {
                return _target;
            } else {
                return WrapAngle(_current - stepDirection * _stepsize); //this will handle wrapping gracefully
            }

        } else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     *
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI ? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     *
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2 * Math.PI;

        if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        } else if (_angle > twoPi) {
            return _angle - twoPi * Math.floor(_angle / twoPi);
        } else if (_angle < 0.0) {
            return _angle + twoPi * (Math.floor((-_angle) / twoPi) + 1);
        } else {
            return _angle;
        }
    }


    /**
     * Returns a signed difference between 2 angles (in radians).
     * The result is in (-pi, pi].
     *
     * If result is positive, 'angleA' is ahead of 'angleB' in CCW sense.
     * If result is negative, 'angleA' is behind 'angleB' in CCW sense.
     */
    public static double angleDifferenceSigned(double angleA, double angleB) {
        // Wrap angles so they lie within [-pi, pi)
        double a = wrapToPi(angleA);
        double b = wrapToPi(angleB);

        double diff = a - b; // raw difference
        // Now ensure diff is in (-pi, +pi]
        if (diff > Math.PI) {
            diff -= 2.0 * Math.PI;
        } else if (diff <= -Math.PI) {
            diff += 2.0 * Math.PI;
        }
        return diff;
    }

    /**
     * Wrap an angle (in radians) to the range [-pi, pi).
     */
    public static double wrapToPi(double angle) {
        // shift everything by +pi, mod 2π, shift back
        // or handle directly:
        double wrapped = ((angle + Math.PI) % (2.0 * Math.PI)) - Math.PI;
        // in Java, % can yield negative for negative inputs, so we might do:
        // wrapped = wrapped < -Math.PI ? wrapped + 2.0*Math.PI : wrapped;
        // or something to force into [-pi, pi)
        if (wrapped <= -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

}