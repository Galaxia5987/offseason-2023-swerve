package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;

import java.util.function.DoubleUnaryOperator;

public class Utils {


    /**
     * This method ensures that any accidental input is set to 0.
     * In other words, if the value is less than the threshold the function returns 0.
     *
     * @param val       the input value.
     * @param threshold the threshold value.
     * @return 0 if the value is less than the threshold else the value.
     */
    public static double conventionalDeadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return val;
    }

    /**
     * @param val       the input value.
     * @param threshold the threshold value.
     * @return the updated value after the deadband.
     */
    public static double continuousDeadband(double val, double threshold) {
        val = conventionalDeadband(val, threshold);
        return (val - (Math.signum(val) * threshold)) / (1 - threshold);
    }

    /**
     * This function converts rps into rpm.
     *
     * @param rps is the input velocity. [rps]
     * @return the same velocity. [rpm]
     */
    public static double rpsToRpm(double rps) {
        return rps * 60.0;
    }


    /**
     * This function converts rps into rpm.
     *
     * @param rpm is the input velocity. [rpm]
     * @return the same velocity. [rps]
     */
    public static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}