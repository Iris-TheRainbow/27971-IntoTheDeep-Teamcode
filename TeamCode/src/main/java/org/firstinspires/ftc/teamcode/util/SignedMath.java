package org.firstinspires.ftc.teamcode.util;

import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

public class SignedMath {
    public static double signedSqrt(double input){
        return Math.signum(input) * Math.sqrt(Math.abs(input));
    }
    public static double signedSquared(double input){
        return Math.signum(input) * (input * input);
    }
    public static double distance(double x1, double y1, double x2, double y2){
        return (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
    }
    public static double signedFunc(UnaryOperator<Double> func, double value){
        return Math.signum(value) * func.apply(Math.abs(value));
    }

}
