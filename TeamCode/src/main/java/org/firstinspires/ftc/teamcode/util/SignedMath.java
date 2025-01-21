package org.firstinspires.ftc.teamcode.util;

public class SignedMath {
    public static double signedSqrt(double input){
        return Math.signum(input) * Math.sqrt(Math.abs(input));
    }
    public static double signedSquared(double input){
        return Math.signum(input) * (input * input);
    }
    //public static double signedFunc(function func, double input){
    //    return Math.signum(input) * func(input);
    //}
}
