package org.firstinspires.ftc.teamcode.util;

public class SignedSqrt {
    public static double signedSqrt(double input){
        return Math.signum(input) * Math.sqrt(Math.abs(input));
    }
}
