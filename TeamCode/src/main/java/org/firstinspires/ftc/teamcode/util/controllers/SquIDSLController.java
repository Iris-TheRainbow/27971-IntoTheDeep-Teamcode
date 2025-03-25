package org.firstinspires.ftc.teamcode.util.controllers;


import org.firstinspires.ftc.teamcode.util.MathLibKt;

public class SquIDSLController {
    private double kP, kD , kS, minkL, maxkL;
    private int maxkLPose;
    private double lastTime;
    private final int lastError = 0;
    public SquIDSLController(double kP, double kD, double kS, double minkL, double maxkL, int maxkLPose){
        setConstants(kP, kD, kS, minkL, maxkL, maxkLPose);
    }
    public SquIDSLController(double kP, double kD, double kS){
         this(kP, kD, kS, 0, 0, 0);
    }
    private double calckL(int pose){
        return (maxkL - minkL) * ((double) pose / maxkLPose) + minkL;
    }
    public double calculate(int actual, int target){
        int error = target - actual;
        double time = System.nanoTime() * 1e-9;
        double errorVelocity = (error - lastError) / (time - lastTime);
        lastTime = time;
        double L = calckL(actual);
        if (Double.isNaN(L)){
            L = 0;
        }
        return MathLibKt.signedFunc(Math::sqrt, kP * error) + kD * errorVelocity + kS + L;
    }
    public void setConstants(double kP, double kD, double kS, double minkL, double maxkL, int maxkLPose){
        this.kP = kP;
        this.kD = kD;
        this.kS = kS;
        this.minkL = minkL;
        this.maxkL = maxkL;
        this.maxkLPose = maxkLPose;
    }
    public void setConstants(double kP, double kD, double kS){
        setConstants(kP, kD, kS, 0, 0, 0);

    }

}
