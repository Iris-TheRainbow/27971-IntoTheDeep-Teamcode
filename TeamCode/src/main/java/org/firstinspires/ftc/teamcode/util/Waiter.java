package org.firstinspires.ftc.teamcode.util;

public class Waiter {
    private long startTime;
    private long waitMS;

    public void start(long waitMS){
        startTime = System.nanoTime() / 1000000;
        this.waitMS = waitMS;
    }
    public long waitedTime(){
        return (System.nanoTime() / 1000000) - startTime;
    }
    public boolean isDone(){ return ((System.nanoTime() / 1000000) > (startTime + waitMS )); }
}
