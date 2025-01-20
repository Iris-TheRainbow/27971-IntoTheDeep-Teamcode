package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Waiter;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class deposit implements Subsystem {
    public static final deposit INSTANCE = new deposit();
    public static Servo leftWrist, rightWrist, clawServo;
    private static Waiter waiter;

    private deposit() {
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private static StateMachine<ClawState> clawStates = new StateMachine<>(ClawState.CLOSED)
            .withState(ClawState.CLOSED, (stateRef, name) -> closeClaw())
            .withState(ClawState.OPEN, (stateRef, name) -> openClaw());

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        leftWrist = new CachingServo(hwmap.get(Servo.class, "depositWristLeft"));
        rightWrist = new CachingServo(hwmap.get(Servo.class, "depositWristRight"));
        clawServo = hwmap.get(Servo.class, "depositClaw");
        clawServo.setDirection(Servo.Direction.REVERSE);
        rightWrist.setDirection(Servo.Direction.REVERSE);
        waiter = new Waiter();
    }
    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){ setPosition(.85); }

    public static void setPosition(double position) {
        leftWrist.setPosition(position);
        rightWrist.setPosition(position);
    }

    private static void close(){ clawServo.setPosition(.6); clawStates.setState(ClawState.CLOSED);}
    private static void open(){ clawServo.setPosition(.3); clawStates.setState(ClawState.OPEN);}

    @NonNull
    public static Lambda wristDeposit() {
        return new Lambda("wrist flat")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setPosition(.6);
                    waiter.start(300);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda toggleClaw(){  
        return new Lambda("claw toggle")
                .setInit(() -> {
                    switch (clawStates.getState()){
                        case OPEN:
                            clawStates.schedule(ClawState.CLOSED);
                            break;
                        case CLOSED:
                            clawStates.schedule(ClawState.OPEN);
                            break;
                    }
                });
    }

    @NonNull
    public static Lambda wristTransfer() {
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setPosition(.02);
                    waiter.start(300);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda closeClaw(){
        return new Lambda("Close Claw")
                .addRequirements(INSTANCE)
                .setInterruptible(false)
                .setInit(() -> {
                    close();
                    waiter.start(200);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda openClaw(){
        return new Lambda("open Claw")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    open();
                    waiter.start(200);
                })
                .setFinish(() -> waiter.isDone());
    }
    private enum ClawState{
        OPEN,
        CLOSED
    }
}