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

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class intake implements Subsystem {
    private static Servo clawServo, leftWrist, rightWrist, rotation;
    public static final intake INSTANCE = new intake();
    private static Waiter waiter;


    private intake() { }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
    private static StateMachine<ClawState> clawStates = new StateMachine<>(ClawState.CLOSED)
            .withState(ClawState.CLOSED, (stateRef, name) -> closeClaw())
            .withState(ClawState.OPEN, (stateRef, name) -> openClaw());

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        clawServo = hwmap.get(Servo.class, "intakeClaw");
        leftWrist = hwmap.get(Servo.class, "intakeWristLeft");
        rightWrist = hwmap.get(Servo.class, "intakeWristRight");
        leftWrist.setDirection(Servo.Direction.REVERSE);
        rotation = hwmap.get(Servo.class, "intakeRotate");
        rotation.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.REVERSE);
        waiter = new Waiter();
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode){
        opMode.getOpMode().telemetry.addData("LeftServo", leftWrist.getPosition());
        opMode.getOpMode().telemetry.addData("RightWrist", rightWrist.getPosition());
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){
        setDefaultCommand(autoWrist());
        clawServo.setPosition(0);
    }
    public static void setWristPosition(double position) {
        leftWrist.setPosition(position);
        rightWrist.setPosition(position);
    }
    private static void close(){ clawServo.setPosition(.1); clawStates.setState(ClawState.CLOSED);
    }
    private static void open(){
        clawServo.setPosition(.4); clawStates.setState(ClawState.OPEN); }

    private static void setRotation(){
        rotation.setPosition(.5 + ((-Mercurial.gamepad1().leftTrigger().state() + Mercurial.gamepad1().rightTrigger().state()) * .5));
    }
    private static void setWristRotation(double pos){
        rotation.setPosition(pos);
    }
    @NonNull
    public static Lambda wristRotate(double pos){
        return new Lambda("rotate the wrist")
                .setRequirements(INSTANCE)
                .setExecute(() -> { setWristRotation(pos);})
                .setInterruptible(true);
    }
    @NonNull
    public static Lambda autoWrist(){
        return new Lambda("rotate the wrist")
                .setRequirements(INSTANCE)
                .setExecute(intake::setRotation)
                .setInterruptible(true);
    }

    @NonNull
    public static Lambda wristIntake() {
        return new Lambda("wrist flat")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.82);
                    waiter.start(300);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda wristTransfer() {
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.13);
                    waiter.start(800);
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
    public static Lambda closeClaw(){
        return new Lambda("Close Claw")
                .addRequirements(INSTANCE)
                .setInit(intake::close);
    }

    @NonNull
    public static Lambda openClaw(){
        return new Lambda("Close Claw")
                .addRequirements(INSTANCE)
                .setInit(intake::open);
    }

    private enum ClawState{
        OPEN,
        CLOSED
    }

}