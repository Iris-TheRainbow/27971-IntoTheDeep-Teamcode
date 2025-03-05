package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Waiter;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;
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
        clawServo = new CachingServo(hwmap.get(Servo.class, "intakeClaw"));
        leftWrist = new CachingServo(hwmap.get(Servo.class, "intakeWristLeft"));
        rightWrist = new CachingServo(hwmap.get(Servo.class, "intakeWristRight"));
        rightWrist.setDirection(Servo.Direction.REVERSE);
        rotation = hwmap.get(Servo.class, "intakeRotate");
        rotation.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.REVERSE);
        waiter = new Waiter();
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode){
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
    private static void close(){ clawServo.setPosition(.02); clawStates.setState(ClawState.CLOSED);
    }
    private static void open(){
        clawServo.setPosition(.14); clawStates.setState(ClawState.OPEN); }

    private static void setRotation(){
        rotation.setPosition(.5 + ((-Mercurial.gamepad1().leftTrigger().state() * .8 + Mercurial.gamepad1().rightTrigger().state() * .83) * .5));
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
    public static Lambda wristRotateSupplier(DoubleSupplier pos){
        return new Lambda("rotate the wrist")
                .setRequirements(INSTANCE)
                .setExecute(() -> { setWristRotation(pos.getAsDouble());})
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
                    setWristPosition(.9);
                    waiter.start(150);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda wristVision() {
        return new Lambda("wrist vision")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.6);
                    waiter.start(150);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda wristGrab() {
        return new Lambda("wrist flat")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.9);
                    waiter.start(100);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda wristExtend() {
        return new Lambda("wrist flat")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.65);
                    waiter.start(175);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda wristTransfer() {
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.24);
                    waiter.start(350);
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
                })
                .setFinish(() -> true);
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