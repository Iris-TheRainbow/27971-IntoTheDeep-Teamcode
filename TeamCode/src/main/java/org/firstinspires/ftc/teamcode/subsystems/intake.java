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

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        clawServo = hwmap.get(Servo.class, "intakeClaw");
        leftWrist = hwmap.get(Servo.class, "leftIntakeWrist");
        rightWrist = hwmap.get(Servo.class, "rightIntakeWrist");
        rotation = hwmap.get(Servo.class, "intakeRotate");
        clawServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){
        setDefaultCommand(wristRotate());
        clawServo.setPosition(0);
    }
    public static void setWristPosition(double position) {
        leftWrist.setPosition(position);
        rightWrist.setPosition(position);
    }
    private static void close(){
        clawServo.setPosition(0);
    }
    private static void open(){
        clawServo.setPosition(.2);
    }

    private static void setRotation(){
        rotation.setPosition(.5 + ((-Mercurial.gamepad1().leftTrigger().state() + Mercurial.gamepad1().rightTrigger().state()) * .5));
    }
    @NonNull
    public static Lambda wristRotate(){
        return new Lambda("rotate the wrist")
                .addRequirements(INSTANCE)
                .setExecute(intake::setRotation);
    }
    @NonNull
    public static Lambda transfer(){
        return new Lambda("home rotation")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    rotation.setPosition(.5);
                    close();
                    setWristPosition(1);
                    waiter.start(600);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda wristIntake() {
        return new Lambda("wrist flat")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.3);
                    waiter.start(300);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda wristTransfer() {
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setWristPosition(.6);
                    waiter.start(300);
                })
                .setFinish(() -> waiter.isDone());
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
}