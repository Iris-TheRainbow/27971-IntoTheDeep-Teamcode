package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class IntakeWrist implements Subsystem {
    private static Servo leftWrist, rightWrist;
    public static final IntakeWrist INSTANCE = new IntakeWrist();
    private IntakeWrist() { }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        leftWrist = new CachingServo(hwmap.get(Servo.class, "intakeWristLeft"));
        rightWrist = new CachingServo(hwmap.get(Servo.class, "intakeWristRight"));
        rightWrist.setDirection(Servo.Direction.REVERSE);
    }
    public static void setWristPosition(double position) {
        leftWrist.setPosition(position);
        rightWrist.setPosition(position);
    }
    @NonNull
    public static Command wristIntake() {
        return new Sequential(new Lambda("wrist intake")
                .addRequirements(INSTANCE)
                .setInit(() -> setWristPosition(.9)),
                new Wait(.125));
    }
    @NonNull
    public static Command wristVision() {
        return new Sequential(new Lambda("wrist vision")
                .addRequirements(INSTANCE)
                .setInit(() -> setWristPosition(.6)),
                new Wait(.150));

    }
    @NonNull
    public static Command wristGrab() {
        return new Sequential(new Lambda("wrist grab")
                .addRequirements(INSTANCE)
                .setInit(() -> setWristPosition(.9)),
                new Wait(.100));
    }

    @NonNull
    public static Command wristExtend() {
        return new Sequential(new Lambda("wrist extend")
                .addRequirements(INSTANCE)
                .setInit(() -> setWristPosition(.75)),
                new Wait(.175));
    }

    @NonNull
    public static Command wristTransfer() {
        return new Sequential(new Lambda("wrist transfer")
                .addRequirements(INSTANCE)
                .setInit(() -> setWristPosition(.24)),
                new Wait(.350));
    }
    public enum States{
        TRANSFER,
        EXTEND,
        GRAB,
        VISION,
        INTAKE
    }
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}