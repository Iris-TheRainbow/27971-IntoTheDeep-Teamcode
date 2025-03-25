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

public class DepositWrist implements Subsystem {
    public static final DepositWrist INSTANCE = new DepositWrist();
    public static Servo leftWrist, rightWrist;
    private DepositWrist() {
    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        leftWrist = new CachingServo(hwmap.get(Servo.class, "depositWristLeft"));
        rightWrist = new CachingServo(hwmap.get(Servo.class, "depositWristRight"));

        rightWrist.setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){ setPosition(.85); }

    public static void setPosition(double position) {
        leftWrist.setPosition(position);
        rightWrist.setPosition(position);
    }

    @NonNull
    public static Command wristDeposit() {
        return new Sequential(new Lambda("wrist deposit")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.7)),
                new Wait(.175));
    }
    @NonNull
    public static Command wristWait() {
        return new Sequential(new Lambda("wrist wait")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.2)),
                new Wait(.175));
    }
    @NonNull
    public static Command wristTransfer() {
        return new Sequential(new Lambda("wrist transfer")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.10)),
                new Wait(.100));
    }
    @NonNull
    public static Command wristSepc() {
        return new Sequential(new Lambda("wrist spec")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.8)),
                new Wait(.300));
    }
    public enum States {
        SPEC,
        TRANSFER,
        WAIT,
        DEPOSIT
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
}