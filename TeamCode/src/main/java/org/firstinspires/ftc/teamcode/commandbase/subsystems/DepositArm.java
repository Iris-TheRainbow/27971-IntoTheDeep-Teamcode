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

public class DepositArm implements Subsystem {
    public static final DepositArm INSTANCE = new DepositArm();
    private static Servo leftArm, rightArm;

    private DepositArm() { }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        leftArm = new CachingServo(hwmap.get(Servo.class, "armLeft"));
        rightArm = new CachingServo(hwmap.get(Servo.class, "armRight"));
        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    public static void setPosition(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    @NonNull
    public static Command armTransfer() {
        return new Sequential(new Lambda("wrist wait")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.1)),
                new Wait(.075));
    }
    @NonNull
    public static Command armWait() {
        return new Sequential(new Lambda("wrist wait")
                        .addRequirements(INSTANCE)
                        .setInit(() -> setPosition(.2)),
                        new Wait(.200));
    }

    @NonNull
    public static Command armUp(){
        return new Sequential(new Lambda("wrist wait")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.45)),
                new Wait(.200));
    }

    @NonNull
    public static Command armExtend(){
        return new Sequential(new Lambda("wrist wait")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.3)),
                new Wait(.200));

    }
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}