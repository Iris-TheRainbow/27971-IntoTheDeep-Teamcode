package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.StateMachine;
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
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class arm implements Subsystem {
    public static final arm INSTANCE = new arm();
    private static Servo leftArm, rightArm;
    private static Waiter waiter;

    private arm() { }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }



    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        leftArm = new CachingServo(hwmap.get(Servo.class, "armLeft"));
        rightArm = new CachingServo(hwmap.get(Servo.class, "armRight"));
        leftArm.setDirection(Servo.Direction.REVERSE);
        waiter = new Waiter();
    }

    public static void setPosition(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    @NonNull
    public static Lambda armTransfer() {
        return new Lambda("arm stow")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setPosition(.08);
                    waiter.start(75);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda armWait() {
        return new Lambda("arm stow")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setPosition(.2);
                    waiter.start(200);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda armUp(){
        return new Lambda("arm up")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setPosition(.43);
                    waiter.start(200);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda armExtend(){
        return new Lambda("arm up")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setPosition(.3);
                    waiter.start(200);
                })
                .setFinish(() -> waiter.isDone());
    }
}