package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class arm implements Subsystem {
    public static final arm INSTANCE = new arm();
    public static Servo leftWrist, rightWrist;

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
        leftWrist = hwmap.get(Servo.class, "leftWrist");
        rightWrist = hwmap.get(Servo.class, "rightWrist");
        rightWrist.setDirection(Servo.Direction.REVERSE);
    }

    public static void flat() {
        leftWrist.setPosition(.3);
        rightWrist.setPosition(.3);
    }

    public static void down(){
        leftWrist.setPosition(.6);
        rightWrist.setPosition(.6);
    }


    @NonNull
    public static Lambda command() {
        return new Lambda("simple")
                .addRequirements(INSTANCE)
                .setExecute(arm::function);
    }
}