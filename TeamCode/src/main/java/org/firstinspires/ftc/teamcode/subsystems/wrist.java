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
import kotlin.internal.InlineOnly;

public class wrist implements Subsystem {
    public static final wrist INSTANCE = new wrist();
    public static Servo leftWrist, rightWrist;

    private wrist() { }

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
        leftWrist = hwmap.get(Servo.class, "wristLeft");
        rightWrist = hwmap.get(Servo.class, "wristRight");
        leftWrist.setDirection(Servo.Direction.REVERSE);
        setPosition(.6);
    }

    public static void setPosition(double position) {
        leftWrist.setPosition(position);
        rightWrist.setPosition(position);
    }

    @NonNull
    public static Lambda wristFlat(){
        return new Lambda("wrist flat")
                .addRequirements(INSTANCE)
                .setExecute(() -> setPosition(.3));
    }

    @NonNull
    public static Lambda wristDown(){
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setExecute(() -> setPosition(.6));
    }
}