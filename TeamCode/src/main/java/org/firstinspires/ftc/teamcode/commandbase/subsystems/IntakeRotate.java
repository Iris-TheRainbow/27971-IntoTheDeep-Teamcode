package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class IntakeRotate implements Subsystem {
    private static Servo rotation;
    public static final IntakeRotate INSTANCE = new IntakeRotate();

    private IntakeRotate() { }


    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        rotation = hwmap.get(Servo.class, "intakeRotate");
        rotation.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){
        setDefaultCommand(autoRotate());
    }

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
    public static Lambda autoRotate(){
        return new Lambda("rotate the wrist")
                .setRequirements(INSTANCE)
                .setExecute(IntakeRotate::setRotation)
                .setInterruptible(true);
    }
    public enum States{
        HOME,
        AUTO,
        MANUAL
    }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}