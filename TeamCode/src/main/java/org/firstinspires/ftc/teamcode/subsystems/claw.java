package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

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

public class claw implements Subsystem {
    private static Servo clawServo;
    public static final claw INSTANCE = new claw();
    private static boolean state;

    private claw() { }

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
        clawServo = hwmap.get(Servo.class, "claw");
        clawServo.setDirection(Servo.Direction.REVERSE);
        clawServo.setPosition(0);
    }
    private static void close(){
        clawServo.setPosition(0);
    }
    private static void open(){
        clawServo.setPosition(.2);
    }

    @NonNull
    public static Lambda clawToggle() {
        return new Lambda("toggle Claw")
                .addRequirements(INSTANCE)
                .setExecute(claw::close)
                .setEnd((interrupted) -> open());
    }

    @NonNull
    public static Lambda closeClaw(){
        return new Lambda("Close Claw")
                .addRequirements(INSTANCE)
                .setExecute(claw::close);
    }

    @NonNull
    public static Lambda openClaw(){
        return new Lambda("Close Claw")
                .addRequirements(INSTANCE)
                .setExecute(claw::open);
    }
}