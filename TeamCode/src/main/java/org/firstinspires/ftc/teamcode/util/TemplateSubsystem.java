package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class
TemplateSubsystem implements Subsystem {
    public static final TemplateSubsystem INSTANCE = new TemplateSubsystem();

    private TemplateSubsystem() { }

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
    public void preUserInitHook(@NonNull Wrapper opMode) { setDefaultCommand(command()); }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
    }

    public static void function() {

    }

    @NonNull
    public static Lambda command() {
        return new Lambda("simple")
                .addRequirements(INSTANCE)
                .setExecute(TemplateSubsystem::function);
    }
}