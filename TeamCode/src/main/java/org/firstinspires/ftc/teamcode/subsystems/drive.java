package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import kotlin.annotation.MustBeDocumented;


public class drive implements Subsystem {
        public static final drive INSTANCE = new drive();

        private drive() {}

        @Retention(RetentionPolicy.RUNTIME)
        @Target(ElementType.TYPE)
        @MustBeDocumented
        @Inherited
        public @interface Attach{}

        private Dependency<?> dependency =
                Subsystem.DEFAULT_DEPENDENCY
                        .and(new SingleAnnotation<>(Attach.class));

        @NonNull
        @Override
        public Dependency<?> getDependency() {
            return dependency;
        }

        @Override
        public void setDependency(@NonNull Dependency<?> dependency) {
            this.dependency = dependency;
        }

        private final SubsystemObjectCell<DcMotorEx> motor = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, ""));
        public static DcMotorEx getMotor() {
            return INSTANCE.motor.get();
        }

        @Override
        public void preUserInitHook(@NonNull Wrapper opMode) {
            setDefaultCommand(simpleCommand());
        }
        @Override
        public void postUserInitHook(@NonNull Wrapper opMode) {

        }
        @Override
        public void preUserInitLoopHook(@NonNull Wrapper opMode) {}
        @Override
        public void preUserLoopHook(@NonNull Wrapper opMode) {}
        @Override
        public void postUserInitLoopHook(@NonNull Wrapper opMode) {}
        @Override
        public void postUserLoopHook(@NonNull Wrapper opMode) {}
        @Override
        public void preUserStopHook(@NonNull Wrapper opMode) {}
        @Override
        public void postUserStopHook(@NonNull Wrapper opMode) {}
        @Override
        public void cleanup(@NonNull Wrapper opMode) {}

        @NonNull
        public static Lambda simpleCommand() {
            return new Lambda("simple")
                    .addRequirements(INSTANCE)
                    .setInit(() -> getMotor().setPower(0.4))
                    .setEnd(interrupted -> {
                        if (!interrupted) getMotor().setPower(0.0);
                    });
        }
    }