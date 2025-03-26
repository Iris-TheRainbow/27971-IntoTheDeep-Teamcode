package org.firstinspires.ftc.teamcode.util.features;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.List;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.VoidDependency;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;

public class BulkRead implements Feature {
    private BulkRead() {}

    public static final BulkRead INSTANCE = new BulkRead();

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        RobotLog.vv("BulkRead", "Enabling bulk reads for " + opMode.getName());
        for (LynxModule hub : opMode.getOpMode().hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void preUserInitLoopHook(Wrapper opMode) {
        for (LynxModule hub : opMode.getOpMode().hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void preUserLoopHook(Wrapper opMode) {
        for (LynxModule hub : opMode.getOpMode().hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
    }

    private Dependency<?> dependency = (VoidDependency) (Wrapper opMode, List<? extends Feature> resolvedFeatures, boolean yielding) -> {};

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }


    @Target(ElementType.TYPE)
    @Retention(RetentionPolicy.RUNTIME)
    @Documented
    @Inherited
    public @interface Attach {}
}