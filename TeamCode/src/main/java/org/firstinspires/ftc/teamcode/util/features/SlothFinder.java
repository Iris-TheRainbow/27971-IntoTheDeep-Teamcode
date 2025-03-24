package org.firstinspires.ftc.teamcode.util.features;

import androidx.annotation.NonNull;

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
//import dev.frozenmilk.sinister.loaders.SlothClassLoader;

public class SlothFinder implements Feature {
    private String loadStatus = "Normal load";
    private boolean isSloth = false;
    @Override
    public void preUserInitHook(@NonNull Wrapper opMode){
        if (true/*getClass().getClassLoader() instanceof SlothClassLoader*/){
            loadStatus = "SLOTHLOADED";
            isSloth = true;
        }
    }
    @Override
    public void preUserInitLoopHook(Wrapper opMode) {
        opMode.getOpMode().telemetry.addLine(loadStatus);
    }

    @Override
    public void preUserLoopHook(Wrapper opMode) {
        opMode.getOpMode().telemetry.addLine(loadStatus);
    }

    public boolean isSlothLoad(){
        return isSloth;
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