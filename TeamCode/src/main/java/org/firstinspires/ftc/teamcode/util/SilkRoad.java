package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;

public class SilkRoad implements Feature {
    private Dependency<?> dependency = new SingleAnnotation<>(org.firstinspires.ftc.teamcode.util.SilkRoad.Attach.class);
    private static FtcDashboard dash;
    private static Canvas canvas;
    private static Action actions;
    private static boolean run;
    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private SilkRoad() {}

    public static final org.firstinspires.ftc.teamcode.util.SilkRoad INSTANCE = new org.firstinspires.ftc.teamcode.util.SilkRoad();

    @Override
    public void postUserInitHook(@NotNull Wrapper opMode) {
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        run = true;
    }

    @Override
    public void postUserLoopHook(@NotNull Wrapper opMode) {
        if (run && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            run = actions.run(packet);

            dash.sendTelemetryPacket(packet);
        }
    }

    public static void RunAsync(Action actions){
        SilkRoad.actions = actions;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {}
}