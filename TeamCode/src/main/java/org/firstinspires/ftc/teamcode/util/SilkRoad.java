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
    private Dependency<?> dependency = new SingleAnnotation<>(SilkRoad.Attach.class);
    private static FtcDashboard dash;
    private static Canvas canvas;
    private static Action actions;
    private static boolean run;
    private static boolean auto;

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private SilkRoad() {}

    public static final SilkRoad INSTANCE = new SilkRoad();

    @Override
    public void postUserInitHook(@NotNull Wrapper opMode) {
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        run = true;
        auto = true;
    }

    @Override
    public void postUserLoopHook(@NotNull Wrapper opMode) {
        if (auto) {
            SilkRoad.update();
        }
    }

    public static void manualMode(){ SilkRoad.auto = false;}
    public static void update(){
        if (run && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            run = actions.run(packet);

            dash.sendTelemetryPacket(packet);
        }
    }
    public static void RunAsync(Action actions){ SilkRoad.actions = actions; }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {}
}