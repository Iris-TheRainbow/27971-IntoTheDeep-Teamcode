package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Iterator;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Wavedash implements Subsystem {
    public static final Wavedash INSTANCE = new Wavedash();
    private Wavedash() { }
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
    private static Pose2d initialPose = new Pose2d(0, 0, 0);
    private static MecanumDrive RRDrive;

    private static FtcDashboard dash;
    private static Canvas canvas;
    private static final ArrayList<Action> actions = new ArrayList<Action>();
    private static HardwareMap hwmap;

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        hwmap = opMode.getOpMode().hardwareMap;
        MecanumDrive RRDrive = GenerateRRDrive(hwmap);
        if (opMode.getOpModeType() == OpModeMeta.Flavor.AUTONOMOUS){
            setDefaultCommand(PIDToLast());
        }
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(canvas.getOperations());
        Iterator<Action> iter = actions.iterator();
        while (iter.hasNext() && !Thread.currentThread().isInterrupted()) {
            Action action = iter.next();
            if (!action.run(packet)) iter.remove();
        }
        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void cleanup(@NonNull Wrapper opMode) {
        dash = null;
        canvas = null;
        actions.clear();
    }

    public static void runAsync(Action action){
        actions.add(action);
    }

    public static void setInitialPose(Pose2d initialPose){
        Wavedash.initialPose = initialPose;
    }

    private static MecanumDrive GenerateRRDrive(HardwareMap hwmap){
        return new MecanumDrive(hwmap, Wavedash.initialPose);
    }

    public static TrajectoryActionBuilder actionBuilder(Pose2d initialPose){
        return RRDrive.actionBuilder(initialPose);
    }

    @NonNull
    public static Lambda PIDToLast() {
        return new Lambda("PID to last set target")
                .addRequirements(INSTANCE)
                .setExecute(() -> {
                    if (RRDrive.getLastTarget() != null){
                        Pose2dDual<Time> target = RRDrive.getLastTarget();
                        TelemetryPacket packet = new TelemetryPacket();
                        RRDrive.goToTarget(target, packet.fieldOverlay());
                        dash.sendTelemetryPacket(packet);
                    }
                });
    }

    @NonNull
    public static Lambda DriveBusy(){
        return new Lambda("RR is doing stuff")
                .addRequirements(INSTANCE)
                .setFinish(() -> RRDrive.isDriveDone());
    }

}