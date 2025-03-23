package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathing.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.util.Evaluation;
import org.firstinspires.ftc.teamcode.util.LazyPose2d;
import org.firstinspires.ftc.teamcode.util.PidToPointBuilderKt;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.concurrent.Callable;

import java.util.concurrent.Future;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
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
    private static SparkFunOTOSDrive RRDrive;
    private static Canvas canvas;
    private static final ArrayList<Action> actions = new ArrayList<Action>();
    private static HardwareMap hwmap;


    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        hwmap = opMode.getOpMode().hardwareMap;
        RRDrive = GenerateRRDrive(hwmap);
        telem = opMode.getOpMode().telemetry;
        setDefaultCommand(PIDToLast());
        canvas = new Canvas();
    }

    @Override
    public void cleanup(@NonNull Wrapper opMode) {
        canvas = null;
        actions.clear();
    }
    public static Pose2d getPose(){
        return RRDrive.pose;
    }
    public static void runAsync(Action action){
        actions.add(action);
    }

    public static void setInitialPose(Pose2d initialPose){
        Wavedash.initialPose = initialPose;
        RRDrive = GenerateRRDrive(hwmap);
        RRDrive.pose = initialPose;
    }
    public static void setInitialPose(HardwareMap hwmap, Pose2d initialPose){
        Wavedash.initialPose = initialPose;
        RRDrive = GenerateRRDrive(hwmap);
        RRDrive.pose = initialPose;
    }

    private static SparkFunOTOSDrive GenerateRRDrive(HardwareMap hwmap){
        return new SparkFunOTOSDrive(hwmap, Wavedash.initialPose);
    }

    public static TrajectoryActionBuilder actionBuilder(Pose2d initialPose) {
        Wavedash.initialPose = initialPose;
        RRDrive = GenerateRRDrive(hwmap);
        return RRDrive.actionBuilder(initialPose);
    }

    @NonNull
    public static Lambda PIDToLast() {
        if(RRDrive.getLastPose() != null){
            return PIDToPoint(
                    () -> RRDrive.getLastPose().position.x,
                    () -> RRDrive.getLastPose().position.y,
                    () -> RRDrive.getLastPose().heading.toDouble(), .5, 1,
                    Evaluation.onInit
            );
        }
        else return new Lambda("null");
    }

    @NonNull
    public static Lambda PIDToPoint(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h, double translationalAccuracy, double headingAccuracy, Evaluation evaluate) {
        LazyPose2d pose = new LazyPose2d(x, y, h);
        return new Lambda("PID to last set target")
                .addRequirements(INSTANCE)
                .setInit(pose::evaluate)
                .setExecute(() -> {
                    Pose2dDual<Time> target = Pose2dDual.constant(pose.value(), 3);
                    RRDrive.setLastPose(pose.value());
                    double translationalError = target.value().minusExp(RRDrive.pose).position.norm();
                    double headingError = Math.abs(Math.toDegrees(target.value().minusExp(RRDrive.pose).heading.toDouble()));
                    if (!(translationalError < .5 && headingError <  1)) {
                        RRDrive.goToTarget(target, canvas);
                    } else {
                        RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    }
                })
                .setFinish(() -> {
                    Pose2dDual<Time> target = Pose2dDual.constant(pose.value(), 3);
                    double translationalError = target.value().minusExp(RRDrive.pose).position.norm();
                    double headingError = Math.abs(Math.toDegrees(target.value().minusExp(RRDrive.pose).heading.toDouble()));
                    return  translationalError < translationalAccuracy && headingError <  headingAccuracy;
                })
                .setEnd((interupted) -> RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0)));
    }


    public static PidToPointBuilderKt p2pBuilder(Pose2d pose){
        setInitialPose(pose);
        return new PidToPointBuilderKt(pose, Wavedash::PIDToPoint, Wavedash::getPose);
    }
    public static PidToPointBuilderKt p2pBuilder(HardwareMap hwmap, Pose2d pose){
        setInitialPose(hwmap, pose);
        return new PidToPointBuilderKt(pose, Wavedash::PIDToPoint, Wavedash::getPose);
    }

}