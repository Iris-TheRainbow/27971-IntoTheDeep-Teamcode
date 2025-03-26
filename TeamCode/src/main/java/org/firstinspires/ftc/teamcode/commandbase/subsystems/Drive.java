package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.pathing.Drawing;
import org.firstinspires.ftc.teamcode.util.OrderedPair;
import org.firstinspires.ftc.teamcode.util.controllers.HolonomicRobotCentricController;
import org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt;
import org.firstinspires.ftc.teamcode.pathing.Evaluation;
import org.firstinspires.ftc.teamcode.util.LazyPose2d;
import org.firstinspires.ftc.teamcode.util.MathLibKt;
import org.firstinspires.ftc.teamcode.pathing.PidToPointBuilderKt;
import org.firstinspires.ftc.teamcode.util.drivers.Pinpoint;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Drive implements Subsystem {
        public static final Drive INSTANCE = new Drive();
        private static DcMotorEx leftFront, leftBack, rightFront, rightBack;
        private static Pinpoint pinpoint;
        private static Pose2d pose;
        private static Pose2d lastTarget;

        private static HolonomicRobotCentricController controller;
        private static double turnNerf = 1;
        private static final LinkedList<Pose2d> poseHistory = new LinkedList<>();
        private Drive() { }

        @Override
        public void postUserInitHook(@NonNull Wrapper opMode) {
                HardwareMap hwmap = opMode.getOpMode().hardwareMap;
                leftBack = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "leftFront"));
                leftFront = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "rightFront"));
                rightBack = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "rightBack"));
                rightFront = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "leftBack"));
                pinpoint = hwmap.get(Pinpoint.class, "pinpoint");
                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                pinpoint = hwmap.get(Pinpoint.class, "pinpoint");
                pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(PathingConstantsKt.pinpointXDirection, PathingConstantsKt.pinpointYDirection);
                pinpoint.setOffsets(PathingConstantsKt.pinpointXOffsetMM, PathingConstantsKt.pinpointYOffsetMM, DistanceUnit.MM);
                pinpoint.resetPosAndIMU();
                if (pose == null){
                        pose = new Pose2d(0,0,0);
                }
                lastTarget = pose;
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble()));
                preUserLoopHook(opMode);
                if (opMode.getOpModeType() == OpModeMeta.Flavor.TELEOP) {
                        setDefaultCommand(driveCommand());
                }
                if (opMode.getOpModeType() == OpModeMeta.Flavor.AUTONOMOUS){
                        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        setDefaultCommand(PIDToLast());
                }
                controller = new HolonomicRobotCentricController(
                        PathingConstantsKt.axialGain, PathingConstantsKt.lateralGain, PathingConstantsKt.headingGain,
                        PathingConstantsKt.axialVelGain, PathingConstantsKt.lateralVelGain, PathingConstantsKt.headingVelGain
                );
        }
        @Override
        public void preUserLoopHook(@NonNull Wrapper opMode){
                pinpoint.update();
                pose = Pinpoint.pinpointToRRPose(pinpoint.getPosition());
                poseHistory.add(pose);
                Drawing.drawPoseHistory(Telem.packet.fieldOverlay(), poseHistory);
                drawRobot("#3F51B5", pose);
        }
        public static void driveUpdate(){
                double x = MathLibKt.signedFunc(MathLibKt::cube, Mercurial.gamepad1().leftStickX().state());
                double y = MathLibKt.signedFunc(MathLibKt::cube, Mercurial.gamepad1().leftStickY().state());
                double rot = MathLibKt.signedFunc(MathLibKt::cube, Mercurial.gamepad1().rightStickX().state()) * turnNerf;

                double newTargetX = pose.position.x;
                double newTargetY = pose.position.y;
                double newTargetHead = pose.heading.toDouble();

                PoseVelocity2d command = computeCommand(lastTarget);

//                if (Math.abs(Mercurial.gamepad1().leftStickX().state()) < .05 && Math.abs(Mercurial.gamepad1().leftStickY().state()) < .05){
//                        x = 0;
//                        y = 0;
//                        if (lastTarget.minusExp(Drive.pose).position.norm() > .5){
//                                x = command.linearVel.x;
//                                y = command.linearVel.y;
//                        }
//                        newTargetX = lastTarget.position.x;
//                        newTargetY = lastTarget.position.y;
//                }
                if (Math.abs(Mercurial.gamepad1().rightStickX().state()) < .05 ){
                        rot = 0;
                        if (Math.abs(Math.toDegrees(lastTarget.minusExp(Drive.pose).heading.toDouble())) > 1){
                                rot = command.angVel;
                        }
                        newTargetHead = lastTarget.heading.toDouble();
                }

                lastTarget = new Pose2d(newTargetX, newTargetY, newTargetHead);
                drawRobot("#4CAF50", lastTarget);
                setDrivePower(new Vector2d(x, y), rot);
        }
        public static void setDrivePower(Vector2d vector, Double rotate) {
                drawVector("#4CAF50", pose, vector);
                drawVector("#4CAF50", new Pose2d(pose.position.x + 15, pose.position.y, 0), new Vector2d(0, rotate*5));
                double heading = 0;
                // Do the kinematics math
                double rotX = (vector.x * cos(-heading) - vector.y * sin(-heading)) * 1.1;
                double rotY = vector.x * sin(-heading) + vector.y * cos(-heading);
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
                double lfPower = (rotY + rotX + rotate) / denominator;
                double lbPower = (rotY - rotX + rotate) / denominator;
                double rfPower = (rotY - rotX - rotate) / denominator;
                double rbPower = (rotY + rotX - rotate) / denominator;
                // Set the Motor Power
                leftBack.setPower(lbPower);
                leftFront.setPower(lfPower);
                rightBack.setPower(rbPower);
                rightFront.setPower(rfPower);
        }
        public static void setDrivePower(PoseVelocity2d input) {
                setDrivePower(new Vector2d(input.linearVel.x, input.linearVel.y), input.angVel);
        }
        private static void drawRobot(String stroke, Pose2d pose){
                Canvas canvas = Telem.packet.fieldOverlay();
                canvas.setStroke(stroke);
                Drawing.drawRobot(canvas, pose);
        }
        private static void drawVector(String stroke, Pose2d pose, Vector2d vector){
                Canvas canvas = Telem.packet.fieldOverlay();
                canvas.setStroke(stroke);
                OrderedPair<Double> endPoint = new OrderedPair<>(pose.position.x + 20 * vector.x,pose.position.y + 20 * vector.y);
                canvas.strokeLine(pose.position.x, pose.position.y, endPoint.getX(), endPoint.getY());
        }
        private static void drawArrowNormalized(String stroke, Pose2d pose, Vector2d vector){
                Double max = Math.max(vector.x, vector.y);
                Canvas canvas = Telem.packet.fieldOverlay();
                canvas.setStroke(stroke);
                canvas.strokeLine(pose.position.x, pose.position.y, pose.position.x + 3 * vector.x/max, pose.position.y + 3 * vector.y/max);
        }
        private static PoseVelocity2d computeCommand(Pose2d poseTarget){
                return controller.compute(poseTarget, pose);
        }

        public static void goToTarget(Pose2d poseTarget){
                setDrivePower(computeCommand(poseTarget));
                drawRobot("#4CAF50", poseTarget);
        }
        @NonNull
        public static Lambda nerfDrive(double multiplier){
                return new Lambda("nerf turn speed")
                        .setExecute(() -> {turnNerf = multiplier; });
        }

        @NonNull
        public static Lambda driveCommand() {
            return new Lambda("driveCommand")
                    .addRequirements(INSTANCE)
                    .setExecute(Drive::driveUpdate);
        }
        @NonNull
        public static Lambda PIDToLast() {
                if(lastTarget != null){
                        return PIDToPoint(
                                () -> lastTarget.position.x,
                                () -> lastTarget.position.y,
                                () -> lastTarget.heading.toDouble(), .5, 1,
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
                                lastTarget = pose.value();
                                double translationalError = pose.value().minusExp(Drive.pose).position.norm();
                                double headingError = Math.abs(Math.toDegrees(pose.value().minusExp(Drive.pose).heading.toDouble()));
                                if (!(translationalError < .5 && headingError <  1)) {
                                        goToTarget(pose.value());
                                } else {
                                        setDrivePower(new Vector2d(0, 0), 0.0);
                                }
                        })
                        .setFinish(() -> {
                                Pose2dDual<Time> target = Pose2dDual.constant(pose.value(), 3);
                                double translationalError = target.value().minusExp(Drive.pose).position.norm();
                                double headingError = Math.abs(Math.toDegrees(target.value().minusExp(Drive.pose).heading.toDouble()));
                                return  translationalError < translationalAccuracy && headingError <  headingAccuracy;
                        })
                        .setEnd((interrupted) -> setDrivePower(new Vector2d(0, 0), 0.0));
        }
        public static Pose2d getPose(){
                return pose;
        }
        public static PidToPointBuilderKt p2p(Pose2d pose){
                pinpoint.setPosition(Pinpoint.rrToPinpointPose(pose));
                Drive.pose = pose;
                return new PidToPointBuilderKt(pose, Drive::PIDToPoint, Drive::getPose);
        }

        @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
        public @interface Attach { }

        private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

        @NonNull @Override
        public Dependency<?> getDependency() { return dependency; }

        @Override
        public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
    }