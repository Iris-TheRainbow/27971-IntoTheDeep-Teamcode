package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pathing.SparkFunOTOSDrive.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.pathing.Drawing;
import org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt;
import org.firstinspires.ftc.teamcode.util.Evaluation;
import org.firstinspires.ftc.teamcode.util.LazyPose2d;
import org.firstinspires.ftc.teamcode.util.MathLibKt;
import org.firstinspires.ftc.teamcode.util.PidToPointBuilderKt;
import org.firstinspires.ftc.teamcode.util.Pinpoint;

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

public class drive implements Subsystem {
        public static final drive INSTANCE = new drive();
        private static DcMotorEx leftFront, leftBack, rightFront, rightBack;
        private static Pinpoint pinpoint;
        private static Pose2d pose;
        private static Pose2d lastTarget;
        private static PoseVelocity2d poseVelocity;
        private static MecanumKinematics kinematics;
        private static double turnNerf = 1;
        private static VoltageSensor voltageSensor;
        private static final LinkedList<Pose2d> poseHistory = new LinkedList<>();
        private static TelemetryPacket packet;
        private drive() { }

        @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
        public @interface Attach { }

        private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

        @NonNull @Override
        public Dependency<?> getDependency() { return dependency; }

        @Override
        public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

        @Override
        public void preUserInitHook(@NonNull Wrapper opMode) { }

        @Override
        public void postUserInitHook(@NonNull Wrapper opMode) {
                HardwareMap hwmap = opMode.getOpMode().hardwareMap;
                leftBack = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "leftFront"));
                leftFront = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "rightFront"));
                rightBack = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "rightBack"));
                rightFront = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "leftBack"));
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
                pinpoint = hwmap.get(Pinpoint.class, "pinpoint");
                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                if (opMode.getOpModeType() == OpModeMeta.Flavor.TELEOP) {
                        setDefaultCommand(driveCommand());
                }
                if (opMode.getOpModeType() == OpModeMeta.Flavor.AUTONOMOUS){
                        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                kinematics = new MecanumKinematics(
                        PathingConstantsKt.inPerTick * PathingConstantsKt.trackWidthTicks, PathingConstantsKt.inPerTick / PathingConstantsKt.trackWidthTicks);
        }
        @Override
        public void preUserLoopHook(@NonNull Wrapper opMode){
                packet = new TelemetryPacket();
                pinpoint.update();
                pose = Pinpoint.pinpointToRRPose(pinpoint.getPosition());
                poseVelocity = new PoseVelocity2d( new Vector2d(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH)), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
                poseHistory.add(pose);
                drawPoseHistory(packet.fieldOverlay());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        public static void driveUpdate(){
                driveUpdate(new Vector2d(
                                MathLibKt.signedFunc((x) -> Math.pow(x, 3), Mercurial.gamepad1().leftStickX().state()),
                                MathLibKt.signedFunc((x) -> Math.pow(x, 3), Mercurial.gamepad1().leftStickY().state())
                        ),
                        MathLibKt.signedFunc((x) -> Math.pow(x, 3), Mercurial.gamepad1().rightStickX().state()) * turnNerf);
        }
        public static void driveUpdate(Vector2d vector, Double rotate) {
                double heading = 0;
                // Do the kinematics math
                double rotX = (vector.x * Math.cos(-heading) - vector.y * Math.sin(-heading)) * 1.1;
                double rotY = vector.x * Math.sin(-heading) + vector.y * Math.cos(-heading);
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
        private void drawPoseHistory(Canvas c) {
                double[] xPoints = new double[poseHistory.size()];
                double[] yPoints = new double[poseHistory.size()];

                int i = 0;
                for (Pose2d t : poseHistory) {
                        xPoints[i] = t.position.x;
                        yPoints[i] = t.position.y;

                        i++;
                }

                c.setStrokeWidth(1);
                c.setStroke("#3F51B5");
                c.strokePolyline(xPoints, yPoints);
        }

        public static void goToTarget(Pose2dDual<Time> txWorldTarget){
                Pose2d error = txWorldTarget.value().minusExp(pose);
                PoseVelocity2dDual<Time> command = new HolonomicController(
                        PathingConstantsKt.axialGain, PathingConstantsKt.lateralGain, PathingConstantsKt.headingGain,
                        PathingConstantsKt.axialVelGain, PathingConstantsKt.lateralVelGain, PathingConstantsKt.headingVelGain
                )
                        .compute(txWorldTarget, pose, poseVelocity);

                MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
                double voltage = voltageSensor.getVoltage();

                final MotorFeedforward feedforward = new MotorFeedforward(PathingConstantsKt.kS,
                PathingConstantsKt.kA / PathingConstantsKt.inPerTick, PathingConstantsKt.kV / PathingConstantsKt.inPerTick);
                double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;


                leftFront.setPower(leftFrontPower);
                leftBack.setPower(leftBackPower);
                rightBack.setPower(rightBackPower);
                rightFront.setPower(rightFrontPower);

                Canvas canvas = packet.fieldOverlay();
                canvas.setStroke("#4CAF50");
                Drawing.drawRobot(canvas, txWorldTarget.value());

                canvas.setStroke("#3F51B5");
                Drawing.drawRobot(canvas, pose);
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
                    .setExecute(drive::driveUpdate);
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
                                Pose2dDual<Time> target = Pose2dDual.constant(pose.value(), 3);
                                lastTarget = pose.value();
                                double translationalError = target.value().minusExp(drive.pose).position.norm();
                                double headingError = Math.abs(Math.toDegrees(target.value().minusExp(drive.pose).heading.toDouble()));
                                if (!(translationalError < .5 && headingError <  1)) {
                                        goToTarget(target);
                                } else {
                                        driveUpdate(new Vector2d(0, 0), 0.0);
                                }
                        })
                        .setFinish(() -> {
                                Pose2dDual<Time> target = Pose2dDual.constant(pose.value(), 3);
                                double translationalError = target.value().minusExp(drive.pose).position.norm();
                                double headingError = Math.abs(Math.toDegrees(target.value().minusExp(drive.pose).heading.toDouble()));
                                return  translationalError < translationalAccuracy && headingError <  headingAccuracy;
                        })
                        .setEnd((interrupted) -> driveUpdate(new Vector2d(0, 0), 0.0));
        }
        public static Pose2d getPose(){
                return pose;
        }
        public static PidToPointBuilderKt p2p(Pose2d pose){
                pinpoint.setPosition(Pinpoint.rrToPinpointPose(pose));
                drive.pose = pose;
                return new PidToPointBuilderKt(pose, drive::PIDToPoint, drive::getPose);
        }
    }