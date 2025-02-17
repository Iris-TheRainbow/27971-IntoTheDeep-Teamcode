package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class WavedashMecanumDrive implements Drive {
    public Pose2d pose;
    public static Params PARAMS = new Params();
    private Pose2dDual<Time> lastTxWorldTarget;
    public Pose2dDual<Time> getLastTxWorldTarget(){
        return this.lastTxWorldTarget;
    }

    public void goToTarget(Pose2dDual<Time> txWorldTarget, Canvas canvas){
        PoseVelocity2d robotVelRobot = updatePoseEstimate();
        Pose2d error = txWorldTarget.value().minusExp(this.pose);
        PoseVelocity2dDual<Time> command = new HolonomicController(
                PARAMS.axialGain - 2, PARAMS.lateralGain - 2, PARAMS.headingGain - 2,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        )
                .compute(txWorldTarget, pose, robotVelRobot);

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        drawPoseHistory(canvas);

        canvas.setStroke("#4CAF50");
        Drawing.drawRobot(canvas, txWorldTarget.value());

        canvas.setStroke("#3F51B5");
        Drawing.drawRobot(canvas, pose);
        lastTxWorldTarget = txWorldTarget;
    }



    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
        public double lateralInPerTick =  .65;
        public double trackWidthTicks = 12;

        // feedforward parameters (in tick units)
        public double kS =  1.9;
        public double kV = .145;
        public double kA = .05;

        // path profile parameters (in inches)
        public double maxWheelVel = 70;
        public double minProfileAccel = -50;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = 2 * Math.PI; // shared with path
        public double maxAngAccel = 1* Math.PI;

        // path controller gains
        public double axialGain = 6;
        public double lateralGain = 6;
        public double headingGain = 7; // shared with turn

        public double axialVelGain = .15;
        public double lateralVelGain = .15;
        public double headingVelGain = .15; // shared with turn
    }

    public final MecanumKinematics kinematics;

    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint defaultVelConstraint;

    public final AccelConstraint defaultAccelConstraint;

    public final DcMotorEx leftFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;

    public final DcMotorEx rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public WavedashMecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        leftBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBack"));
        rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftBack"));

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
         class DriveLocalizer implements Localizer {
            public final IMU imu;

            private double lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
            private Rotation2d lastHeading;
            private boolean initialized;

            public DriveLocalizer() {
                imu = lazyImu.get();
            }

            @Override
             public Twist2dDual<Time> update(){
                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }
        }
        localizer = new DriveLocalizer();

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);

        kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));

        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

        defaultAccelConstraint =
                new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
    }

    @NonNull
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    @Override
    public void setDrivePowers(@NonNull PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    @Override
    public void setDrivePowersWithFF(@NonNull PoseVelocity2d powers) {
        setDrivePowersWithFF(PoseVelocity2dDual.constant(powers, 1));
    }

    public void setDrivePowersWithFF(PoseVelocity2dDual<Time> powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(powers);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
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

    public TrajectoryBuilder trajectoryBuilder(Pose2d beginPose) {
        return new TrajectoryBuilder(
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose,
                0.0,
                defaultVelConstraint,
                defaultAccelConstraint
        );
    }

    /**
     * Follow a trajectory.
     * @param trajectory trajectory to follow
     * @param t time to follow in seconds
     * @return whether the trajectory has been completed
     */
    public boolean followTrajectory(TimeTrajectory trajectory, double t) {
        if (t >= trajectory.duration) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            return true;
        }

        Pose2dDual<Time> txWorldTarget = trajectory.get(t);
        targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        )
                .compute(txWorldTarget, pose, robotVelRobot);
        driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        return false;
    }

    /**
     * Follow a trajectory.
     * @param trajectory trajectory to follow
     * @param t time to follow in seconds
     * @return whether the trajectory has been completed
     **/
    public boolean followTrajectory(@NonNull Trajectory trajectory, double t) {
        return followTrajectory(new TimeTrajectory(trajectory), t);
    }

    @Override
    public boolean turn(TimeTurn turn, double t) {
        if (t >= turn.duration) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            return true;
        }

        Pose2dDual<Time> txWorldTarget = turn.get(t);
        targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        )
                .compute(txWorldTarget, pose, robotVelRobot);
        driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();
        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
        leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
        rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
        rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

        return false;
    }

    public class FollowTrajectory{
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private double timeSinceStart;

        private final double[] xPoints, yPoints;
        public FollowTrajectory(TimeTrajectory t){
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        public void execute(@NonNull TelemetryPacket p){
            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(timeSinceStart);
            lastTxWorldTarget = txWorldTarget;
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }

        public boolean isFinished(){
            if (beginTs < 0) {
                beginTs = Actions.now();
                timeSinceStart = 0;
            } else {
                timeSinceStart = Actions.now() - (beginTs + .25);
            }
            return timeSinceStart >= timeTrajectory.duration;
        }

        public void end(){
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }

        public void preview(Canvas c){
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }

    }

    public final class FollowTrajectoryAction implements Action {
        private final FollowTrajectory traj;

        public FollowTrajectoryAction(TimeTrajectory t) {
            traj = new FollowTrajectory(t);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (traj.isFinished()){
                traj.end();
                return false;
            }
            traj.execute(p);
            return true;
        }

        @Override
        public void preview(Canvas c) {
            traj.preview(c);
        }
    }
    public class Turn{
        private final TimeTurn turn;
        private double beginTs = -1;
        private double timeSinceStart;

        public Turn(TimeTurn turn){
            this.turn = turn;
        }
        public void execute(TelemetryPacket p){
            Pose2dDual<Time> txWorldTarget = turn.get(timeSinceStart);
            lastTxWorldTarget = txWorldTarget;
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
        public boolean isFinished(){
            if (beginTs < 0) {
                beginTs = Actions.now();
                timeSinceStart = 0;
            } else {
                timeSinceStart = Actions.now() - beginTs;
            }
            return timeSinceStart >= turn.duration;
        }
        public void end(){
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }

        public void preview(Canvas c){
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public final class TurnAction implements Action {
        private final Turn turn;

        public TurnAction(TimeTurn turn) {
            this.turn = new Turn(turn);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (turn.isFinished()){
                turn.end();
                return false;
            }
            turn.execute(p);
            return true;
        }

        @Override
        public void preview(Canvas c) {
            turn.preview(c);
        }
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
//    @NonNull
//    @Override
//    public TrajectoryCommandBuilder commandBuilder(@NonNull Pose2d beginPose) {
//        return new TrajectoryCommandBuilder(
//                this::followTrajectoryCommand,
//                this::turnCommand,
//                beginPose,
//                new TrajectoryBuilderParams(
//                        1e-6,
//                        new ProfileParams(
//                                0.25, 0.1, 1e-2
//                        )
//                ),
//                defaultVelConstraint,
//                defaultAccelConstraint,
//                defaultTurnConstraints
//        );
//    }


}
