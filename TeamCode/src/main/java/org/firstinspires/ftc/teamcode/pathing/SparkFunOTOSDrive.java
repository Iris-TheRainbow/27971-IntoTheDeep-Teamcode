package org.firstinspires.ftc.teamcode.pathing;



import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.FlightRecorderKt;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pathing.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.util.Pinpoint;

import dev.frozenmilk.dairy.core.FeatureRegistrar;


@Config
public class SparkFunOTOSDrive extends WavedashMecanumDrive {
    public static class Params {
        public double OtosOffsetX = -3.0998, OtosOffsetY = -3.0397, OtosOffsetH = -1.5672;

        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(OtosOffsetX, OtosOffsetY, OtosOffsetH);
        //target over actual
        public double linearScalar = .985; // 1.2084
        public double angularScalar = .995;
    }
    public static class PinpointParams{
        public double xOffsetMM = 75;
        public double yOffsetMM = 27;
        public Pinpoint.EncoderDirection xDirection = Pinpoint.EncoderDirection.REVERSED;
        public Pinpoint.EncoderDirection yDirection = Pinpoint.EncoderDirection.FORWARD;
    }
    public static PinpointParams PINPOINTPARAMS = new PinpointParams();
    public static SparkFunOTOSDrive.Params PARAMS = new SparkFunOTOSDrive.Params();
    public SparkFunOTOSCorrected otos;
    public Pinpoint pinpoint;
    private Pose2d lastPose = pose;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    public SparkFunOTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("OTOS_PARAMS",PARAMS);
        FlightRecorder.write("PINPOINT_PARAMS", PINPOINTPARAMS);
        otos = hardwareMap.get(SparkFunOTOSCorrected.class,"otos");
        pinpoint = hardwareMap.get(Pinpoint.class, "pinpoint");
        pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(PINPOINTPARAMS.xDirection, PINPOINTPARAMS.yDirection);
        pinpoint.setOffsets(PINPOINTPARAMS.xOffsetMM, PINPOINTPARAMS.yOffsetMM, DistanceUnit.MM);

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(PARAMS.offset);
        System.out.println("OTOS calibration beginning!");
        System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
        System.out.println(otos.setAngularScalar(PARAMS.angularScalar));
        pinpoint.resetPosAndIMU();
        otos.setPosition(RRPoseToOTOSPose(pose));
        System.out.println(otos.calibrateImu(255, false));
        System.out.println("OTOS calibration complete!");
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble()));
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPose != pose) {
            otos.setPosition(RRPoseToOTOSPose(pose));
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble()));
        }

        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        pinpoint.update();

        pose = new Pose2d(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.RADIANS));
        lastPose = pose;
        FlightRecorder.write("PinpointPose", new PoseMessage(pose));
        FlightRecorder.write("PinpointVel", new PoseMessage(new Pose2d(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))));
        FlightRecorder.write("PinpointStatus", pinpoint.getDeviceStatus());
        FlightRecorder.write("OtosPose", new PoseMessage(OTOSPoseToRRPose(otosPose)));
        FlightRecorder.write("OtosVel", new PoseMessage(OTOSPoseToRRPose(otosVel)));

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));
        Pose2d delta = pose.minusExp(OTOSPoseToRRPose(otosPose));
        //FeatureRegistrar.getActiveOpMode().telemetry.addLine("OTOS vs PP Pose: x:" + delta.position.x + " y: " + delta.position.y + " h: " + Math.toDegrees(delta.heading.toDouble()));
        return new PoseVelocity2d(new Vector2d(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH)), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }


}
