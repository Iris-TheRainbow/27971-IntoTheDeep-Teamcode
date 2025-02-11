package org.firstinspires.ftc.teamcode.roadrunner;



import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;


@Config
public class SparkFunOTOSDrive extends WavedashMecanumDrive {
    public static class Params {
        public double OtosOffsetX = -3.0998, OtosOffsetY = -3.0397, OtosOffsetH = -1.5672;

        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(OtosOffsetX, OtosOffsetY, OtosOffsetH);
        //target over actual
        public double linearScalar = .985; // 1.2084
        public double angularScalar = .995;
    }

    public static SparkFunOTOSDrive.Params PARAMS = new SparkFunOTOSDrive.Params();
    public SparkFunOTOSCorrected otos;
    private Pose2d lastOtosPose = pose;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    public SparkFunOTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("OTOS_PARAMS",PARAMS);
        otos = hardwareMap.get(SparkFunOTOSCorrected.class,"otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(PARAMS.offset);
        System.out.println("OTOS calibration beginning!");
        System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
        System.out.println(otos.setAngularScalar(PARAMS.angularScalar));

        otos.setPosition(RRPoseToOTOSPose(pose));

        System.out.println(otos.calibrateImu(255, true));
        System.out.println("OTOS calibration complete!");
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastOtosPose != pose) {
            otos.setPosition(RRPoseToOTOSPose(pose));
        }

        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        pose = OTOSPoseToRRPose(otosPose);
        lastOtosPose = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

         return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y),otosVel.h);
    }


}
