package org.firstinspires.ftc.teamcode.pathing.tuning;

import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OtosEncoder;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.pathing.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.SparkFunOTOSDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveViewF {
    public static DriveViewFactory getDvf(){
        DriveViewFactory dvf = hardwareMap -> {
            SparkFunOTOSDrive od = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));

            List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
            List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
            parEncs.add(new OtosEncoder(od.otos, false, false, od.leftBack));
            perpEncs.add(new OtosEncoder(od.otos, true, false, od.leftBack));

            return new DriveView(
                    DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    hardwareMap.getAll(LynxModule.class),
                    Arrays.asList(
                            od.leftFront,
                            od.leftBack
                    ),
                    Arrays.asList(
                            od.rightFront,
                            od.rightBack
                    ),
                    leftEncs,
                    rightEncs,
                    parEncs,
                    perpEncs,
                    od.lazyImu,
                    od.voltageSensor,
                    () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick)
            );
        };
        return dvf;
    }
}
