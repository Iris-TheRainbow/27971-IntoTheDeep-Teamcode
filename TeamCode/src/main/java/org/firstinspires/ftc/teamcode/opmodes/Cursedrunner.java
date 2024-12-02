package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.CursedOTOSDrive;
import org.firstinspires.ftc.teamcode.util.SilkRoad;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.mercurial.Mercurial;

@SilkRoad.Attach
@Mercurial.Attach
public class Cursedrunner extends OpMode {
    CursedOTOSDrive CursedRunner = new CursedOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
    Action testSpline;
    @Override
    public void init() {
        testSpline = CursedRunner.actionBuilder(new Pose2d(0,0,0))
                .splineTo(new Vector2d(24,24), Math.toRadians(90))
                .build();
    }

    @Override
    public void start() {
        SilkRoad.RunAsync(testSpline);
    }

    @Override
    public void loop() {
    }
}
