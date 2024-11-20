package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.claw;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.wrist;
import org.firstinspires.ftc.teamcode.util.MercurialAction;
import org.firstinspires.ftc.teamcode.util.SilkRoad;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;

@Autonomous(name = "no Sample")
@SilkRoad.Attach
@Mercurial.Attach
@lift.Attach
@claw.Attach
@wrist.Attach
@arm.Attach
@BulkRead.Attach
public class SafeSample extends OpMode {
    private final Pose2d initialPose = new Pose2d(-24, -62.5, Math.toRadians(180));
    private Action path0, path1, path2, path3, path4, path5, path6;
    private SparkFunOTOSDrive drive;

    @Override
    public void init() { drive = new SparkFunOTOSDrive(hardwareMap, initialPose); }

    @Override
    public void init_loop(){


    TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
            .setReversed(true)
            .splineTo(new Vector2d(-34, -48), Math.toRadians(0))
            .setReversed(false)
            .splineTo(new Vector2d(-30, -50), Math.toRadians(0));
    path3 = tab3.build();
    }

    @Override
    public void start(){
        SilkRoad.RunAsync(
                new SequentialAction(
                        path3,
                        new SleepAction(1),
                        new MercurialAction(new Parallel(arm.armUp(), wrist.wristFlat()))
                        )
                );
    }

    @Override
    public void loop() {

    }
}
