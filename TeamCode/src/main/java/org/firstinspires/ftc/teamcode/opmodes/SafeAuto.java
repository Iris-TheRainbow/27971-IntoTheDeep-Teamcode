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
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.util.MercurialAction;
import org.firstinspires.ftc.teamcode.util.SilkRoad;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;

@Autonomous(name = "safe spec")
@SilkRoad.Attach
@Mercurial.Attach
@lift.Attach
@intake.Attach
@deposit.Attach
@arm.Attach
@BulkRead.Attach
public class SafeAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(24, -62.5, Math.toRadians(0));
    private Action path0;
    private SparkFunOTOSDrive drive;

    @Override
    public void init() { drive = new SparkFunOTOSDrive(hardwareMap, initialPose); }
    @Override
    public void init_loop(){

    TrajectoryActionBuilder tab0 = drive.actionBuilder(initialPose)
            .strafeTo(new Vector2d(60, -62.5));
    path0 = tab0.build();
    }

    @Override
    public void start(){
        SilkRoad.RunAsync(
                new SequentialAction(
                        new MercurialAction(new Parallel(intake.closeClaw(), deposit.wristDown(), arm.armStow(), lift.goTo(0))),
                        new SleepAction(1),
                        path0
                        )
                );
    }
    @Override
    public void loop() {

    }
}
