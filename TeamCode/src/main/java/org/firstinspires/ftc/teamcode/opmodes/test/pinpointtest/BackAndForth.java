package org.firstinspires.ftc.teamcode.opmodes.test.pinpointtest;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;

@TeleOp
@Mercurial.Attach
@Extendo.Attach
@Lift.Attach
@DepositArm.Attach
@IntakeWrist.Attach
@DepositWrist.Attach
@Drive.Attach
@Telem.Attach
public class BackAndForth extends OpMode {
    private Command driveCommand;
    @Override
    public void init() {
        driveCommand = Drive.p2p(new Pose2d(0,0,0))
                .stopAndAdd(() -> RobotLog.i("begin pinpoint 5in back and forth"))
                .repeat(10)
                .stopAndAdd(() -> RobotLog.v("pinpoint to 5in"))
                .pidTo(new Pose2d(5, 0, 0))
                .stopAndAdd(() -> RobotLog.v("pinpoint to 0in"))
                .pidTo(new Pose2d(0,0,0))
                .stopRepeat()

                .stopAndAdd(() -> RobotLog.i("begin pinpoint 10in back and forth"))
                .repeat(10)
                .stopAndAdd(() -> RobotLog.v("pinpoint to 10in"))
                .pidTo(new Pose2d(10, 0, 0))
                .stopAndAdd(() -> RobotLog.v("pinpoint to 0in"))
                .pidTo(new Pose2d(0,0,0))
                .stopRepeat()

                .stopAndAdd(() -> RobotLog.i("\"begin pinpoint 20in back forth\""))
                .repeat(10)
                .stopAndAdd(() -> RobotLog.v("pinpoint to 20in"))
                .pidTo(new Pose2d(20, 0, 0))
                .stopAndAdd(() -> RobotLog.v("pinpoint to 0"))
                .pidTo(new Pose2d(0,0,0))
                .stopAndAdd(this::func)
                .stopRepeat()
                .build();
    }
    public double func(){
        return 0.0;
    }
    @Override
    public void start(){
        driveCommand.schedule();

    }
    @Override
    public void loop() {

    }
}
