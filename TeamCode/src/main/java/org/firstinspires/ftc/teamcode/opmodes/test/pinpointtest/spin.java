package org.firstinspires.ftc.teamcode.opmodes.test.pinpointtest;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.arm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.deposit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.lift;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;

@TeleOp
@Mercurial.Attach
@extendo.Attach
@lift.Attach
@arm.Attach
@intake.Attach
@deposit.Attach
@drive.Attach
@Telem.Attach
public class spin extends OpMode {
    private Command driveCommand;
    @Override
    public void init() {
        RobotLog.a("fuck this shit");
    }
    @Override
    public void start(){
        RobotLog.a("fuck this shit 2 the fuckening");
        driveCommand = drive.p2p(new Pose2d(0,0,0))
                .stopAndAdd(() -> RobotLog.i("begin pinpoint pi/2 spin test"))
                .stopAndAdd(() -> RobotLog.v("pinpoint to pi/2"))
                .pidTo(new Pose2d(0, 0, Math.PI/2))
                .stopAndAdd(() -> RobotLog.v("pinpoint to 0"))
                .pidTo(new Pose2d(0,0,0))

                .stopAndAdd(() -> RobotLog.i("begin pinpoint pi spin test"))
                .stopAndAdd(() -> RobotLog.v("pinpoint to pi"))
                .pidTo(new Pose2d(0, 0, Math.PI))
                .stopAndAdd(() -> RobotLog.v("pinpoint to 0"))
                .pidTo(new Pose2d(0,0,0))

                .stopAndAdd(() -> RobotLog.i("begin pinpoint pi/2 spin test"))
                .stopAndAdd(() -> RobotLog.v("pinpoint to pi/4"))
                .pidTo(new Pose2d(0, 0, Math.PI/4))
                .stopAndAdd(() -> RobotLog.v("pinpoint to 0"))
                .pidTo(new Pose2d(0,0,0))
                .build();
        driveCommand.schedule();
        for (Command command: Mercurial.INSTANCE.getActiveCommandSnapshot()){
            RobotLog.v(command.toString());
        }
        String a = " " + driveCommand.toString();
        RobotLog.a("DriveCommand:"+ a);

    }
    @Override
    public void loop() {
        for (Command command: Mercurial.INSTANCE.getActiveCommandSnapshot()){
            RobotLog.v(command.toString());
        }
    }
}
