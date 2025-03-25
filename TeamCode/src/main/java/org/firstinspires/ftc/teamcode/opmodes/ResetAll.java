package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.features.BulkRead;
import org.firstinspires.ftc.teamcode.util.features.LoopTimes;
import org.firstinspires.ftc.teamcode.util.features.SlothFinder;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.mercurial.Mercurial;

@Autonomous
@Mercurial.Attach
@DepositArm.Attach
@DepositWrist.Attach
@DepositClaw.Attach
@IntakeWrist.Attach
@IntakeClaw.Attach
@IntakeRotate.Attach
@Lift.Attach
@Extendo.Attach
@Drive.Attach
@BulkRead.Attach
@LoopTimes.Attach
@SlothFinder.Attach
@Telem.Attach
public class ResetAll extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void start(){
        DepositArm.armTransfer().schedule();
        DepositWrist.wristSepc().schedule();
        Lift.reset();
        Extendo.reset();
    }
    @Override
    public void loop() {

    }
}
