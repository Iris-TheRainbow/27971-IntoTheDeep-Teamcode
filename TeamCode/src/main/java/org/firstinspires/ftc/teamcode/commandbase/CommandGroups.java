package org.firstinspires.ftc.teamcode.commandbase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class CommandGroups {
    public static Command transfer(){
        return new Sequential(new Parallel(DepositWrist.wristTransfer(),new Wait(.1), DepositArm.armTransfer()),new Wait(.05), DepositClaw.closeClaw(),new Wait(.05), IntakeClaw.openClaw());
    }
    public static Command intake(){
        return new Sequential(new Parallel(Lift.goTo(0), Extendo.goTo(475), IntakeClaw.openClaw(), DepositWrist.wristTransfer(), DepositArm.armWait(), IntakeWrist.wristExtend()), IntakeWrist.wristIntake());
    }
    public static Command intakeAuto(){
        return new Sequential(new Parallel(Extendo.goTo(430), IntakeClaw.openClaw(), DepositWrist.wristTransfer(), DepositArm.armWait(), IntakeWrist.wristExtend()), IntakeWrist.wristIntake());
    }
    public static Command intakeAutoShort(){
        return new Sequential(new Parallel(Lift.goTo(0), Extendo.goTo(465), IntakeClaw.openClaw(), DepositWrist.wristWait(), DepositArm.armWait(), IntakeWrist.wristExtend()), IntakeWrist.wristIntake());
    }
    public static Command retract(){
        return new Parallel(IntakeWrist.wristTransfer(), IntakeClaw.closeClaw(), DepositClaw.openClaw(), DepositWrist.wristWait(), DepositArm.armWait(), Lift.goTo(0), Extendo.goTo(0));
    }
    public static Command prepDepo(){
        return new Parallel(DepositWrist.wristDeposit(), DepositArm.armExtend(), DepositClaw.closeClaw());
    }
    private static Command liftGoTo(int ticks){
        return new Sequential(new Parallel(prepDepo(), Lift.goTo(ticks)), DepositArm.armUp());
    }
    public static Command liftHigh(){
        return liftGoTo(1500);
    }
    public static Command liftMedium(){
        return new Parallel(Lift.goTo(580), DepositWrist.wristSepc(), DepositArm.armUp());
    }
    public static Command depositSpec(){
        return new Sequential(DepositWrist.wristSepc(), Lift.goTo(250), DepositClaw.openClaw());
    }
    public static Command liftHang(){
        return new Parallel(Lift.goTo(1700), DepositWrist.wristTransfer(), DepositArm.armWait(), Extendo.goTo(0), IntakeWrist.wristTransfer());
    }
}
