package org.firstinspires.ftc.teamcode.commandbase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.arm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.deposit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.lift;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class CommandGroups {
    public static Command transfer(){
        return new Sequential(new Parallel(deposit.wristTransfer(),new Wait(.1), arm.armTransfer()),new Wait(.05), deposit.closeClaw(),new Wait(.05), intake.openClaw());
    }
    public static Command intake(){
        return new Sequential(new Parallel(lift.goTo(0), extendo.goTo(475), intake.openClaw(), deposit.wristTransfer(), arm.armWait(), intake.wristExtend()), intake.wristIntake());
    }
    public static Command intakeAuto(){
        return new Sequential(new Parallel(extendo.goTo(430), intake.openClaw(), deposit.wristTransfer(), arm.armWait(), intake.wristExtend()), intake.wristIntake());
    }
    public static Command intakeAutoShort(){
        return new Sequential(new Parallel(lift.goTo(0), extendo.goTo(465), intake.openClaw(), deposit.wristWait(), arm.armWait(), intake.wristExtend()), intake.wristIntake());
    }
    public static Command retract(){
        return new Parallel(intake.wristTransfer(), intake.closeClaw(), deposit.openClaw(), deposit.wristWait(), arm.armWait(), lift.goTo(0), extendo.goTo(0));
    }
    public static Command prepDepo(){
        return new Parallel(deposit.wristDeposit(), arm.armExtend(), deposit.closeClaw());
    }
    private static Command liftGoTo(int ticks){
        return new Sequential(new Parallel(prepDepo(), lift.goTo(ticks)), arm.armUp());
    }
    public static Command liftHigh(){
        return liftGoTo(1500);
    }
    public static Command liftMedium(){
        return new Parallel(lift.goTo(580), deposit.wristSepc(), arm.armUp());
    }
    public static Command depositSpec(){
        return new Sequential(deposit.wristSepc(), lift.goTo(250), deposit.openClaw());
    }
    public static Command liftHang(){
        return new Parallel(lift.goTo(1700), deposit.wristTransfer(), arm.armWait(), extendo.goTo(0), intake.wristTransfer());
    }
}
