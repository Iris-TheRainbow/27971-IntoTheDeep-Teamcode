package org.firstinspires.ftc.teamcode.subsystems;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class CommandGroups {
    public static Command transfer(){
        return new Sequential(arm.armTransfer(), deposit.closeClaw(),new Wait(.3), intake.openClaw());
    }
    public static Command intake(){
        return new Sequential(new Parallel(lift.retract(), extendo.goTo(485), intake.openClaw(), deposit.wristTransfer(), arm.armWait()), intake.wristIntake());
    }
    public static Command intakeAuto(){
        return new Sequential(new Parallel(extendo.goTo(430), intake.openClaw(), deposit.wristTransfer(), arm.armWait()), intake.wristIntake());
    }
    public static Command intakeAutoShort(){
        return new Sequential(new Parallel(lift.retract(), extendo.goTo(405), intake.openClaw(), deposit.wristTransfer(), arm.armWait()), intake.wristIntake());
    }
    public static Command retract(){
        return new Parallel(intake.wristTransfer(), intake.closeClaw(), deposit.openClaw(), deposit.wristTransfer(), arm.armWait(), lift.retract(), extendo.goTo(0));
    }
    public static Command prepDepo(){
        return new Parallel(deposit.wristDeposit(), arm.armUp(), deposit.closeClaw());
    }
    private static Command liftGoTo(int ticks){
        return new Sequential(new Parallel(prepDepo(), lift.goTo(ticks)));
    }
    public static Command liftHigh(){
        return liftGoTo(1700);
    }
    public static Command liftMedium(){
        return new Parallel(lift.goTo(840), deposit.wristSepc());
    }
    public static Command depositSpec(){
        return new Sequential(lift.goTo(650), new Wait(.3), deposit.openClaw(), new Wait(.3), lift.goTo(840));
    }
    public static Command liftHang(){
        return new Parallel(lift.goTo(1800), deposit.wristTransfer(), arm.armWait(), extendo.goTo(0), intake.wristTransfer());
    }
}
