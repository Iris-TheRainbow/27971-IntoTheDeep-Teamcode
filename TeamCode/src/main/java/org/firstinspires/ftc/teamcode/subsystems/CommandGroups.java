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
        return new Sequential(new Parallel(lift.goTo(0), extendo.goTo(485), intake.openClaw(), deposit.wristTransfer(), arm.armWait()), intake.wristIntake());
    }
    public static Command retract(){
        return new Parallel(intake.wristTransfer(), intake.closeClaw(), deposit.openClaw(), deposit.wristTransfer(), arm.armWait(), lift.goTo(0), extendo.goTo(0));
    }
    public static Command prepDepo(){
        return new Parallel(deposit.wristDeposit(), arm.armUp(), deposit.closeClaw());
    }
    private static Command liftGoTo(int ticks){
        return new Sequential(new Parallel(prepDepo(), lift.goTo(ticks)));
    }
    public static Command liftHigh(){
        return liftGoTo(1800);
    }
    public static Command liftMedium(){
        return new Parallel(lift.goTo(1000), deposit.wristSepc());
    }
    public static Command depositSpec(){
        return new Sequential(lift.goTo(800), deposit.openClaw(), new Wait(.3), lift.goTo(1000));
    }
    public static Command liftHang(){
        return new Parallel(lift.goTo(1800), deposit.wristTransfer(), arm.armWait(), extendo.goTo(0), intake.wristTransfer());
    }
}
