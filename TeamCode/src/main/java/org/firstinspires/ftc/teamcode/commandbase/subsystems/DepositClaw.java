package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class DepositClaw implements Subsystem {
    public static final DepositClaw INSTANCE = new DepositClaw();
    public static Servo  clawServo;
    private DepositClaw() {
    }

    private static final StateMachine<States> clawStates = new StateMachine<>(States.CLOSED)
            .withState(States.CLOSED, (stateRef, name) -> closeClaw())
            .withState(States.OPEN, (stateRef, name) -> openClaw());

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        clawServo = hwmap.get(Servo.class, "depositClaw");
        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    private static void close(){ clawServo.setPosition(1); clawStates.setState(States.CLOSED);}
    private static void open(){ clawServo.setPosition(.7); clawStates.setState(States.OPEN);}

    @NonNull
    public static Lambda toggleClaw(){  
        return new Lambda("claw toggle")
                .setInit(() -> {
                    switch (clawStates.getState()){
                        case OPEN:
                            clawStates.schedule(States.CLOSED);
                            break;
                        case CLOSED:
                            clawStates.schedule(States.OPEN);
                            break;
                    }
                })
                .setFinish(() -> true);
    }


    @NonNull
    public static Lambda closeClaw(){
        return new Lambda("Close Claw")
                .addRequirements(INSTANCE)
                .setInterruptible(false)
                .setInit(DepositClaw::close);
    }

    @NonNull
    public static Lambda openClaw(){
        return new Lambda("open Claw")
                .addRequirements(INSTANCE)
                .setInit(DepositClaw::open);
    }
    private enum States {
        OPEN,
        CLOSED
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }
}