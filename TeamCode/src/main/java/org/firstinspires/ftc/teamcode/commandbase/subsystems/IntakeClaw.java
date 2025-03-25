package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class IntakeClaw implements Subsystem {
    private static Servo clawServo;
    public static final IntakeClaw INSTANCE = new IntakeClaw();

    private IntakeClaw() { }

    private static final StateMachine<States> clawStates = new StateMachine<>(States.CLOSED)
            .withState(States.CLOSED, (stateRef, name) -> closeClaw())
            .withState(States.OPEN, (stateRef, name) -> openClaw());

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        clawServo = new CachingServo(hwmap.get(Servo.class, "intakeClaw"));
        clawServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){
        clawServo.setPosition(0);
    }
    private static void close(){ clawServo.setPosition(.02); clawStates.setState(States.CLOSED);
    }
    private static void open(){
        clawServo.setPosition(.14); clawStates.setState(States.OPEN); }

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
                .setInit(IntakeClaw::close);
    }

    @NonNull
    public static Lambda openClaw(){
        return new Lambda("Close Claw")
                .addRequirements(INSTANCE)
                .setInit(IntakeClaw::open);
    }

    private enum States{
        OPEN,
        CLOSED
    }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}