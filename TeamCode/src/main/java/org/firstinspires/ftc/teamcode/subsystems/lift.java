package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PDFController;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.util.controller.implementation.DoubleController;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.CommandGroup;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class lift implements Subsystem {
    public static final lift INSTANCE = new lift();
    private static DcMotorEx liftLeft, liftRight, liftEncoder;
    private static int liftTarget;
    private static PDFController pid;
    private static double kP = .004, kD = .0004, kF = 0;
    private static boolean cancle = false;
    private lift() { }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        liftLeft = hwmap.get(DcMotorEx.class, "liftLeft");
        liftRight = hwmap.get(DcMotorEx.class, "liftRight");
        liftEncoder = hwmap.get(DcMotorEx.class, "leftFront");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        setDefaultCommand(update());
        pid = new PDFController(kP, kD, kF);
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }

    public static void setTarget(int target){ liftTarget = target; }

    public static int getTarget(){ return liftTarget; }

    public static void pidUpdate() {
        pid.setSetPoint(liftTarget);
        double power = pid.calculate(liftTarget, liftEncoder.getCurrentPosition());
        liftRight.setPower(power);
        liftLeft.setPower(power);
    }

    public static void hold() {
        liftRight.setPower(kF);
        liftLeft.setPower(kF);
    }


    @NonNull
    public static Lambda update() {
        return new Lambda("update the pid")
                .addRequirements(INSTANCE)
                .setExecute(lift::pidUpdate)
                .setFinish(() -> false);
    }

    @NonNull
    public static Lambda goTo(int to){
        return new Lambda("set pid target")
                .setExecute(() -> setTarget(to));
    }

    @NonNull
    public static Lambda hang(){
        return new Lambda("hang mode")
                .setRequirements(INSTANCE)
                .setExecute(() -> {
                    if (Mercurial.gamepad1().rightTrigger().state() >= .1){
                        setTarget((int) (-20 *  Mercurial.gamepad1().rightTrigger().state()) + getTarget());
                    }
                    if (Mercurial.gamepad1().leftTrigger().state() >= .1){
                        setTarget((int) (20 *  Mercurial.gamepad1().leftTrigger().state()) + getTarget());
                    }
                })
                .setFinish(() -> {
                    if (cancle){
                        cancle = false;
                        return true;
                    }
                    return false;
                })
                .setInterruptible(true);
    }
    @NonNull
    public static Lambda hangOff(){
        return new Lambda("hang off")
                .setRequirements(INSTANCE)
                .setExecute(() -> cancle = true);
    }
}