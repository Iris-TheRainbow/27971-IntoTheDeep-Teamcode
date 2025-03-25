package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.controllers.SquIDSLController;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();
    private static DcMotorEx liftLeft, liftLeft2, liftRight, liftEncoder;
    private static int liftTarget;
    public static int liftOffset;
    private static double power;
    private static SquIDSLController squid;
    private static final double kP = .006;
    private static final double kD = 0;
    private static final double kS = 0;
    private static final int tollerence = 30;
    private static Telemetry telem;
    private Lift() { }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        telem = opMode.getOpMode().telemetry;
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        liftLeft = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "liftLeft"));
        liftLeft2 = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "liftLeft2"));
        liftRight = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "liftRight"));
        liftEncoder = hwmap.get(DcMotorEx.class, "leftFront");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        setDefaultCommand(update());
        squid = new SquIDSLController(kP, kD, kS);
    }

    public static void setTarget(int target){ liftTarget = (liftOffset + target); }

    public static int getTarget(){ return liftTarget; }
    private static void setPower(double power){
        liftRight.setPower(power);
        liftLeft.setPower(power);
        liftLeft2.setPower(power);
    }
    private static void setMode(DcMotor.RunMode mode){
        liftRight.setMode(mode);
        liftLeft.setMode(mode);
        liftLeft2.setMode(mode);
    }
    public static void pidUpdate() {
        power = squid.calculate(liftTarget, -liftEncoder.getCurrentPosition());
        setPower(power);
    }

    public static void reset(){
        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static boolean atTarget() { return Math.abs(liftTarget + liftEncoder.getCurrentPosition()) < tollerence; }

    @NonNull
    public static Lambda update() {
        return new Lambda("update the pid")
                .addRequirements(INSTANCE)
                .setExecute(Lift::pidUpdate)
                .setInterruptible(() -> true)
                .setFinish(() -> false);
    }

    @NonNull
    public static Lambda goTo(int to){
        return new Lambda("set pid target")
                .setExecute(() -> setTarget(to))
                .setFinish(Lift::atTarget);
    }

    @NonNull
    public static Lambda offset(int offset){
        return new Lambda("set pid target")
                .setInit(() -> {liftOffset += (offset);});
    }

    @NonNull
    public static Lambda retract(){
        return new Lambda("retract lift")
                .setRequirements(INSTANCE)
                .setInit(() -> {
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    });
    }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

}