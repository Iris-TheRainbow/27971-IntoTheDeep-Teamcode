package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PDFController;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class extendo implements Subsystem {
    public static final extendo INSTANCE = new extendo();
    private static DcMotorEx extendoMotor, extendoEncoder;
    private static int liftTarget;
    private static PDFController pid;
    private static double power;
    private static double kP = .006, kD = .0006, kF = 0;
    private static int tollerence = 10;
    private extendo() { }

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
        extendoMotor = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "extendo"));
        extendoEncoder = hwmap.get(DcMotorEx.class, "rightFront");
        extendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pid = new PDFController(kP, kD, 0);
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){
        setDefaultCommand(update());
    }
    public static void setTarget(int target){ liftTarget = target; }

    public static int getTarget(){ return liftTarget; }

    public static void pidUpdate() {
        pid.setSetPoint(liftTarget);
        power = pid.calculate(liftTarget, -extendoEncoder.getCurrentPosition()) + kF;
        extendoMotor.setPower(power);
    }

    public static void hold() {
        extendoMotor.setPower(kF);
    }
    public static double getPower(){
        return power;
    }

    public static int getLiftPosition(){
        return extendoEncoder.getCurrentPosition();
    }
    public static boolean atTarget() { return (extendoEncoder.getCurrentPosition() >= (getTarget() - tollerence) || extendoEncoder.getCurrentPosition() <= (getTarget() + tollerence)); }
    public static void reset(){extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
    public static boolean extended(){
        return liftTarget > 0;
    }
    @NonNull
    public static Lambda update() {
        return new Lambda("update the extendo")
                .addRequirements(INSTANCE)
                .setExecute(extendo::pidUpdate)
                .setFinish(() -> false);
    }

    @NonNull
    public static Lambda goTo(int target){
        return new Lambda("set pid target")
                .setExecute(() -> setTarget(target))
                .setFinish(extendo::atTarget);
    }

}