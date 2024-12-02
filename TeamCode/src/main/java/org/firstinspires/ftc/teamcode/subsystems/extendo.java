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
    private static double kP = .004, kD = .0004, kF = 0;
    private static boolean cancle = false;
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
        extendoMotor = hwmap.get(DcMotorEx.class, "liftLeft");
        extendoEncoder = hwmap.get(DcMotorEx.class, "leftFront");
        setDefaultCommand(update());
        pid = new PDFController(kP, kD, 0);
    }

    public static void setTarget(int target){ liftTarget = target; }

    public static int getTarget(){ return liftTarget; }

    public static void pidUpdate() {
        pid.setSetPoint(liftTarget);
        double power = pid.calculate(liftTarget, extendoEncoder.getCurrentPosition()) + kF;
        extendoMotor.setPower(power);
    }

    public static void hold() {
        extendoMotor.setPower(kF);
    }
    public static boolean atTarget() { return (extendoEncoder.getCurrentPosition() >= (getTarget() - tollerence) || extendoEncoder.getCurrentPosition() <= (getTarget() + tollerence)); }


    @NonNull
    public static Lambda update() {
        return new Lambda("update the pid")
                .addRequirements(INSTANCE)
                .setExecute(extendo::pidUpdate)
                .setFinish(() -> false);
    }

    @NonNull
    public static Lambda goTo(int to){
        return new Lambda("set pid target")
                .setExecute(() -> setTarget(to))
                .setFinish(extendo::atTarget);
    }

}