package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import kotlin.annotation.MustBeDocumented;

public class drive implements Subsystem {
        public static final drive INSTANCE = new drive();
        private static DcMotorEx leftFront, leftBack, rightFront, rightBack;
        private static SparkFunOTOSCorrected otos;

        private drive() { }

        @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
        public @interface Attach { }

        private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

        @NonNull @Override
        public Dependency<?> getDependency() { return dependency; }

        @Override
        public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

        @Override
        public void preUserInitHook(@NonNull Wrapper opMode) { }

        @Override
        public void postUserInitHook(@NonNull Wrapper opMode) {
                HardwareMap hwmap = opMode.getOpMode().hardwareMap;
                leftBack = hwmap.get(DcMotorEx.class, "leftBack");
                leftFront = hwmap.get(DcMotorEx.class, "leftFront");
                rightBack = hwmap.get(DcMotorEx.class, "rightBack");
                rightFront = hwmap.get(DcMotorEx.class, "rightFront");
                otos = hwmap.get(SparkFunOTOSCorrected.class, "otos");
                System.out.println(otos.setLinearScalar(PARAMS.linearScalar));
                System.out.println(otos.setAngularScalar(PARAMS.angularScalar));
                otos.setAngularUnit(AngleUnit.RADIANS);
                System.out.println(otos.calibrateImu(255, false));
        }

        @Override
        public void postUserInitLoopHook(@NonNull Wrapper opMode){ if (otos.getImuCalibrationProgress() == 0){ setDefaultCommand(driveCommand()); } }
        @Override
        public void postUserLoopHook(@NonNull Wrapper opMode){ if (otos.getImuCalibrationProgress() == 0){ setDefaultCommand(driveCommand()); } }

        public static void driveUpdate() {
                // read the gamepads
                double rightX = Mercurial.gamepad1().leftStickX().state();
                double rightY = -Mercurial.gamepad1().leftStickY().state();
                double turn = Mercurial.gamepad1().rightStickX().state();

                double heading = otos.getPosition().h;
                // Do the kinematics math
                double rotX = (rightX * Math.cos(-heading) - rightY * Math.sin(-heading)) * 1.1;
                double rotY = rightX * Math.sin(-heading) + rightY * Math.cos(-heading);
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
                double lfPower = (rotY + rotX + turn) / denominator;
                double lbPower = (rotY - rotX + turn) / denominator;
                double rfPower = (rotY - rotX - turn) / denominator;
                double rbPower = (rotY + rotX - turn) / denominator;
                // Set the Motor Power
                leftBack.setPower(lbPower);
                leftFront.setPower(lfPower);
                rightBack.setPower(rbPower);
                rightFront.setPower(rfPower);
        }

        public static void resetHeading(){
                otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        }

        @NonNull
        public static Lambda driveCommand() {
            return new Lambda("driveCommand")
                    .addRequirements(INSTANCE)
                    .setExecute(drive::driveUpdate);
        }

        @NonNull
        public static Lambda zeroHeading() {
                return new Lambda("zeroHeading")
                        .addRequirements(INSTANCE)
                        .setInit(drive::resetHeading);
        }
    }