package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.controllers.SquIDSLController;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@TeleOp
public class SquIDSLTuner extends OpMode {
    private DcMotorEx liftLeft, liftLeft2, liftRight, liftEncoder;
    public static double kP, kD, kS;
    private SquIDSLController squid;
    public static int liftTarget;
    public MultipleTelemetry telem;
    @Override
    public void init() {
        liftLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftLeft"));
        liftLeft2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftLeft2"));
        liftRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftRight"));
        liftEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        squid = new SquIDSLController(kP, kD, kS);
        telem = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void loop() {
        squid.setConstants(kP, kD, kS);
        double power = squid.calculate(liftTarget, -liftEncoder.getCurrentPosition());
        liftRight.setPower(power);
        liftLeft.setPower(power);
        liftLeft2.setPower(power);
        telem.addData("target", liftTarget);
        telem.addData("Actual", liftEncoder.getCurrentPosition());
        telem.addData("power", power);

    }
}
