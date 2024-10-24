package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;

@Autonomous(name = "auto")
@Mercurial.Attach
@BulkRead.Attach
public class Auto extends OpMode {
    private final Pose2d initialPose = new Pose2d(1, 1, Math.toRadians(90));
    private Action path;
    private SparkFunOTOSDrive drive;
    private Gamepad oldGamepad;
    private boolean left, right, red, blue, allow = true;

    @Override
    public void init() {
         drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
         oldGamepad = gamepad1;
    }
    @Override
    public void init_loop(){
        if (allow) {
            telemetry.addLine("A: confirm");
            telemetry.addLine("B: red");
            telemetry.addLine("X: blue");
            telemetry.addLine("Left: Basket Side");
            telemetry.addLine("Right: hp side");
            if (gamepad1.b) { red = true; red = false; }
            if (gamepad1.x) { blue = true; red = false; }
            if (gamepad1.dpad_left) { left = true; right = false; }
            if (gamepad1.dpad_right) { right = true; left = false; }
        }
        if (red) { telemetry.addLine("RED"); }
        if (blue) { telemetry.addLine("BLUE");}
        if (left) { telemetry.addLine("LEFT"); }
        if (right) { telemetry.addLine("RIGHT");}
        if (oldGamepad.a != gamepad1.a && gamepad1.a) {
            allow = false;
            TrajectoryActionBuilder pathBuilder = drive.actionBuilder(initialPose)
                    .splineTo(new Vector2d(6, -36), Math.toRadians(90))
                    .waitSeconds(5)
                    .setReversed(true)
                    .splineTo(new Vector2d(40, -62), Math.toRadians(-90));
            path = pathBuilder.build();
        }
    }
    @Override
    public void loop() {
        Actions.runBlocking(
                new SequentialAction(
                        path
                )
        );
    }
}
