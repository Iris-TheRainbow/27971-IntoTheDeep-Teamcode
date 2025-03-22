package org.firstinspires.ftc.teamcode.pathing;



import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class MecanumParams {

    // IMU orientation
    // TODO: fill in these values based on
    //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // hardware device names (in robot configuration)
    public String lfName = "leftFront";
    public String lbName = "leftBack";
    public String rbName = "rightBack";
    public String rfName = "rightFront";
    public String imuName = "imu";

    // motor directions
    public DcMotorSimple.Direction lfDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction lbDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction rfDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction rbDirection = DcMotorSimple.Direction.FORWARD;


    // drive model parameters
    public double inPerTick = 1;
    public double lateralInPerTick = inPerTick;
    public double trackWidthTicks = 0;

    // feedforward parameters (in tick units)
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;

    // path profile parameters (in inches)
    public double maxWheelVel = 50;
    public double minProfileAccel = -30;
    public double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public double maxAngVel = Math.PI; // shared with path
    public double maxAngAccel = Math.PI;

    // path controller gains
    public double axialGain = 0.0;
    public double lateralGain = 0.0;
    public double headingGain = 0.0; // shared with turn

    public double axialVelGain = 0.0;
    public double lateralVelGain = 0.0;
    public double headingVelGain = 0.0; // shared with turn

    // start pose (to be used in constructor)
    public Pose2d beginPose;

    public MecanumParams(Pose2d beginPose) {
        this.beginPose = beginPose;
    }
}
