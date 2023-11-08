package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Hanz \'s Controller V1.2")
public class HanzsControllerV12 extends LinearOpMode {

  private Servo servo;
  private DcMotor left_drive;
  private IMU imu;
  private DcMotor right_drive;
  private DcMotor arm_motor;
  private VoltageSensor ControlHub_VoltageSensor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    boolean DRIVEMODE;
    int drivePower;
    ElapsedTime elapsedTime;
    double servoTargetPosition;

    servo = hardwareMap.get(Servo.class, "servo");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    imu = hardwareMap.get(IMU.class, "imu");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
    ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

    // Put initialization blocks here.
    DRIVEMODE = true;
    drivePower = 1;
    elapsedTime = new ElapsedTime();
    servoTargetPosition = servo.getPosition();
    left_drive.setDirection(DcMotor.Direction.REVERSE);
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
    waitForStart();
    elapsedTime.reset();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Gear shift up
        if (gamepad1.right_bumper) {
          if (drivePower < 1) {
            drivePower += 0.05;
          }
        }
        // Gear shift down
        if (gamepad1.left_bumper) {
          if (drivePower > 0) {
            drivePower += -0.05;
          }
        }
        // Stop
        if (gamepad1.guide) {
          requestOpModeStop();
        }
        // Robot drive
        right_drive.setPower(gamepad1.left_stick_y * drivePower - gamepad1.left_stick_x * drivePower);
        left_drive.setPower(gamepad1.left_stick_y * drivePower + gamepad1.left_stick_x * drivePower);
        // Arm motor drive
        arm_motor.setPower(gamepad1.right_stick_y);
        servoTargetPosition += 0.05 * gamepad1.right_stick_x;
        servoTargetPosition = Math.min(Math.max(servoTargetPosition, 0), 1);
        servo.setPosition(servoTargetPosition);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addData("<h1>Motor Power", JavaUtil.formatNumber(drivePower * 100, 0) + "%</h1>");
        telemetry.addData("Stick X", Double.parseDouble(JavaUtil.formatNumber(gamepad1.left_stick_x, 3)));
        telemetry.addData("Stick Y", Double.parseDouble(JavaUtil.formatNumber(gamepad1.left_stick_y, 3)));
        telemetry.addData("Left Motor Power", Double.parseDouble(JavaUtil.formatNumber(left_drive.getPower(), 3)));
        telemetry.addData("Right Motor Power", Double.parseDouble(JavaUtil.formatNumber(right_drive.getPower(), 3)));
        telemetry.addData("Servo Position", Double.parseDouble(JavaUtil.formatNumber(servo.getPosition(), 3)));
        telemetry.addData("Pitch", Double.parseDouble(JavaUtil.formatNumber(imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES), 3)));
        telemetry.addData("Roll", Double.parseDouble(JavaUtil.formatNumber(imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES), 3)));
        telemetry.addData("Yaw", Double.parseDouble(JavaUtil.formatNumber(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 3)));
        telemetry.addData("Voltage", Double.parseDouble(JavaUtil.formatNumber(ControlHub_VoltageSensor.getVoltage(), 3)));
        telemetry.addData("Elapsed Time", JavaUtil.formatNumber(elapsedTime.seconds(), 0) + "s");
        telemetry.update();
      }
    }
  }
}
