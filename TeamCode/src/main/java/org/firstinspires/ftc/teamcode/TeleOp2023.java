package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp2023 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors--
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");
        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        CRServo intake = hardwareMap.crservo.get("intake");
        //Servo intake = hardwareMap.servo.get("intake");
        Servo claw = hardwareMap.servo.get("claw"); //claw
        Servo wrist = hardwareMap.servo.get("wrist");

        frontLeftMotor.getCurrentPosition();

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        // Function Speeds
        double driveSpeed = .5;
        double armSpeed = .25;
        double intakeSpeed = .8;

        // Function Positions
        double clawOpen = .5;
        double clawClose = .3;

        // Limits
        double topLimit = 1500;
        double bottomLimit = 200;

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y * driveSpeed; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * driveSpeed;
            double rx = gamepad1.right_stick_x * driveSpeed;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            //frontLeftMotor.setPower(frontLeftPower);
            //backLeftMotor.setPower(backLeftPower);
            //frontRightMotor.setPower(frontRightPower);
            //rcbackRightMotor.setPower(backRightPower); // Drive

            // Other functions

            // Reset Odometry
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Moves the arm up and down
            if (gamepad1.left_bumper && leftSlideMotor.getCurrentPosition() < topLimit)  {
                leftSlideMotor.setPower(armSpeed);
                rightSlideMotor.setPower(armSpeed);
            }  else if (gamepad1.right_bumper && leftSlideMotor.getCurrentPosition() > bottomLimit) {
                leftSlideMotor.setPower(-armSpeed);
                rightSlideMotor.setPower(-armSpeed);
            } else {
                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);
            }

            // Intake speed
            if (gamepad1.a) {
                intake.setPower(intakeSpeed);
            } else {
                intake.setPower(0);
                //Continuous Servo speed
            }

            // Claw position
            if (gamepad1.y) {
                claw.setPosition(clawOpen);
            } else if (gamepad1.x) {
                claw.setPosition(clawClose);
            }

            telemetry.addData("Left Bore Reading", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Slide Reading", leftSlideMotor.getCurrentPosition());
            telemetry.addData("Claw Reading", claw.getPosition());
            telemetry.update();

        }

    }

}
