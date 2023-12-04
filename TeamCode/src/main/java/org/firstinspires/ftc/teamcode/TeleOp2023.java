package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");

        frontLeftMotor.getCurrentPosition();

        // Reverse the right side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
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
        double collectionPosition = .3;
        double scoringPositiop = .7;

        // Limits
        double topLimit = 1500;
        double bottomLimit = 200;

        // PID Values
        double armP = .008;
        double armI = 0;
        double armD = 0;

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

            if (gamepad1.left_bumper) {

                leftSlideMotor.setPower(PIDControl(100, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);
                rightSlideMotor.setPower(PIDControl(100, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);

            } else if (gamepad1.right_bumper) {

                leftSlideMotor.setPower(PIDControl(1500, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);
                rightSlideMotor.setPower(PIDControl(1500, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);

            } else {

                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);

            }

            // Intake speed
            if (gamepad1.a) {
                intake.setPower(intakeSpeed);
            } else {
                intake.setPower(0);
            }

            // Claw position
            if (gamepad1.y) {
                claw.setPosition(clawOpen);
            } else if (gamepad1.x) {
                claw.setPosition(clawClose);
            }

            // Wrist position
            if (gamepad1.dpad_up) {
                wrist.setPosition(collectionPosition);
            } else if (gamepad1.dpad_down) {
                wrist.setPosition(scoringPositiop);
            }

            telemetry.addData("PID Reading", PIDControl(700, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .2);
            telemetry.addData("Left Bore Reading", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Slide Reading", leftSlideMotor.getCurrentPosition());
            telemetry.addData("Claw Reading", claw.getPosition());
            telemetry.update();

        }

    }

    // Generic PID Variables
    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // Generic PID control for our functions
    public double PIDControl(double reference, double state, double kP, double kI, double kD) {

        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;

    }

}
