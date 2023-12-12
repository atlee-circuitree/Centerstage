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
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        // Function Speeds
        double driveSpeed = .75;
        double armSpeed = .25;
        double intakeSpeed = 1;
        double reverseIntakeSpeed = -1;

        // Function Positions
        double clawOpen = .36;
        double clawClose = .3;
        double collectionPosition = .28;
        double downPosition = .2;
        double upPosition = .9;
        double testServoPosition = .5;
        boolean increased = false;
        boolean decreased = false;

        // Limits
        double topLimit = 1500;
        double bottomLimit = 20;
        double collectionLimit = 200;
        double clawState = 2;
        boolean clawStateChanged = false;

        // PID Values
        double armP = .008;
        double armI = 0;
        double armD = 0;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y * driveSpeed; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * driveSpeed;
            double rx = -gamepad1.right_stick_x * driveSpeed;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //double botHeading = 0;

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

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower); // Drive

            // Other functions

            // Reset Odometry

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad2.right_stick_y > .5) {

                leftSlideMotor.setPower(PIDControl(bottomLimit, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);
                rightSlideMotor.setPower(PIDControl(bottomLimit, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);

            } else if (gamepad2.right_stick_y < -.5) {

                leftSlideMotor.setPower(PIDControl(topLimit, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);
                rightSlideMotor.setPower(PIDControl(topLimit, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);

            } else if (gamepad2.b) {

                leftSlideMotor.setPower(PIDControl(collectionLimit, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);
                rightSlideMotor.setPower(PIDControl(collectionLimit, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);

            } else {

                leftSlideMotor.setPower(0);
                rightSlideMotor.setPower(0);

            }

            // Intake speed
            if (gamepad1.left_trigger > .2) {
                intake.setPower(intakeSpeed);
            } else if (gamepad1.right_trigger > .2) {
                intake.setPower(reverseIntakeSpeed);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.right_trigger > 0.3) {

                // 0 close, 1 open
                if (clawState == 0 && clawStateChanged == false) {

                    clawState = 1;
                    clawStateChanged = true;

                } else if (clawState = 1 && clawStateChanged == false) {

                    clawState = 0;
                    clawStateChanged = true;

                }

            } else {

                clawStateChanged = true;

            }


            if (clawState == 0) {

                claw.setPosition(clawClose);

            } else if (clawState == 1) {

                claw.setPosition(clawOpen);

            }

            // Wrist position
            if (gamepad2.left_stick_y < -.5) {

                wrist.setPosition(upPosition); //

            } else if (gamepad2.left_stick_y > .5) {

                wrist.setPosition(downPosition); // .7

            } else {

                wrist.setPosition(collectionPosition);

            }

        } if (gamepad1.dpad_up);

            telemetry.addData("Test Servo Position", testServoPosition);
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

        double speedLimit = .8;

        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);

        if (output > speedLimit) {

            output = speedLimit;

        }

        if (output < -speedLimit) {

            output = -speedLimit;

        }

        if (output > 0 && state > 2000) {

            output = 0;

        }

        if (output < 0 && state < 20) {

            output = 0;

        }

        return output;

    }

}
