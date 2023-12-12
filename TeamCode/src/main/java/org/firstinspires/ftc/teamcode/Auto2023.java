/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous (name="Auto 1", group="Linear OpMode")

public class Auto2023 extends LinearOpMode {

    //comment

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor rightSlideMotor;
    DcMotor leftSlideMotor;
    CRServo intake;
    Servo claw;
    Servo wrist;

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

    // PID Values
    double armP = .008;
    double armI = 0;
    double armD = 0;
    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");
        leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        intake = hardwareMap.crservo.get("intake");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Auto Starts
            // Auto Ends
            break;

        }

    }

    // Basic driving function
    public void drive(double speed) {

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

    }

    // Basic turning function
    public void turn(double speed) {

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);

    }

    // Basic strafing function
    public void strafe(double speed) {

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);

    }

    public void driveForwardUsingEncoders(double inches, double speed, double timeout) {

        // Get the inital encoder reading
        double initalTime = runtime.time();
        double initalReading = -backRightMotor.getCurrentPosition();
        double target = inches * 336.8786366;

        while (-backRightMotor.getCurrentPosition() < target + initalReading && runtime.time() < initalTime + timeout) { //and

            drive(-speed);

            telemetry.addData("Running to : ", inches + initalReading);
            telemetry.addData("Currectly at : ", backRightMotor.getCurrentPosition());
            telemetry.update();

        }

        // Kill the motors
        drive(0);

    }

    public void driveBackwardsUsingEncoders(double inches, double speed, double timeout) {

        // Get the inital encoder reading
        double initalTime = runtime.time();
        double initalReading = -backRightMotor.getCurrentPosition();
        double target = inches * 336.8786366;

        while (-backRightMotor.getCurrentPosition() > target - initalReading && runtime.time() < initalTime + timeout) { //and

            drive(speed);

            telemetry.addData("Running to : ", inches + initalReading);
            telemetry.addData("Currectly at : ", backRightMotor.getCurrentPosition());
            telemetry.update();

        }

        // Kill the motors
        drive(0);

    }


    public void strafeRightUsingEncoders(double inches, double speed, double timeout) {

        // Get the inital encoder reading
        double initalTime = runtime.time();
        double initalReading = -frontRightMotor.getCurrentPosition();
        double target = inches * 336.8786366;

        while (-frontRightMotor.getCurrentPosition() < target + initalReading && runtime.time() < initalTime + timeout) { //and

            strafe(-speed);

            telemetry.addData("Running to : ", inches + initalReading);
            telemetry.addData("Currectly at : ", -frontRightMotor.getCurrentPosition());
            telemetry.update();

        }

        // Kill the motors
        drive(0);

    }

    public void strafeLeftUsingEncoders(double inches, double speed, double timeout) {

        // Get the inital encoder reading
        double initalTime = runtime.time();
        double initalReading = -frontRightMotor.getCurrentPosition();
        double target = inches * 336.8786366;

        while (-frontRightMotor.getCurrentPosition() > target - initalReading && runtime.time() < initalTime + timeout) { //and

            strafe(speed);

            telemetry.addData("Running to : ", inches + initalReading);
            telemetry.addData("Currectly at : ", -frontRightMotor.getCurrentPosition());
            telemetry.update();

        }

        // Kill the motors
        drive(0);

    }

    public void controlClaw(double clawPosition, double timeout) {

        double initalTime = runtime.time();
        double time = timeout;

        while (runtime.time() < initalTime + timeout) {

            claw.setPosition(clawPosition);

        }

    }

    public void controlWrist(double wristPosition, double timeout) {

        double initalTime = runtime.time();
        double time = timeout;

        while (runtime.time() < initalTime + timeout) {

            wrist.setPosition(wristPosition);

        }

    }

    public void controlArm(double armPosition, double timeout) {

        double initalTime = runtime.time();
        double time = timeout;

        while (runtime.time() < initalTime + timeout) {

            leftSlideMotor.setPower(PIDControl(armPosition, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);
            rightSlideMotor.setPower(PIDControl(armPosition, leftSlideMotor.getCurrentPosition(), armP, armI, armD) * .5);

        }

        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);

    }

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