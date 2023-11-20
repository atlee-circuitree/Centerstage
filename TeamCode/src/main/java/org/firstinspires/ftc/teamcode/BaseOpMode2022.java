package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class BaseOpMode2022 extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor frontLeft = null;
    public DcMotor rearLeft = null;
    public DcMotor frontRight = null;
    public DcMotor rearRight = null;

    public int testModeV = 0;

    public enum AutoSide {

        REDLEFT,
        REDRIGHT,
        BLUELEFT,
        BLUERIGHT

    }

    public static double timer;

    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    int loop = 0;

    //turn motor at 200 ticks per second
    public double motorVelocity = 200;

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: GOBUILDA Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE = 1;

    static final double OMNI_COUNTS_PER_REV = 8192; //For rev through bore encoder (This is the correct number)
    static final double OMNI_WHEEL_DIAMETER = 3.77953;
    public static final double OMNI_COUNTS_PER_INCH = (OMNI_COUNTS_PER_REV) / (OMNI_WHEEL_DIAMETER * Math.PI);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    //Initializes hardware
    public void GetHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        GetIMU();
        //Motor and Servo Variables
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        SetDriveMode(Mode.STOP_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SetDriveMode(Mode.RUN_WITH_ENCODER);
        SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);
    }

    public void GetIMU() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("IMU", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        // while (!isStopRequested() && !imu.isGyroCalibrated()) {
        //    sleep(50);
        //    idle();
        // }
        telemetry.addData("IMU", "calibrated");
        telemetry.update();
    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360 * 2;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;

    }

    public double cmHorizAngleArm(double cm) { //ticks from horiz and angle arm coverted to cm

        final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: goBilda motor encoder
        final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
        final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        return COUNTS_PER_CM * cm;

    }

    public double cmVertARm(double cm) { //ticks from vert arm coverted to cm

        final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: goBilda motor encoder
        final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        return COUNTS_PER_INCH * cm;

    }

    // Below code from 2022-2023, left in as an example

    /*
    public void horizArmPIDLoop(int posTarget) {
        horizController.setPID(hP, hI, hD);
        int horizArmPos = horizArm.getCurrentPosition();
        double pid = horizController.calculate((horizArmPos), posTarget);
        // double ff = Math.cos(Math.toRadians(posTarget / horizArmTicksPerRev)) * hF;

        double horizArmPower = pid + hF;


        horizArm.setPower(horizArmPower);


        telemetry.addData("Horiz Arm Pos", horizArmPos);
        telemetry.addData("Horiz Arm Target", posTarget);
    }

    //posTarget is distance you want to put in like the value of hArmExtend
    //You would put in vArmTopHigh in posTarget for example when using this function in a tele-op or autonomous
    public void vertArmPIDLoop(int posTarget) {
        vertController.setPID(vP, vI, vD);
        int vertArmPos = vertArm.getCurrentPosition();
        double pid = vertController.calculate((vertArmPos), posTarget);
        // double ff = Math.cos(Math.toRadians(posTarget / vertArmTicksPerRev)) * vF;

        double vertArmPower = pid + vG;

        vertArm.setPower(vertArmPower);

        telemetry.addData("Vert Arm Pos", vertArmPos);
        telemetry.addData("Vert Arm Target", posTarget);
    }

    //posTarget is distance you want to put in like the value of hArmExtend
    //You would put in aArmCone1 in posTarget for example when using this function in a tele-op or autonomous
    public void angleArmPIDLoop(int posTarget) {
        angleController.setPID(aP, aI, aD);
        int angleArmPos = angleArm.getCurrentPosition();
        double pid = angleController.calculate((angleArmPos), posTarget);
        double ff = Math.cos(Math.toRadians(posTarget / angleArmTicksPerRev)) * aF;

        double angleArmPower = pid + ff;

        angleArm.setPower(angleArmPower);

        telemetry.addData("Angle Arm Pos", angleArmPos);
        telemetry.addData("Angle Arm Target", posTarget);
    }

    //For testing in teleop, don't use for matches
    public void horizArmPIDLoopTeleOp() {
        horizController.setPID(hP, hI, hD);
        int horizArmPos = horizArm.getCurrentPosition();
        double pid = horizController.calculate((horizArmPos), horizArmPIDTarget);
        // double ff = Math.cos(Math.toRadians(horizArmPIDTarget / horizArmTicksPerRev)) * hF;

        double horizArmPower = pid + hF;

        horizArm.setPower(horizArmPower);

        telemetry.addData("Horiz Arm Pos", horizArmPos);
        telemetry.addData("Horiz Arm Target", horizArmPIDTarget);
    }

    public void vertArmPIDLoopTeleOp() {
        vertController.setPID(vP, vI, vD);
        int vertArmPos = vertArm.getCurrentPosition();
        double pid = vertController.calculate((vertArmPos), vertArmPIDTarget);
        //double ff = Math.cos(Math.toRadians(vertArmPIDTarget / vertArmTicksPerRev)) * vF;

        double vertArmPower = pid + vG;

        vertArm.setPower(vertArmPower);

        telemetry.addData("Vert Arm Pos", vertArmPos);
        telemetry.addData("Vert Arm Target", vertArmPIDTarget);
    }

    public void angleArmPIDLoopTeleOp() {
        angleController.setPID(aP, aI, aD);
        int angleArmPos = angleArm.getCurrentPosition();
        double pid = angleController.calculate((angleArmPos), angleArmPIDTarget);
        double ff = Math.cos(Math.toRadians(angleArmPIDTarget / angleArmTicksPerRev)) * aF;

        double angleArmPower = pid + ff;

        angleArm.setPower(angleArmPower);

        telemetry.addData("Angle Arm Pos", angleArmPos);
        telemetry.addData("Angle Arm Target", angleArmPIDTarget);
    }

    public int horizArmMech(int horizArmState, int horizArmTarget, int ENCODER_ERROR_THRESHOLD) {
        if (horizArmState == HORIZ_ARM_EXTENDING) {
            if (Math.abs(horizArm.getCurrentPosition() - horizArmTarget) <= ENCODER_ERROR_THRESHOLD) {
                horizArmState = HORIZ_ARM_EXTENDED;
            } else {
                horizArmPIDLoop(horizArmTarget);
            }
        }
        if (horizArmState == HORIZ_ARM_RETRACTING) {
            if (Math.abs(horizArm.getCurrentPosition() - horizArmTarget) <= ENCODER_ERROR_THRESHOLD) {
                horizArmState = HORIZ_ARM_RETRACTED;
                horizArm.setPower(0);
            } else {
                horizArmPIDLoop(horizArmTarget);
            }
        }
        if (horizArmState == HORIZ_ARM_STOP) {
            horizArm.setPower(0);
        }
        return horizArmState;
    }

    public int vertArmMech(int vertArmState, int vertArmTarget, int ENCODER_ERROR_THRESHOLD) {
        if (vertArmState == VERT_ARM_EXTENDING) {
            if (Math.abs(vertArm.getCurrentPosition() - vertArmTarget) <= ENCODER_ERROR_THRESHOLD) {
                vertArmState = VERT_ARM_EXTENDED;
            } else {
                vertArmPIDLoop(vertArmTarget);
            }
        }
        if (vertArmState == VERT_ARM_RETRACTING) {
            if (Math.abs(vertArm.getCurrentPosition() - vertArmTarget) <= ENCODER_ERROR_THRESHOLD) {
                vertArmState = VERT_ARM_RETRACTED;
                horizArm.setPower(0);
            } else {
                vertArmPIDLoop(vertArmTarget);
            }
        }
        if (vertArmState == VERT_ARM_STOP) {
            vertArm.setPower(0);
        }
        return vertArmState;
    }

    public int angleArmMech(int angleArmState, int angleArmTarget, int ENCODER_ERROR_THRESHOLD) {
        if (angleArmState == ANGLE_ARM_EXTENDING) {
            if (Math.abs(angleArm.getCurrentPosition() - (angleArmTarget + angleArmOffset)) <= ENCODER_ERROR_THRESHOLD) {
                angleArmState = ANGLE_ARM_EXTENDED;
            } else {
                angleArmPIDLoop(angleArmTarget);
            }
        }
        if (angleArmState == ANGLE_ARM_RETRACTING) {
            if (Math.abs(angleArm.getCurrentPosition() - (angleArmTarget + angleArmOffset)) <= ENCODER_ERROR_THRESHOLD) {
                angleArmState = ANGLE_ARM_RETRACTED;
            } else {
                angleArmPIDLoop(angleArmTarget);
            }
        }
        if (angleArmState == ANGLE_ARM_STOP) {
            angleArm.setPower(0);
        }
        return angleArmState;
    }

    public void getGyro() {

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }
    }
    */

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int FLTarget;
        int FRTarget;
        int RLTarget;
        int RRTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double FLSpeed;
        double RLSpeed;
        double FRSpeed;
        double RRSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);

            FLTarget = frontLeft.getCurrentPosition() + moveCounts;
            RLTarget = rearLeft.getCurrentPosition() + moveCounts;
            FRTarget = frontRight.getCurrentPosition() + moveCounts;
            RRTarget = rearRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontLeft.setTargetPosition(FLTarget);
            frontRight.setTargetPosition(FRTarget);
            rearLeft.setTargetPosition(RLTarget);
            rearRight.setTargetPosition(RRTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            rearLeft.setPower(speed);
            rearRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                FLSpeed = speed - steer;
                RLSpeed = speed - steer;
                FRSpeed = speed + steer;
                RRSpeed = speed + steer;

                // max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));


                // Normalize speeds if either one exceeds +/- 1.0;

                // max = Math.max(Math.abs(FLSpeed), Math.abs(FRSpeed), Math.max(Math.abs(RRSpeed), Math.abs(RLSpeed));

                //max = Math.max(Math.abs(FLSpeed), (RLSpeed), (Math.abs(FRSpeed), (RRSpeed));

                // max = Math.max(Math.abs(FLSpeed), Math.abs(FRSpeed));
                max = Math.max(Math.abs(RRSpeed), Math.abs(RLSpeed));

                if (max > 1.0) {
                    FLSpeed /= max;
                    FRSpeed /= max;
                    RRSpeed /= max;
                    RLSpeed /= max;
                }
                frontLeft.setPower(FLSpeed);
                frontRight.setPower(FRSpeed);
                rearLeft.setPower(RLSpeed);
                rearRight.setPower(RRSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", FLTarget, FRTarget, RLTarget, RRTarget);
                telemetry.addData("Actual", "%7d:%7d", frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", FLSpeed, FRSpeed, RLSpeed, RRSpeed);
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double FLSpeed;
        double RLSpeed;
        double FRSpeed;
        double RRSpeed;
        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            FLSpeed = 0.0;
            RLSpeed = 0.0;
            FRSpeed = 0.0;
            RRSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            FRSpeed = speed * steer;
            RRSpeed = speed * steer;
            FLSpeed = -FRSpeed;
            RLSpeed = -RRSpeed;

        }

        // Send desired speeds to motors.
        frontLeft.setPower(FLSpeed);
        frontRight.setPower(FRSpeed);
        rearLeft.setPower(RLSpeed);
        rearRight.setPower(RRSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", FLSpeed, FRSpeed, RLSpeed, RRSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public enum STRAFE {
        LEFT, RIGHT
    }


    public enum Mode {
        STOP_RESET_ENCODER,
        RUN_WITH_ENCODER,
        RUN_WITHOUT_ENCODERS,
    }

    public enum Drive {
        STOP
    }

    public enum Shoot {
        SHOOT_FAR,
        GET_VELOCITY,
    }

    public void DriveTrain(Drive Stop) {
        if (Stop == Drive.STOP) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
        }
    }

    public void encoderLift(double speed, double timeoutS) {
        int newLiftTarget;
    }

    public void SetDriveMode(Mode DriveMode) {

        if (DriveMode == Mode.STOP_RESET_ENCODER) {

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        if (DriveMode == Mode.RUN_WITH_ENCODER) {

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (DriveMode == Mode.RUN_WITHOUT_ENCODERS) {

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    public void zeroGyro() {
        // navx_centered.zeroYaw();
        GetIMU();
    }

    /*public void Conesensor(double centimeters) {
        while (FrontColor.getDistance(DistanceUnit.CM) > centimeters) {
            frontLeft.setPower(.1);
            rearLeft.setPower(.1);
            frontRight.setPower(.1);
            rearRight.setPower(.1);
        }
            frontLeft.setPower(0);
            rearLeft.setPower(0);
            frontRight.setPower(0);
            rearRight.setPower(0);
    }*/

    //Code from 2022-2023, left as an example

    /*
    public void checkBehaviors() {

        //Check States
        //if (horizArmState == HORIZ_ARM_RETRACTING) {
        //    horizArmState = horizArmMech(horizArmState, hArmRetract, ENCODER_ERROR_THRESHOLD);
        //}
        //if (horizArmState == HORIZ_ARM_EXTENDING) {
        //    horizArmState = horizArmMech(horizArmState, vArmTarget, ENCODER_ERROR_THRESHOLD);
        //}
        //if (vertArmState == VERT_ARM_RETRACTING) {
        //    vertArmState = vertArmMech(vertArmState, vArmTarget, ENCODER_ERROR_THRESHOLD);
        //}
        //if (vertArmState == VERT_ARM_EXTENDING) {
        //    vertArmState = vertArmMech(vertArmState, vArmHigh, ENCODER_ERROR_THRESHOLD);
        // }
        //Behaviors
        if (behavior == BEHAVIOR_TRANSFER_CONE) {
            //lower vert to cone
            //close transfer claw
            //open horiz claw
            //wait till servos finished
            //raise vert
            //wait til vArmLow finished
            //raise vert
            //flip transfer arm and wrist
            //lower onto pole
            if (behaviorStep == 1) {
                transferClaw.setPosition(TRANSFER_CLAW_OPEN);
                vertArmState = vertArmMech(VERT_ARM_RETRACTING, vArmPickup, ENCODER_ERROR_THRESHOLD);
                if (vertArmState == VERT_ARM_RETRACTED) {
                    behaviorStep = 2;
                }
            }
            if (behaviorStep == 2) {
                transferClaw.setPosition(TRANSFER_CLAW_CLOSE);
                horizClaw.setPosition(HORIZONTAL_CLAW_HALF_CLOSE);
                // wait(.1); //Don't be evil
                horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetractFully, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_RETRACTED) {
                    behaviorStep = 3;
                }
            }
            if (behaviorStep == 3) {
                vertArmState = vertArmMech(VERT_ARM_EXTENDING, vArmHigh, ENCODER_ERROR_THRESHOLD);
                if (vertArm.getCurrentPosition() > vArmLow) {
                    behaviorStep = 4;
                }
            }
            if (behaviorStep == 4) {
                vertArmState = vertArmMech(VERT_ARM_EXTENDING, vArmHigh, ENCODER_ERROR_THRESHOLD);
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_BACK);
                transferArmTop.setPosition(TRANSFER_ARM_TOP_BACK);
                if (vertArmState == VERT_ARM_EXTENDED) {
                    behavior = BEHAVIOR_FINISHED;
                    behaviorStep = 1;
                }
            }
        }

        if (behavior == BEHAVIOR_TRANSFER_RETURN) {
            //Move vertArm down on high pole
            //Open transfer claw
            //Move vertArm max height
            //Move transferArm and wrist to front
            //Move vertArm to low pole/pickup level
            if (behaviorStep == 1) {  // ArmHigh - vArmPoleInsert seems to mess up going to pickup position
                // vertArmState = vertArmMech(VERT_ARM_RETRACTING, vArmHigh, ENCODER_ERROR_THRESHOLD);
                // if (vertArmState == VERT_ARM_RETRACTED) {
                behaviorStep = 2;
                // }
            }
            if (behaviorStep == 2) {
                transferClaw.setPosition(TRANSFER_CLAW_OPEN);
                horizClaw.setPosition(HORIZONTAL_CLAW_HALF_CLOSE);
                horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                vertArmState = vertArmMech(VERT_ARM_EXTENDING, vArmHigh, ENCODER_ERROR_THRESHOLD);

                // wait(.1); //Don't be evil
                behaviorStep = 3;
            }

            if (behaviorStep == 3) {
                vertArmState = vertArmMech(VERT_ARM_EXTENDING, vArmHigh, ENCODER_ERROR_THRESHOLD);
                if (vertArm.getCurrentPosition() > vArmPoleSafe) {
                    behaviorStep = 4;
                }
            }

            if (behaviorStep == 4) {
                vertArmState = vertArmMech(VERT_ARM_RETRACTING, vArmLow, ENCODER_ERROR_THRESHOLD);
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_FRONT);
                transferArmTop.setPosition(TRANSFER_ARM_TOP_FRONT);
                if (vertArmState == VERT_ARM_RETRACTED) {
                    behavior = BEHAVIOR_FINISHED;
                    behaviorStep = 1;
                }
            }
        }
        if (behavior == BEHAVIOR_EXTEND_HORIZ_ARM_TO_MAX) {
            horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
            if (horizArmState == HORIZ_ARM_EXTENDED) {
                behavior = BEHAVIOR_FINISHED;
                // behaviorStep = 1;
            }

        }

        if (behavior == BEHAVIOR_RETRACT_HORIZ_ARM_TO_MAX) {
            horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
            if (horizArmState == HORIZ_ARM_RETRACTED) {
                behavior = BEHAVIOR_FINISHED;
                //  behaviorStep = 1;
            }
        }


        if (behavior == BEHAVIOR_EXTEND_VERT_ARM_TO_MAX) {
            vertArmState = vertArmMech(VERT_ARM_EXTENDING, vArmHigh, ENCODER_ERROR_THRESHOLD);
            if (vertArmState == VERT_ARM_EXTENDED) {
                behavior = BEHAVIOR_FINISHED;
                //  behaviorStep = 1;
            }
        }

        if (behavior == BEHAVIOR_RETRACT_VERT_ARM_TO_MAX) {
            vertArmState = vertArmMech(VERT_ARM_RETRACTING, vArmPickup, ENCODER_ERROR_THRESHOLD);
            if (vertArmState == VERT_ARM_EXTENDED) {
                behavior = BEHAVIOR_FINISHED;
                //   behaviorStep = 1;
            }
        }


        if (behavior == BEHAVIOR_GET_CONE1) {
            if (behaviorStep == 1) {
                horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeFlat, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_EXTENDED && angleArmState == ANGLE_ARM_EXTENDED) {
                    timer = runtime.milliseconds();
                    behaviorStep = 2;
                }
            }
            if (behaviorStep == 2) {
                horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                //Don't be evil
                if (runtime.milliseconds() - timer >= WAIT_FOR_CLAW) {
                    behaviorStep = 3;
                }

            }
            if (behaviorStep == 3) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeRaisedSlightly, ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 4;
                }
            }
            if (behaviorStep == 4) {
                horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_RETRACTED) {
                    behaviorStep = 5;
                }
            }
            if (behaviorStep == 5) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeGround, ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 6;
                }
            }
            if (behaviorStep == 6) {
                //horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                behavior = BEHAVIOR_FINISHED;
                behaviorStep = 1;
            }
        }

        if (behavior == BEHAVIOR_GET_CONE2) {
            if (behaviorStep == 1) {
                horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmCone2, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_EXTENDED && angleArmState == ANGLE_ARM_EXTENDED) {
                    timer = runtime.milliseconds();
                    behaviorStep = 2;
                }
            }
            if (behaviorStep == 2) {
                horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                //Don't be evil
                if (runtime.milliseconds() - timer >= WAIT_FOR_CLAW) {
                    behaviorStep = 3;
                }

            }
            if (behaviorStep == 3) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, (aArmCone2 + aArmConeLift), ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 4;
                }
            }
            if (behaviorStep == 4) {
                horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_RETRACTED) {
                    behaviorStep = 5;
                }
            }
            if (behaviorStep == 5) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeGround, ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 6;
                }
            }
            if (behaviorStep == 6) {
                //horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                behavior = BEHAVIOR_FINISHED;
                behaviorStep = 1;
            }
        }


        if (behavior == BEHAVIOR_GET_CONE3) {
            if (behaviorStep == 1) {
                horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmCone3, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_EXTENDED && angleArmState == ANGLE_ARM_EXTENDED) {
                    timer = runtime.milliseconds();
                    behaviorStep = 2;
                }
            }
            if (behaviorStep == 2) {
                horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                //Don't be evil
                if (runtime.milliseconds() - timer >= WAIT_FOR_CLAW) {
                    behaviorStep = 3;
                }

            }
            if (behaviorStep == 3) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, (aArmCone3 + aArmConeLift), ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 4;
                }
            }
            if (behaviorStep == 4) {
                horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_RETRACTED) {
                    behaviorStep = 5;
                }
            }
            if (behaviorStep == 5) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeGround, ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 6;
                }
            }
            if (behaviorStep == 6) {
                //horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                behavior = BEHAVIOR_FINISHED;
                behaviorStep = 1;
            }
        }


        if (behavior == BEHAVIOR_GET_CONE4) {
            if (behaviorStep == 1) {
                horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmCone4, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_EXTENDED && angleArmState == ANGLE_ARM_EXTENDED) {
                    timer = runtime.milliseconds();
                    behaviorStep = 2;
                }
            }
            if (behaviorStep == 2) {
                horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                //Don't be evil
                if (runtime.milliseconds() - timer >= WAIT_FOR_CLAW) {
                    behaviorStep = 3;
                }

            }
            if (behaviorStep == 3) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, (aArmCone4 + aArmConeLift), ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 4;
                }
            }
            if (behaviorStep == 4) {
                horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_RETRACTED) {
                    behaviorStep = 5;
                }
            }
            if (behaviorStep == 5) {
                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeGround, ENCODER_ERROR_THRESHOLD);
                if (angleArmState == ANGLE_ARM_EXTENDED) {
                    behaviorStep = 6;
                }
            }
            if (behaviorStep == 6) {
                //horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                behavior = BEHAVIOR_FINISHED;
                behaviorStep = 1;
            }
        }

        if (behavior == BEHAVIOR_GET_CONE5) {
            if (behaviorStep == 1) {

                angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmCone5, ENCODER_ERROR_THRESHOLD);
                if (runtime.milliseconds() - timer >= WAIT_FOR_ARM)
                    horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                if (horizArmState == HORIZ_ARM_EXTENDED && angleArmState == ANGLE_ARM_EXTENDED) {
                    timer = runtime.milliseconds();
                    behaviorStep = 2;

                    if (behaviorStep == 2) {
                        horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                        //Don't be evil
                        behaviorStep = 3;
                    }

                }
                if (behaviorStep == 3) {
                    angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmabovestack, ENCODER_ERROR_THRESHOLD);
                    if (angleArmState == ANGLE_ARM_EXTENDED) {
                        behaviorStep = 4;
                    }
                }
                if (behaviorStep == 4) {
                    horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
                    if (horizArmState == HORIZ_ARM_RETRACTED) {
                        behaviorStep = 5;
                    }
                }
                if (behaviorStep == 5) {
                    angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeGround, ENCODER_ERROR_THRESHOLD);
                    if (angleArmState == ANGLE_ARM_EXTENDED) {
                        behaviorStep = 6;
                    }
                }
                if (behaviorStep == 6) {
                    //horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                    behavior = BEHAVIOR_FINISHED;
                    behaviorStep = 1;
                }
            }
            if (behavior == BEHAVIOR_GET_CONE5) {
                if (behaviorStep == 1) {
                    horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                    angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmCone5, ENCODER_ERROR_THRESHOLD);
                    if (horizArmState == HORIZ_ARM_EXTENDED && angleArmState == ANGLE_ARM_EXTENDED) {
                        horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                        timer = runtime.milliseconds();
                        behaviorStep = 2;
                    }
                }
                if (behaviorStep == 2) {
                    angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeLift, ENCODER_ERROR_THRESHOLD);
                    if (angleArmState == ANGLE_ARM_EXTENDED) {
                        horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
                        angleArmState = angleArmMech(ANGLE_ARM_RETRACTING, aArmConeFlat, ENCODER_ERROR_THRESHOLD);
                        if (horizArmState == HORIZ_ARM_RETRACTED && angleArmState == ANGLE_ARM_RETRACTED) {
                            horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                            if (runtime.milliseconds() - timer >= WAIT_FOR_CLAW) {
                                behavior = BEHAVIOR_FINISHED;
                                behaviorStep = 1;
                            }
                        }
                    }

                    horizArmState = horizArmMech(HORIZ_ARM_EXTENDING, hArmExtend, ENCODER_ERROR_THRESHOLD);
                    angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmCone5, ENCODER_ERROR_THRESHOLD);
                    if (horizArmState == HORIZ_ARM_EXTENDED && angleArmState == ANGLE_ARM_EXTENDED) {
                        timer = runtime.milliseconds();
                        behaviorStep = 2;
                    }
                }
                if (behaviorStep == 2) {
                    horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                    //Don't be evil
                    if (runtime.milliseconds() - timer >= WAIT_FOR_CLAW) {
                        behaviorStep = 3;
                    }

                }
                if (behaviorStep == 3) {
                    angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, (aArmCone5 + aArmConeLift), ENCODER_ERROR_THRESHOLD);
                    if (angleArmState == ANGLE_ARM_EXTENDED) {
                        behaviorStep = 4;

                    }
                }
                if (behaviorStep == 4) {
                    horizArmState = horizArmMech(HORIZ_ARM_RETRACTING, hArmRetract, ENCODER_ERROR_THRESHOLD);
                    if (horizArmState == HORIZ_ARM_RETRACTED) {
                        behaviorStep = 5;
                    }
                }
                if (behaviorStep == 5) {
                    angleArmState = angleArmMech(ANGLE_ARM_EXTENDING, aArmConeGround, ENCODER_ERROR_THRESHOLD);
                    if (angleArmState == ANGLE_ARM_EXTENDED) {
                        behaviorStep = 6;
                    }
                }
                if (behaviorStep == 6) {
                    //horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
                    behavior = BEHAVIOR_FINISHED;
                    behaviorStep = 1;
                }
            }
        }
        if (behavior == VERT_ARM_CONE) {
            if (behaviorStep == 1) {
                transferClaw.setPosition(TRANSFER_CLAW_CLOSE);
                vertArmState = vertArmMech(VERT_ARM_EXTENDING, vArmHigh, ENCODER_ERROR_THRESHOLD);
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_BACK);
                transferArmTop.setPosition(TRANSFER_ARM_TOP_BACK);
                if (vertArmState == VERT_ARM_EXTENDED) {
                    behaviorStep = 2;
                }
            }
            if (behaviorStep == 2) {
                vertArmState = vertArmMech(VERT_ARM_RETRACTING, vArmMid, ENCODER_ERROR_THRESHOLD);
                if (vertArmState == VERT_ARM_RETRACTING) {
                    transferClaw.setPosition(TRANSFER_CLAW_OPEN);
                    if (vertArmState == VERT_ARM_RETRACTING && transferClawState == TRANSFER_CLAW_OPEN) {
                        behaviorStep = 3;
                    }
                }

            }
            if (behaviorStep == 3) {
                transferArmTop.setPosition(TRANSFER_ARM_TOP_FRONT);
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_FRONT);
                if (transferArmTopState == TRANSFER_ARM_TOP_FRONT && transferArmBottomState == TRANSFER_ARM_BOTTOM_FRONT) {
                    behaviorStep = 4;
                }
            }
            if (behaviorStep == 4) {
                vertArmState = vertArmMech(VERT_ARM_RETRACTED, vArmPickup, ENCODER_ERROR_THRESHOLD);
                if (vertArmState == VERT_ARM_RETRACTED) {
                    behavior = BEHAVIOR_FINISHED;
                }
            }
        }
    }
    public void vertArmAuto() {
        while (isStarted() && !isStopRequested() && (30 - runtime.seconds() >= AUTO_END_TIME)) {
            //put arm movements here
            checkBehaviors();
            if (behaviorStep == 1) {
                transferClaw.setPosition(TRANSFER_CLAW_CLOSE);
                vertArmState = vertArmMech(VERT_ARM_EXTENDING, vArmHigh, ENCODER_ERROR_THRESHOLD);
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_BACK);
                transferArmTop.setPosition(TRANSFER_ARM_TOP_BACK);
                if (vertArmState == VERT_ARM_EXTENDED) {
                    behaviorStep = 2;
                }
            }
            if (behaviorStep == 2) {
                vertArmState = vertArmMech(VERT_ARM_RETRACTING, vArmMid, ENCODER_ERROR_THRESHOLD);
                if (vertArmState == VERT_ARM_RETRACTED) {
                    transferClaw.setPosition(TRANSFER_CLAW_OPEN);
                    behaviorStep = 3;
                }
            }
            if (behaviorStep == 3) {
                transferArmTop.setPosition(TRANSFER_ARM_TOP_FRONT);
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_FRONT);
                if (transferArmTopState == TRANSFER_ARM_TOP_FRONT && transferArmBottomState == TRANSFER_ARM_BOTTOM_FRONT) {
                    behaviorStep = 4;
                }
            }
            if (behaviorStep == 4) {
                vertArmState = vertArmMech(VERT_ARM_RETRACTING, vArmLow, ENCODER_ERROR_THRESHOLD);
                if (vertArmState == VERT_ARM_RETRACTED) {
                    behavior = BEHAVIOR_FINISHED;
                }
            };
        }
    }
     */
}