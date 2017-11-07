package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by famil on 2/5/2017.
 */

@Autonomous(name="EncoderRed", group="Blue")
@Disabled
public class EncoderRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    double leftHitterPosition;
    double releasePosition;
    double rightHitterPosition;

    public DcMotor pulleyarm;

    public DcMotor spinner;
    public DcMotor collector;

    public Servo leftHitter;
    public Servo release;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public ColorSensor colorSensor;
    public OpticalDistanceSensor opticalSensor;
    //public ModernRoboticsI2cGyro gyro;
    //public double heading = gyro.getHeading();
    public TouchSensor touchSensor;

    public Servo rightHitter;
    public DeviceInterfaceModule DIM;
    static final double WHITE_THRESHOLD = .24;


    public int X;
    public int Y;
    public int Z;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Ticks/Rev for Andy Mark Motors
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        //MOTORS

        leftFront = hardwareMap.dcMotor.get("left_front");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightBack = hardwareMap.dcMotor.get("right_back");

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        spinner = hardwareMap.dcMotor.get("spin_spin");
        collector = hardwareMap.dcMotor.get("collect");

        //SERVO

        rightHitter = hardwareMap.servo.get("right_hitter");
        leftHitter = hardwareMap.servo.get("left_hitter");
        release = hardwareMap.servo.get("release_fork");

        opticalSensor = hardwareMap.opticalDistanceSensor.get("light");
        colorSensor = hardwareMap.colorSensor.get("color");
        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        DIM = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        touchSensor = hardwareMap.touchSensor.get("touch");


        //INITIALIZATION

        rightHitter.setPosition(0.7);
        leftHitter.setPosition(.19);
        release.setPosition(.8);
        opticalSensor.enableLed(true);
        boolean LEDstate = false;


        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightBack.getCurrentPosition());
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        waitForStart();

        encoderStrafe(1, 0, 48, 5.0);

        //sleep(1000);

        //driveForward(.3);

        while (opModeIsActive() && (opticalSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            telemetry.update();
        }

        stopMovement();
        telemetry.addData("Path", "Complete");
        telemetry.update();



        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  48,  48 , 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftBackTarget = leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(leftFrontTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightBack.setTargetPosition(rightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", leftFrontTarget, rightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void encoderStrafe(double speed,
                             double swNEInches, double nwSEInches,
                             double timeoutS) {
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftFrontTarget = leftFront.getCurrentPosition() + (int)(nwSEInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFront.getCurrentPosition() + (int)(swNEInches * COUNTS_PER_INCH);
            leftBackTarget = leftBack.getCurrentPosition() + (int)(nwSEInches * COUNTS_PER_INCH);
            rightBackTarget = rightBack.getCurrentPosition() + (int)(swNEInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(leftFrontTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightBack.setTargetPosition(rightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path3", "Running to %7d :%7d", leftFrontTarget, rightFrontTarget);
                telemetry.addData("Path4", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void driveForward (double power)
    {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
    public void stopMovement ()
    {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
    }
    public void turnRight()
    {
        leftFront.setPower(-.3);
        rightFront.setPower(.3);
        leftBack.setPower(-.3);
        rightBack.setPower(.3);
    }
    public void turnLeft()
    {
        leftFront.setPower(.2);
        rightFront.setPower(-.2);
        leftBack.setPower(.2);
        rightBack.setPower(-.2);
    }
    public void strafeLeft()
    {
        leftFront.setPower(-.2);
        rightFront.setPower(.2);
        leftBack.setPower(.2);
        rightBack.setPower(-.2);
    }
    public void strafeRight()
    {
        leftFront.setPower(1);
        rightFront.setPower(-1);
        leftBack.setPower(-1);
        rightBack.setPower(1);
    }
    public void rightHitter(){
        rightHitter.setPosition(1);
    }
    public void rightHitterReset(){
        rightHitter.setPosition(.3);
    }
    public void leftHitter() {
        leftHitter.setPosition(0);
    }
    public void leftHitterReser(){
        leftHitter.setPosition (.7);
    }
    public void launcher() {
        spinner.setPower(.25);
    }
    public void stoplauncher(){
        spinner.setPower(0);
    }
    public void collector(){
        collector.setPower(-.8);
    }
    public void collectorstop(){
        collector.setPower(0);
    }
    public void sampleRightHitter(){
        rightHitter.setPosition(1);
    }
    public void sampleRightHitterReset(){
        rightHitter.setPosition(.3);
    }
}