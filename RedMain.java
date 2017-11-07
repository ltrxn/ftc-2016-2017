package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by Harish on 2/3/2017.
 */
@Autonomous(name="Burn", group="Red")
@Disabled
public class RedMain extends LinearOpMode {
    double leftHitterPosition;
    double releasePosition;
    double rightHitterPosition;

    public DcMotor pulleyarm;

    public DcMotor spinner;
    public DcMotor collector;
    //public DcMotor linearSlide;

    public Servo leftHitter;
    public Servo release;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public ColorSensor colorSensor;
    public OpticalDistanceSensor opticalSensor;
    public ModernRoboticsI2cGyro gyro;
    public double targetHeading = 0;
    public int zAccumulated;
    public double heading;
    static final double TOLERANCE = 4;
    public TouchSensor touchSensor;

    public Servo rightHitter;
    public DeviceInterfaceModule DIM;
    static final double WHITE_THRESHOLD = .13;


    public int X;
    public int Y;
    public int Z;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Ticks/Rev for Andy Mark Motors
    static final double DRIVE_GEAR_REDUCTION = .5;     // We have geared up, so Gear Reduction < 1
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;

    // Floor condition Factor, T for turn, D for drive
    public double dFlConditnFactorT;
    public double dFlConditnFactorD;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    @Override
    public void runOpMode() throws InterruptedException {

        //MOTOR

        leftFront = hardwareMap.dcMotor.get("left_front");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightBack = hardwareMap.dcMotor.get("right_back");

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinner = hardwareMap.dcMotor.get("spin_spin");
        collector = hardwareMap.dcMotor.get("collect");

        //SERVO

        rightHitter = hardwareMap.servo.get("right_hitter");
        //rightHitterPosition = .8;
        leftHitter = hardwareMap.servo.get("left_hitter");
        //leftHitterPosition = .2;
        release = hardwareMap.servo.get("release_fork");
        //releasePosition = .8;

        //SENSORS

        opticalSensor = hardwareMap.opticalDistanceSensor.get("light");
        colorSensor = hardwareMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        DIM = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        touchSensor = hardwareMap.touchSensor.get("touch");


        //INITIALIZATION

        rightHitter.setPosition(0.74);
        leftHitter.setPosition(.16);
        release.setPosition(1);
        opticalSensor.enableLed(true);
        boolean LEDstate = false;

        telemetry.addData(">", "Please wait for the gyro to calibrate. Don't ruin us.");
        telemetry.update();
        gyro.calibrate();

        //while (!isStopRequested() && gyro.isCalibrating()) {
        //    idle();
        //}

        // start the autonomous code
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            telemetry.addData("Turn Value", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        //gyro.resetZAxisIntegrator();



/*
            driveForward(.2);

            while (opModeIsActive() && (opticalSensor.getLightDetected() < WHITE_THRESHOLD)) {

                // Display the light level while we are looking for the line
                telemetry.addData("Light Level", opticalSensor.getLightDetected());
                telemetry.update();
            }
*/

        //sleep(2000);
        //encoderDrive(.5, 24);
        //encoderTurn(.5, 24);
        //encoderStrafe(.5, 0, 24);
        //encoderStrafe(.5, -.5, 24);

        encoderStrafe(.5, 0, 96);
        stopMovement();
        sleep(100);
        turnAbsolute(0);

        sleep(100);
        while (opticalSensor.getLightDetected() < WHITE_THRESHOLD){
            strafeRight();
        }
        stopMovement();



        /*sleep(110);
        stopMovement();
        sleep(1000);
        turnRight();
        while (opticalSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light Level Turning", opticalSensor.getLightDetected());
            telemetry.update();
        }
        stopMovement();
        sleep(500);
        turnRight();
        sleep(100);
        driveForward(.2);
        sleep(2000);
        //sleep(1000);
        opticalSensor.enableLed(false);
        colorSensor.enableLed(LEDstate);
        if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
            DIM.setLED(1, true);
            DIM.setLED(0, false);
            driveForward(-.2);
            sleep(500);
            rightHitter.setPosition(.55);
            driveForward(.1);
            sleep(700);
            driveForward(-.1);
            sleep(500);
            driveForward(.1);
            sleep(1700);
            telemetry.addData("Sensed Red Color", colorSensor.red());
        }
        else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
            DIM.setLED(0, true);
            DIM.setLED(1, false);
            driveForward(-.2);
            sleep(500);
            leftHitter.setPosition(.35);
            driveForward(.1);
            sleep(700);
            driveForward(-.1);
            sleep(500);
            driveForward(.1);
            sleep(1700);
            telemetry.addData("Sensed Blue Color", colorSensor.blue());
        }
        else {
            DIM.setLED(0, false);
            DIM.setLED(1, false);
            telemetry.addData("Sorry bruh. Better luck next time.", colorSensor.green());
        }
        //turnLeft();
        //sleep(500);
        driveForward(-.1);
        sleep(3500);
        stopMovement();
        sleep(100);
        launcher();
        sleep(3000);
        collector();
        sleep(7000);
        stoplauncher();
        collectorstop();
        strafeLeft();
        sleep(500);
        driveForward(-.4);
        sleep(2000);
        stopMovement();
        //sleep(1000);*/


    }

   /*
   * Drive robot forward with a set power
   * @param Power the power that will be set on the motors
   * @param Time the amount of time the robot will keep the power on the motors
   */

    public void encoderDrive(double power, double inches) {

        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightFrontTarget = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        leftBackTarget = leftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightBackTarget = rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
        }

        stopMovement();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurn(double power, double inches) {

        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightFrontTarget = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        leftBackTarget = leftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightBackTarget = rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(-leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(-leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turnLeft(power);

        while (leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy()) {
        }

        stopMovement();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + gyro.getIntegratedZValue());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                leftBack.setPower(turnSpeed);
                leftFront.setPower(turnSpeed);
                rightBack.setPower(-turnSpeed);
                rightFront.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                leftBack.setPower(-turnSpeed);
                leftFront.setPower(-turnSpeed);
                rightBack.setPower(turnSpeed);
                rightFront.setPower(turnSpeed);
            }

            zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        stopMovement();

    }
    /*public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= TOLERANCE) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftBack.setPower(leftSpeed);
        leftFront.setPower(leftSpeed);
        rightBack.setPower(rightSpeed);
        rightFront.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    /*public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }*/


    public void encoderStrafe(double nwSEPower, double swNEPower, double inches) {

        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightFrontTarget = rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        leftBackTarget = leftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightBackTarget = rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(-rightFrontTarget);
        leftBack.setTargetPosition(-leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(nwSEPower);
        rightFront.setPower(swNEPower);
        leftBack.setPower(swNEPower);
        rightBack.setPower(nwSEPower);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
        }

        stopMovement();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void stopMovement() {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
    }

    public void turnRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    public void turnLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }

    public void strafeLeft() {
        leftFront.setPower(-.2);
        rightFront.setPower(.2);
        leftBack.setPower(.2);
        rightBack.setPower(-.2);
    }

    public void strafeRight() {
        leftFront.setPower(1);
        rightFront.setPower(-1);
        leftBack.setPower(-1);
        rightBack.setPower(1);
    }

    public void rightHitter() {
        rightHitter.setPosition(1);
    }

    public void rightHitterReset() {
        rightHitter.setPosition(.3);
    }

    public void leftHitter() {
        leftHitter.setPosition(0);
    }

    public void leftHitterReser() {
        leftHitter.setPosition(.7);
    }

    public void launcher() {
        spinner.setPower(.25);
    }

    public void stoplauncher() {
        spinner.setPower(0);
    }

    public void collector() {
        collector.setPower(-.8);
    }

    public void collectorstop() {
        collector.setPower(0);
    }

    public void sampleRightHitter() {
        rightHitter.setPosition(1);
    }

    public void sampleRightHitterReset() {
        rightHitter.setPosition(.3);
    }

  /*
   * Turn the robot right with a set power
   * @param Power the power that will be set on the motors
   */



  /*
   * Drop the climbers, repeat dropping just in case climbers don't come out
   */
}
