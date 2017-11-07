package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by Harish on 2/3/2017.
 */
@Autonomous(name="Blue Flames (1 Beacon, Shooting, center Vortex)", group="Blue")

public class RealBlueAutonomous extends LinearOpMode {
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
    public DcMotor lights;
    public ColorSensor colorSensor;
    public OpticalDistanceSensor opticalSensor;
    public GyroSensor gyroMan;
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
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;
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
        lights = hardwareMap.dcMotor.get("lights");

        //SERVO

        rightHitter = hardwareMap.servo.get("right_hitter");
        //rightHitterPosition = .7;
        leftHitter = hardwareMap.servo.get("left_hitter");
        //leftHitterPosition = .2;
        release = hardwareMap.servo.get("release_fork");
        //releasePosition = .8;

        //SENSORS

        opticalSensor = hardwareMap.opticalDistanceSensor.get("light");
        colorSensor = hardwareMap.colorSensor.get("color");
        gyroMan = hardwareMap.gyroSensor.get("gyro");
        gyro = (ModernRoboticsI2cGyro) gyroMan;
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

        while (gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        gyro.resetZAxisIntegrator();

        // start the autonomous code
        while (!(isStarted() || isStopRequested())) {

            telemetry.addData(">", "Do not go gentle into that good night.");
            telemetry.addData(">", "Rage, rage against the dying of the light.");
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            telemetry.addData("Turn Value", gyro.getHeading());
            telemetry.addData("Accumulated Z", gyro.getIntegratedZValue());
            telemetry.addData("Touch Sensor pressed?: ", touchSensor.isPressed());
            telemetry.update();
            idle();
        }

/*
            driveForward(.2);

            while (opModeIsActive() && (opticalSensor.getLightDetected() < WHITE_THRESHOLD)) {

                // Display the light level while we are looking for the line
                telemetry.addData("Light Level", opticalSensor.getLightDetected());
                telemetry.update();
            }
*/
        telemetry.addLine("Aaa");
        //sleep(2000);
        //encoderDrive(.5, 24);
        //encoderTurn(.5, 24);
        //encoderStrafe(.5, 0, 24);
        //encoderStrafe(.5, -.5, 24);
        //.5 means that northwest and southeast (the front left and back right motors) are

        //Go to 1st Beacon
       // encoderStrafe(0, -.5, -110);
      //  stopMovement();
        //gyroTurn(320);
        //sleep(100);
        driveForward(.2);
        while (opticalSensor.getLightDetected() < WHITE_THRESHOLD){
            telemetry.addData("Please don't", "mess us up");
            telemetry.update();
        }
        stopMovement();
        sleep(100);
        telemetry.addData("ZAccumulated: ",  gyro.getIntegratedZValue());
        telemetry.update();
        gyroTurn(-5);

        //1st Beacon
        while(touchSensor.isPressed() != true)
        {
            driveForward(.15);
        }
        stopMovement();
        sleep(100);
        //encoderDrive(-.15, 5);
        driveForward(-.15);
        sleep(200);
        stopMovement();
        sleep(100);
        colorSensor.enableLed(LEDstate);
        if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
            DIM.setLED(0, true);
            DIM.setLED(1, false);
            //driveForward(-.2);
            //encoderDrive();
            leftHitter.setPosition(.35);
            //driveForward(.1);
            driveForward(.15);
            //driveForward(-.1);
            //sleep(500);
            //driveForward(.1);
            sleep(1700);
            telemetry.addData("Sensed Blue Color", colorSensor.blue());

        }
        else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
            DIM.setLED(1, true);
            DIM.setLED(0, false);
            //driveForward(-.2);
            //encoderDrive();
            rightHitter.setPosition(.55);
            //driveForward(.1);
            driveForward(.15);
            //driveForward(-.1);
            //sleep(500);
            //driveForward(.1);
            sleep(1700);
            telemetry.addData("Sensed Red Color", colorSensor.red());
        }
        else {
            DIM.setLED(0, false);
            DIM.setLED(1, false);
            telemetry.addData("Sorry bruh. Better luck next time.", colorSensor.green());
        }


        //Shoot
        driveForward(-.25);
        sleep(700);
        stopMovement();
        launcher();
        sleep(2000);
        collector();
        sleep(5000);
        stoplauncher();
        collectorstop();

        turnLeft(.3);
        sleep(400);
        stopMovement();
        driveForward(-.30);
        sleep(2000);

        //2ndbeacon
        /*driveForward(.25);
        sleep(400);
        strafeRight(.25);
        sleep(500);
        while (optica mlSensor.getLightDetected() < WHITE_THRESHOLD) {
           strafeRight(.25);
        }
        stopMovement();
        sleep(100);
        stopMovement();
        telemetry.addData("ZAccumulated: ",  gyro.getIntegratedZValue());
        telemetry.update();
        sleep(50);
        gyroTurn(-5);
        while(touchSensor.isPressed() != true)
        {
            driveForward(.15);
        }
        stopMovement();
        sleep(100);
        //encoderDrive(-.15, 5);
        driveForward(-.15);
        sleep(200);
        stopMovement();
        sleep(100);
        colorSensor.enableLed(LEDstate);
        if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
            DIM.setLED(0, true);
            DIM.setLED(1, false);
            //driveForward(-.2);
            //encoderDrive();
            leftHitter.setPosition(.35);
            //driveForward(.1);
            driveForward(.15);
            //driveForward(-.1);
            //sleep(500);
            //driveForward(.1);
            sleep(1700);
            telemetry.addData("Sensed Blue Color", colorSensor.blue());

        }
        else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
            DIM.setLED(1, true);
            DIM.setLED(0, false);
            //driveForward(-.2);
            //encoderDrive();
            rightHitter.setPosition(.55);
            //driveForward(.1);
            driveForward(.15);
            //driveForward(-.1);
            //sleep(500);
            //driveForward(.1);
            sleep(1700);
            telemetry.addData("Sensed Red Color", colorSensor.red());
        }
        else {
            DIM.setLED(0, false);
            DIM.setLED(1, false);
            telemetry.addData("Sorry bruh. Better luck next time.", colorSensor.green());
        }

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

        while (leftFront.getCurrentPosition() < leftFront.getTargetPosition()) {
            telemetry.addLine("LeftFront pos: " + leftFront.getCurrentPosition());
            telemetry.addLine("RightFront pos: " + rightFront.getCurrentPosition());
            telemetry.addLine("Leftback pos: " + leftBack.getCurrentPosition());
            telemetry.addLine("Rightback pos: " + rightBack.getCurrentPosition());
            telemetry.addLine("Target pos: " + leftFront.getTargetPosition());
            telemetry.addLine("Target pos: " + leftBack.getTargetPosition());
            telemetry.addLine("Target pos: " + rightFront.getTargetPosition());
            telemetry.addLine("Target pos: " + rightBack.getTargetPosition());
            telemetry.update();
        }

        stopMovement();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderTurn(double power, double angle) {

        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontTarget = leftFront.getCurrentPosition() + (int) (angle * COUNTS_PER_DEGREE);
        rightFrontTarget = rightFront.getCurrentPosition() + (int) (angle * COUNTS_PER_DEGREE);
        leftBackTarget = leftBack.getCurrentPosition() + (int) (angle * COUNTS_PER_DEGREE);
        rightBackTarget = rightBack.getCurrentPosition() + (int) (angle * COUNTS_PER_DEGREE);

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(-rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(-rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turnRight(power);

        while (leftFront.getCurrentPosition() < leftFront.getTargetPosition()) {

            telemetry.addLine("LeftFront pos: " + leftFront.getCurrentPosition());
            telemetry.addLine("RightFront pos: " + rightFront.getCurrentPosition());
            telemetry.addLine("Leftback pos: " + leftBack.getCurrentPosition());
            telemetry.addLine("Rightback pos: " + rightBack.getCurrentPosition());
            telemetry.addLine("Target pos: " + leftFront.getTargetPosition());
            telemetry.addLine("Target pos: " + leftBack.getTargetPosition());
            telemetry.addLine("Target pos: " + rightFront.getTargetPosition());
            telemetry.addLine("Target pos: " + rightBack.getTargetPosition());
            telemetry.update();
        }

        stopMovement();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
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

        while (leftFront.getCurrentPosition() < leftFront.getTargetPosition()) {
            telemetry.addLine("LeftFront pos: " + leftFront.getCurrentPosition());
            telemetry.addLine("RightFront pos: " + rightFront.getCurrentPosition());
            telemetry.addLine("Leftback pos: " + leftBack.getCurrentPosition());
            telemetry.addLine("Rightback pos: " + rightBack.getCurrentPosition());
            telemetry.addLine("Target pos: " + leftFront.getTargetPosition());
            telemetry.addLine("Target pos: " + leftBack.getTargetPosition());
            telemetry.addLine("Target pos: " + rightFront.getTargetPosition());
            telemetry.addLine("Target pos: " + rightBack.getTargetPosition());
            telemetry.update();
        }

        stopMovement();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroTurn(int target){

        zAccumulated = gyro.getIntegratedZValue();
        double turnSpeed = .25;

        while (Math.abs(zAccumulated - target) > 2) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                leftFront.setPower(-turnSpeed);
                rightFront.setPower(turnSpeed);
                leftBack.setPower(-turnSpeed);
                rightBack.setPower(turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                leftFront.setPower(turnSpeed);
                rightFront.setPower(-turnSpeed);
                leftBack.setPower(turnSpeed);
                rightBack.setPower(-turnSpeed);
            }

            zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        stopMovement();
    }
    public void driveForward(double power) {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void stopMovement() {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
    }

    public void turnRight(double power) {
/*        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    public void turnLeft(double power) {
/*        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }

    public void strafeLeft(double power) {

/*        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

        //was .2
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    public void strafeRight(double power) {

/*        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

        //was .1
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
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
        spinner.setPower(.315);
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