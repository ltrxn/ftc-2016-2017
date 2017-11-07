package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by famil on 1/21/2017.
 */
@Autonomous(name="Red Color + Corner", group="Red")
@Disabled
public class RedColorCorner extends LinearOpMode {
    double leftHitterPosition;
    double releasePosition;

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
    //public OpticalDistanceSensor opticalSensor;
    public ColorSensor colorSensor;
    public OpticalDistanceSensor opticalSensor;
    //public DcMotor linearSlide;
    public Servo rightHitter;
    public DeviceInterfaceModule DIM;
    static final double WHITE_THRESHOLD = .1;

    public double rightHitterPosition;

    // Floor condition Factor, T for turn, D for drive
    public double dFlConditnFactorT;
    public double dFlConditnFactorD;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    @Override
    public void runOpMode() throws InterruptedException {

        //These motors are what spin the wheels
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightBack = hardwareMap.dcMotor.get("right_back");
        spinner = hardwareMap.dcMotor.get("spin_spin");
        collector = hardwareMap.dcMotor.get("collect");

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightHitter = hardwareMap.servo.get("right_hitter");
        //rightHitterPosition = .8;
        leftHitter = hardwareMap.servo.get("left_hitter");
        //leftHitterPosition = .2;
        release = hardwareMap.servo.get("release_fork");
        //releasePosition = .8;

        opticalSensor = hardwareMap.opticalDistanceSensor.get("light");
        colorSensor = hardwareMap.colorSensor.get("color");
        DIM = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        //Initialization Area

        rightHitter.setPosition(0.7);
        leftHitter.setPosition(.3);
        release.setPosition(.8);
        opticalSensor.enableLed(true);
        boolean LEDstate = false;

        telemetry.addData("You ready?","Oh bet I am");
        telemetry.update();

        // start the autonomous code
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        driveForward(.2);

        while (opModeIsActive() && (opticalSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            telemetry.update();
        }

        // Stop all motors
        driveForward(.1);
        sleep(110);
        stopMovement();
        sleep(1000);
        turnLeft();
        while (opticalSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light Level Turning", opticalSensor.getLightDetected());
            telemetry.update();
        }
        stopMovement();
        sleep(500);
        turnLeft();
        sleep(200);
        driveForward(.2);
        sleep(2000);
        //sleep(1000);
        opticalSensor.enableLed(false);
        colorSensor.enableLed(LEDstate);
        if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
            DIM.setLED(1, true);
            DIM.setLED(0, false);
            driveForward(-.2);
            sleep(500);
            rightHitter.setPosition(.4);
            driveForward(.1);
            sleep(500);
            driveForward(-.1);
            sleep(500);
            driveForward(.1);
            sleep(1100);
            telemetry.addData("Sensed Red Color", colorSensor.red());
        }
        else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
            DIM.setLED(0, true);
            DIM.setLED(1, false);
            driveForward(-.2);
            sleep(500);
            leftHitter.setPosition(.5);
            driveForward(.1);
            sleep(500);
            driveForward(-.1);
            sleep(500);
            driveForward(.1);
            sleep(1200);
            telemetry.addData("Sensed Blue Color", colorSensor.blue());
        }
        else {
            DIM.setLED(0, false);
            DIM.setLED(1, false);
            telemetry.addData("Sorry bruh. Better luck next time.", colorSensor.green());
        }
        driveForward(-.2);
        sleep(700);
        strafeLeft();
        sleep(4000);
        //sleep(1000);



    }

   /*
   * Drive robot forward with a set power
   * @param Power the power that will be set on the motors
   * @param Time the amount of time the robot will keep the power on the motors
   */

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
        leftFront.setPower(-.2);
        rightFront.setPower(.2);
        leftBack.setPower(-.2);
        rightBack.setPower(.2);
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
        leftFront.setPower(-.5);
        rightFront.setPower(.5);
        leftBack.setPower(.5);
        rightBack.setPower(-.5);
    }
    public void strafeRight()
    {
        leftFront.setPower(.5);
        rightFront.setPower(-.5);
        leftBack.setPower(-.5);
        rightBack.setPower(.5);
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
        spinner.setPower(.229);
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

  /*
   * Turn the robot right with a set power
   * @param Power the power that will be set on the motors
   */



  /*
   * Drop the climbers, repeat dropping just in case climbers don't come out
   */
}
