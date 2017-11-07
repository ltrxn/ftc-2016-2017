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
 * Created by Harish on 12/28/2016.
 */
@Autonomous(name="Red Autonomous", group="ColorSensing")
@Disabled
public class AutonomousRed extends LinearOpMode {

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

        boolean LEDstate = true;

        // start the autonomous code
        waitForStart();
        driveForward(.2);
        sleep(2000);
        opticalSensor.enableLed(true);
        while (opticalSensor.getLightDetected() < .4)
        {
            telemetry.addData("Regular Light",opticalSensor.getLightDetected());
            telemetry.addData("Raw Regular Light",opticalSensor.getRawLightDetected());
            driveForward(.05);
        }
        stopMovement();
        sleep(1000);
        turnLeft();
        sleep(1000);
        driveForward(.2);
        sleep(1000);
        stopMovement();
        opticalSensor.enableLed(false);
        colorSensor.enableLed(LEDstate);
        if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
            DIM.setLED(1, true);
            DIM.setLED(0, false);
            rightHitter.setPosition(.6);
            telemetry.addData("Sensed Red Color", colorSensor.red());
        }
        else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
            DIM.setLED(0, true);
            DIM.setLED(1, false);
            leftHitter.setPosition(.4);
            telemetry.addData("Sensed Blue Color", colorSensor.blue());
        }
        else {
            DIM.setLED(0, false);
            DIM.setLED(1, false);
            telemetry.addData("Sorry bruh. Better luck next time.", colorSensor.green());
        }
        //driveForward(.2);
        //sleep(2000);
        //stopMovement();
        //sleep(2000);
        //turnRight();
        //sleep(2000);
        //stopMovement();
        //sleep(1000);
        //sampleRightHitter();
        //sleep(1000);
        //sampleRightHitterReset();
        //sleep(2000);
        //stopMovement();
        //telemetry.addData("Red", colorSensor.red());

        /*while(opticalSensor.getLightDetected()<.0189){
            driveForward(.4);
        }*/
    }

   /*
   * Drive robot forward with a set power
   * @param Power the power that will be set on the motors
   * @param Time the amount of time the robot will keep the power on the motors
   */

    public void driveForward (double power)
    {
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
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
        leftFront.setPower(-.2);
        rightFront.setPower(.2);
        leftBack.setPower(.2);
        rightBack.setPower(-.2);
    }
    public void strafeRight()
    {
        leftFront.setPower(.2);
        rightFront.setPower(-.2);
        leftBack.setPower(-.2);
        rightBack.setPower(.2);
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