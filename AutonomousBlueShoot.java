package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Harish on 12/28/2016.
 */
@Autonomous(name="BLUE Shoot and Center", group="Blue")

public class AutonomousBlueShoot extends LinearOpMode {

    double rightHitterPosition;
    double leftHitterPosition;
    double releasePosition;

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor pulleyarm;
    public DcMotor spinner;
    public DcMotor collector;
    //public DcMotor linearSlide;

    public Servo leftHitter;
    public Servo release;
    //public OpticalDistanceSensor opticalSensor;
    public ColorSensor colorSensor;
    OpticalDistanceSensor lightSensor;



    double leftPower, rightPower, correction;
    final double FOLLOW_POWER = .04;
    final double PERFECT_COLOR_VALUE = 0.41;
    final double SCALE_VALUE = .9;

    //public DcMotor linearSlide;
    public Servo rightHitter;
    public DeviceInterfaceModule DIM;

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
        pulleyarm = hardwareMap.dcMotor.get ("pulley_arm");
        spinner = hardwareMap.dcMotor.get ("spin_spin");
        collector = hardwareMap.dcMotor.get ("collect");

        //This is where we set the direction of the motors.

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //SERVO
        rightHitter = hardwareMap.servo.get("right_hitter");
        rightHitterPosition = .8;
        leftHitter = hardwareMap.servo.get("left_hitter");
        leftHitterPosition = .9;
        release = hardwareMap.servo.get("release_fork");
        releasePosition = .5;
        //opticalSensor = hardwareMap.opticalDistanceSensor.get("optical_sensor");
        colorSensor = hardwareMap.colorSensor.get("color");
        lightSensor = hardwareMap.opticalDistanceSensor.get("light");
        DIM = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        lightSensor.enableLed(true);
        telemetry.addData("Color Value", lightSensor.getLightDetected());
        telemetry.update();
        waitForStart();//

        stopMovement();
        sleep(10000);
        driveForward(.25, 1650);
        stopMovement();
        launcher();
        sleep(2000);
        collector();
        sleep(4000);
        stoplauncher();
        collectorstop();
        //driveForward(.2, 1000);
        sleep(450);
        driveForward(.4, 1300);
        stopMovement();

        }

   /*
   * Drive robot forward with a set power
   * @param Power the power that will be set on the motors
   * @param Time the amount of time the robot will keep the power on the motors
   */

    public void driveForward (double power, long time) throws InterruptedException
    {
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        Thread.sleep(time);
    }
    public void stopMovement ()
    {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
    }
    public void leftWheels(double power)
    {
        leftFront.setPower(power);
        leftBack.setPower(power);
    }
    public void rightWheels(double power)
    {
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
    public void turnRight()
    {
        leftFront.setPower(-.5);
        rightFront.setPower(.5);
        leftBack.setPower(-.5);
        rightBack.setPower(.5);
    }
    public void turnLeft()
    {
        leftFront.setPower(.5);
        rightFront.setPower(-.5);
        leftBack.setPower(.5);
        rightBack.setPower(-.5);
    }
    public void strafeLeft(){
        leftFront.setPower(-1);
        rightFront.setPower(1);
        leftBack.setPower(1);
        rightBack.setPower(-1);
    }
    public void strafeRight(){
        leftFront.setPower(1);
        rightFront.setPower(-1);
        leftBack.setPower(-1);
        rightBack.setPower(1);
    }

    public void launcher() {
        spinner.setPower(.3125);
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
