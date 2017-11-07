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
@Autonomous(name="Blue Autonomous", group="ColorSensing")
@Disabled
public class AutonomousBlue extends LinearOpMode {

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
        rightBack = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("right_front");
        rightFront = hardwareMap.dcMotor.get("left_back");
        leftFront = hardwareMap.dcMotor.get("right_back");
        pulleyarm = hardwareMap.dcMotor.get ("pulley_arm");
        spinner = hardwareMap.dcMotor.get ("spin_spin");
        collector = hardwareMap.dcMotor.get ("collect");

        //This is where we set the direction of the motors.

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //SERVO
        rightHitter = hardwareMap.servo.get("right_hitter");
        rightHitterPosition = .7;
        leftHitter = hardwareMap.servo.get("left_hitter");
        leftHitterPosition = .3;
        release = hardwareMap.servo.get("release_fork");
        releasePosition = .5;
        //opticalSensor = hardwareMap.opticalDistanceSensor.get("optical_sensor");
        colorSensor = hardwareMap.colorSensor.get("color");
        lightSensor = hardwareMap.opticalDistanceSensor.get("light");
        DIM = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        lightSensor.enableLed(true);
        telemetry.addData("Color Value", lightSensor.getLightDetected());
        telemetry.update();
        waitForStart();

        driveForward(.3, 1000);
        turnRight(500);
        driveForward(.3, 2000);
        turnRight(500);

        while (opModeIsActive()) {

            correction = (PERFECT_COLOR_VALUE - lightSensor.getLightDetected()) * SCALE_VALUE;
            telemetry.addData("Color Value", lightSensor.getLightDetected());
            telemetry.addData("Correction", correction);
            telemetry.update();

            // Sets the powers so they are no less than .075 and apply to correction
            if (correction <= 0) {
                leftPower = FOLLOW_POWER - correction;
                rightPower = FOLLOW_POWER;
            }
            else {
                leftPower = FOLLOW_POWER;
                rightPower = FOLLOW_POWER + correction;
            }

            // Sets the powers to the motors
            leftWheels(leftPower);
            rightWheels(rightPower);

        }
    }

   /*
   * Drive robot forward with a set power
   * @param Power the power that will be set on the motors
   * @param Time the amount of time the robot will keep the power on the motors
   */

    public void driveForward (double power, long time) throws InterruptedException
    {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
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
    public void turnRight(long time) throws InterruptedException
    {
        leftFront.setPower(.2);
        rightFront.setPower(-.2);
        leftBack.setPower(.2);
        rightBack.setPower(-.2);
        Thread.sleep(time);
    }
    public void turnLeft()
    {
        leftFront.setPower(-.2);
        rightFront.setPower(.2);
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
