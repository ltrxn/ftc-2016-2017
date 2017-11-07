package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Harish on 12/28/2016.e
 */
@TeleOp(name="Operation Jaguar", group="Competition TeleOp")
@Disabled
public class OperationJaguar extends LinearOpMode {

    final static double V1_MIN_RANGE = -1;
    final static double V1_MAX_RANGE = 1;
    final static double V2_MIN_RANGE = -1;
    final static double V2_MAX_RANGE = 1;
    final static double V3_MIN_RANGE = -1;
    final static double V3_MAX_RANGE = 1;
    final static double V4_MIN_RANGE = -1;
    final static double V4_MAX_RANGE = 1;
    final static double RIGHT_HITTER_MIN_RANGE = 0;
    final static double RIGHT_HITTER_MAX_RANGE = 1;
    final static double LEFT_HITTER_MIN_RANGE = 0;
    final static double LEFT_HITTER_MAX_RANGE = 1;
    final static double RELEASE_MIN_RANGE = 0;
    final static double RELEASE_MAX_RANGE = 1;

    double rightHitterPosition;
    double leftHitterPosition;
    double releasePosition;
    double releaseBallPosition;

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor pulleyarm;
    public DcMotor spinner;
    public DcMotor collector;
    //public DcMotor linearSlide;

    public Servo rightHitter;
    public Servo leftHitter;
    public Servo release;
    public Servo releaseBall;

    boolean changed = false, on = false; //Outside of loop()
    boolean xchanged = false, xon = false; //Outside of loop()

    @Override
    public void runOpMode() throws InterruptedException {
/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


        //This is where we specify what motors our motor names refer to.

        leftFront = hardwareMap.dcMotor.get("left_front");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightBack = hardwareMap.dcMotor.get("right_back");
        pulleyarm = hardwareMap.dcMotor.get("pulley_arm");
        spinner = hardwareMap.dcMotor.get("spin_spin");
        collector = hardwareMap.dcMotor.get("collect");

        //linearSlide = hardwareMap.dcMotor.get(" ");

        //This is where we set the direction of the motors.

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //SERVO

        rightHitter = hardwareMap.servo.get("right_hitter");
        rightHitter.setPosition(.7);
        leftHitter = hardwareMap.servo.get("left_hitter");
        leftHitter.setPosition(.19);
        release = hardwareMap.servo.get("release_fork");
        release.setPosition(1);
        releaseBall = hardwareMap.servo.get("release_ball");
        releaseBall.setPosition(.9);

        waitForStart();

        while (opModeIsActive()) {


            //REAL MECANUM DRIVE CODE
/*
        double forwardBackward = gamepad1.left_stick_y;
        double rightLeft = gamepad1.left_stick_x;
        double antiClock;
        double Clock;
        forwardBackward = (float) scaleInput(forwardBackward);
        rightLeft = (float) scaleInput(rightLeft);

        if (gamepad1.left_trigger > .25) {
            antiClock = .4;
        } else {
            antiClock = 0;
        }

        if (gamepad1.right_trigger > .25) {
            Clock = .4;
        } else {
            Clock = 0;
        }

        final double v1 = -forwardBackward + rightLeft - antiClock + Clock;
        final double v2 = -forwardBackward - rightLeft + antiClock - Clock;
        final double v3 = -forwardBackward - rightLeft - antiClock + Clock;
        final double v4 = -forwardBackward + rightLeft + antiClock - Clock;
        /*v1 = Range.clip(v1, V1_MIN_RANGE, V1_MAX_RANGE);
        v2 = Range.clip(v2, V2_MIN_RANGE, V1_MAX_RANGE);
        v3 = Range.clip(v3, V3_MIN_RANGE, V3_MAX_RANGE);
        v4 = Range.clip(v4, V4_MIN_RANGE, V4_MAX_RANGE);
        leftFront.setPower(v1/2);
        rightFront.setPower(v2/2);
        leftBack.setPower(v3/2);
        rightBack.setPower(v4/2);
*/
            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x / 2;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setPower(-v1);
            rightFront.setPower(-v2);
            leftBack.setPower(-v3);
            rightBack.setPower(-v4);

            //Coding for the slides.

            float pulley_arm = gamepad2.left_stick_y;

            pulley_arm = Range.clip(pulley_arm, -1, 1);

            pulleyarm.setPower(pulley_arm);

            //Coding for the Beacon Pushers

            if (gamepad1.dpad_right || gamepad2.right_bumper) {
                rightHitter.setPosition(.55);
            }
            if (gamepad1.dpad_left || gamepad2.left_bumper) {
                leftHitter.setPosition(.35);
            } else if (gamepad1.b || gamepad2.a) {
                rightHitter.setPosition(.74);
                leftHitter.setPosition(.16);
            }

            //Coding for ball blocker

            if(gamepad1.dpad_up || gamepad2.dpad_up){
                releaseBall.setPosition(.5);
            }
            else if (gamepad2.dpad_down || gamepad2.dpad_down) {
                releaseBall.setPosition(.9);
            }

            //Release the Fork

            if (gamepad1.x) {
                release.setPosition(.5);
            } else if (gamepad1.y) {
                release.setPosition(1);
            }

            if (gamepad2.x) {
                release.setPosition(.5);
            }

            //Coding for Spinner that lifts cap ball

            float spin_spin = -gamepad2.right_stick_y;

            spin_spin = Range.clip(spin_spin, -1, 1);

            spinner.setPower(spin_spin / 4);

            //Coding for the Collector
 /*
        float collect = -gamepad1.right_stick_y;

        collect = Range.clip(collect, -1, 1);

        collector.setPower(-collect);
 */
            if (gamepad1.right_bumper) {
                collector.setPower(-0.5);
            } else if (gamepad1.left_bumper) {
                collector.setPower(0);
            } else if (gamepad1.right_trigger > .1) {
                collector.setPower(.2);
            } else if (gamepad1.left_trigger > .1) {
                collector.setPower(-.2);
            }
            idle();
        }

    }
}