package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import android.graphics.Color;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "MFAuto")
public class MFAuto extends LinearOpMode {
    HardwarePushbot2 robot = new HardwarePushbot2();

    /*
     * instantiate chassis object. chassis object should contains all driver capability.
     * Gyro sensor should be part of chassis
     * should have following methods:
     *    chassis.shift_left(double distance or double time);
     *    chassis.shift_right(double distance or double time);
     *    chassis.turn_left(double degress);
     *    chassis.turn_right(double degress);
     *    chassis.go_straight(double distance or double time); + foward, - backward
     * optional:
     *    chassis.go_leftdiag(double distance or double time); + foward, - backward
     *    chassis.go_rightdiag(double distance or double time); + foward, - backward
     */
    //HardwarePushbotChassis chassis = new HardwarePushbotChassis(); //FIXME:

    /*
     * instantiate lift and claw object and should have following method
     *     lift_claw.lift_self(double height) // extent, hook, retract
     *     lift_claw.lower_self(double height) // extend, unhook, retract
     *     lift_claw.close()
     *     lift_claw.open()
     *     lift_claw.updown(double height) + up, <=0 down and reset
     *     lift_claw.extend(double length) + extend out, <=0 retract
     */
    //HardWarePushbotLiftClaw lift_claw = new HardWarePushbotLiftClaw(); //FIXME:

    /*
     * instantiate ColorDistanceSensor. this could be part of chassis object
     *     colorDistant.is_color_yellow()
     *     colorDistant.query_distant()
     */
    //HardWarePushbotColorDistant colorDistant = new HardWarePushbotColorDistant(); //FIXME:

    //HardwarePushbot2 robot = new HardwarePushbot2();   // Use a Pushbot's hardware



    private ElapsedTime runtime = new ElapsedTime();

    public int lower_self() {
        // code zero
        robot.liftDrive.setPower(0);

        int raiseValue = robot.liftDrive.getCurrentPosition();

        while (robot.liftDrive.getCurrentPosition() - raiseValue < 100 ) {
            telemetry.addData("LiftMotor ", "ON");
            robot.liftDrive.setPower(.50);
            telemetry.addData("lift", robot.liftDrive.getCurrentPosition());
            telemetry.addData("raiseValue", raiseValue);
            telemetry.update();
        }// Raise the Lift
        telemetry.addData("done", "liftOver");
        robot.liftDrive.setPower(0);
        telemetry.update();

      /*  int startVal = robot.leftDrive.getCurrentPosition();
        while (robot.leftDrive.getCurrentPosition() - startVal < 1000) {
            // implement PID control here
            telemetry.addData("MOTORS ", "ON");
            robot.leftDrive.setPower(-0.5);
            robot.rightDrive.setPower(-0.5); fixme
            robot.leftDriveB.setPower(-0.5);
            robot.rightDriveB.setPower(-0.5);
            telemetry.addData("Motor", robot.leftDrive.getCurrentPosition());
            telemetry.addData("startValue", startVal);
            telemetry.update();*/

            int lowerValue = robot.liftDrive.getCurrentPosition();
            while (robot.liftDrive.getCurrentPosition() - lowerValue < 100) {
                //telemetry.addData("LiftMotor2 ", "ON");
                robot.liftDrive.setPower(-0.50);
                //telemetry.addData("lift2", robot.liftDrive.getCurrentPosition());
                //telemetry.addData("lowerValue", lowerValue);
                //telemetry.update();

            }
            //telemetry.addData("done", "liftOver2");
            robot.liftDrive.setPower(0);
            //telemetry.update();


            return 0;
    }


    //@Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //chassis(hardwareMap);  //FIXME:
        //lift_claw(hardwareMap);  //FIXME:
        //colorDistant(hardwarMap);  //FIXME:

        /*
         * main loop of autonomous
         */
        while (opModeIsActive()) {
            // lift_claw.lift_self()  //FIXME:

            // lift_claw.lower_self()  //FIXME:

            //FIXME:
            // chassis drive to next station
            // etc the whole game should be here.
            //hi this is anamika
        }

    }


}

