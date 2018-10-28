/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "MFTeleop", group = "Pushbot")
//@Disabled

public class MFTeleop extends OpMode {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftDriveB = null;
    public DcMotor rightDriveB = null;
    public DcMotor liftDrive = null; //neverrest 60 - 1
    int boomStart;
    int raiseValue;
    int lowerValue;
    /* Declare OpMode members. */
    HardwarePushbot2 robot = new HardwarePushbot2(); // use the class created to define a Pushbot's hardware

    // could also use HardwarePushbotMatrix class.
    //  double          clawOffset  = 0.0 ;                  // Servo mid position
    //  final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    //ColorSensor sensorColor1;
    //ColorSensor sensorColor2; fixme
    //DistanceSensor sensorDistance1;
    //DistanceSensor sensorDistance2;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

// get a reference to the color sensor.

        telemetry.addData("Say", "Hello Driver");    //
        //reference to motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDriveB = hardwareMap.get(DcMotor.class, "left_driveB");
        rightDriveB = hardwareMap.get(DcMotor.class, "right_driveB");
        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        boomStart = robot.armDrive.getCurrentPosition();//starts at 7

        raiseValue = robot.liftDrive.getCurrentPosition();
        lowerValue = robot.liftDrive.getCurrentPosition();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        // double sideleft;
        // double sideleftneg;
        double sideright;
        double siderightneg;
        double boom;
        double tilt;
        double close;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        //sideleft = gamepad1.left_stick_x;
        //sideleftneg = -gamepad1.left_stick_x;
        sideright = -gamepad1.right_stick_x;
        siderightneg = gamepad1.right_stick_x;
        if (java.lang.Math.abs(sideright) > 0 && java.lang.Math.abs(-gamepad1.right_stick_y) < 0.9) {
            right = 0;
            left = 0;
        }
        if (java.lang.Math.abs(siderightneg) > 0 && java.lang.Math.abs(gamepad1.right_stick_y) < 0.9) {
            left = 0;
            right = 0;
        }
       // boom = gamepad2.right_stick_y;
        //tilt = gamepad2.left_stick_y;
        //close = gamepad2.right_trigger;
      //  robot.armDrive.setPower(boom);
        //robot.armTiltDrive.setPower(tilt);
        //robot.claw.setPosition(close);

        telemetry.addData("armDrive", robot.armDrive.getCurrentPosition());
        telemetry.addData("armTiltDrive", robot.armTiltDrive.getCurrentPosition());// telemetry for arm
        telemetry.addData("claw", robot.claw.getPosition());
        //telemetry.update();
        // get a reference to the color sensor.
        //go forward & backwards
        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        robot.leftDriveB.setPower(left);
        robot.rightDriveB.setPower(right);

       /* // go left
        robot.leftDrive.setPower(sideleftneg);
        robot.rightDrive.setPower(sideleft);
        robot.leftDriveB.setPower(sideleft); fixme
        robot.rightDriveB.setPower(sideleftneg);*/

        // go right
        robot.leftDrive.setPower(sideright);
        robot.leftDriveB.setPower(siderightneg);
        robot.rightDrive.setPower(siderightneg);
        robot.rightDriveB.setPower(sideright);



        if (robot.liftDrive.getCurrentPosition() - raiseValue < 1440 && gamepad2.dpad_up) {
            telemetry.addData("LiftMotor ", "ON");
            robot.liftDrive.setPower(-.50);
            telemetry.addData("lift", robot.liftDrive.getCurrentPosition());
            telemetry.addData("raiseValue", raiseValue);
            telemetry.update();
        }// Raise the Lift
        telemetry.addData("done", "liftOver");
        robot.liftDrive.setPower(0);
        telemetry.update();


        if (robot.liftDrive.getCurrentPosition() - lowerValue < 1440 && gamepad2.dpad_down) {
            telemetry.addData("LiftMotor2 ", "ON");
            robot.liftDrive.setPower(.50);
            telemetry.addData("lift2", robot.liftDrive.getCurrentPosition());
            telemetry.addData("lowerValue", lowerValue);
            telemetry.update();

        }
        telemetry.addData("done", "liftOver2");
        robot.liftDrive.setPower(0);
        telemetry.update();

        double armpower = gamepad2.right_stick_y;
        double armmult = 1.0;
        if ( boomStart - robot.armDrive.getCurrentPosition() < 200 ){

            armmult = .25;

        }
        else if ( boomStart - robot.armDrive.getCurrentPosition() < 400 ){
            armmult = .50;
        } else {
                if (armpower < 0) {
                    armmult = 1.0;
                } else {
                    armmult = .2;                }
        }
        robot.armDrive.setPower(armpower * armmult);

        double tiltarmpower = gamepad2.left_stick_y;
        double tiltarmmult = .75;
        robot.armTiltDrive.setPower(tiltarmpower*tiltarmmult);









        // Use gamepad left & right Bumpers to open and close the claw
        //  if (gamepad1.right_bumper)
        //      clawOffset += CLAW_SPEED;
        //  else if (gamepad1.left_bumper)
        //     clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        //  clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        //  robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        //   robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        // if (gamepad1.y)
        //     robot.leftArm.setPower(robot.ARM_UP_POWER);
        //  else if (gamepad1.a)
        //     robot.leftArm.setPower(robot.ARM_DOWN_POWER);millennium falcon
        //  else
        //     robot.leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
        //  telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        //telemetry.addData("left", "%.2f", left);


        //  telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
