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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class MFChassis extends LinearOpMode{

    /* Declare OpMode members. */
    HardwarePushbot2 robot = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime period  = new ElapsedTime();

    static final double     DRIVE_SPEED             = 0.6;

    /* Constructor */
    public MFChassis(){
    }


    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Lift",  "Starting at %7d",
                          robot.liftDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int startVal = robot.leftDrive.getCurrentPosition();

        while (robot.leftDrive.getCurrentPosition() - startVal < 2000){
            telemetry.addData("MOTORS ", "ON");
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        robot.leftDriveB.setPower(0.5);
        robot.rightDriveB.setPower(0.5);
        telemetry.addData("left", robot.leftDrive.getCurrentPosition());
        telemetry.addData("startValue", startVal);
        telemetry.update();}

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveB.setPower(0);
        robot.rightDriveB.setPower(0);

        int startValR = robot.rightDrive.getCurrentPosition();

        while (robot.rightDrive.getCurrentPosition() - startValR <  1000 ){
        robot.leftDrive.setPower(-0.5);
        robot.rightDrive.setPower(0.5);
        robot.leftDriveB.setPower(0.5);
        robot.rightDriveB.setPower(-0.5);
        telemetry.addData("right", robot.rightDrive.getCurrentPosition());
        telemetry.addData("startValue", startValR);
        telemetry.update();}

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveB.setPower(0);
        robot.rightDriveB.setPower(0);

        while(robot.leftDrive.getCurrentPosition() - startVal < 1000){
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(-0.5);
        robot.leftDriveB.setPower(-0.5);
        robot.rightDriveB.setPower(0.5);}

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveB.setPower(0);
        robot.rightDriveB.setPower(0);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
       /* encoderDrive(DRIVE_SPEED,  5,  5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        telemetry.addData("Lift self to top ", "Complete. Sleep 10");
        telemetry.update();

        sleep(10);     // pause for servos to move
 
	// now lower robot
        encoderDrive(DRIVE_SPEED,  -5,  -5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
 
        telemetry.addData("Lift", "Complete");
        telemetry.update();*/
	
    }

    /* FIXME: add code here
    public int shift_left(double distance or double time)
    {
    }
     */

    /* FIXME: add code here
    public int shift_right(double distance or double time)
    {
    }
     */

    /* FIXME: add code here
    public int turn_left(double distance or double time)
    {
    }
     */

    /* FIXME: add code here
    public int turn_right(double distance or double time)
    {
    }
     */

    /* FIXME: add code here
    public int go_straight(double distance or double time)
    {
    }
     */

}

