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
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    DcMotor leftDrive;
    DcMotor leftDriveB;
    DcMotor rightDrive;
    DcMotor rightDriveB;

    HardwarePushbot2 robot = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor sensorColor1;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance1;
    DistanceSensor sensorDistance2;

    //@Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        sensorColor1 = hardwareMap.get(ColorSensor.class, "sensor_color_distance1");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance2");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance1 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance1");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance2");
        // Send telemetry message to signify robot waiting;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues1[] = {0F, 0F, 0F};
        float hsvValues2[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values1[] = hsvValues1;
        final float values2[] = hsvValues2;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor1.red() * SCALE_FACTOR),
                (int) (sensorColor1.green() * SCALE_FACTOR),
                (int) (sensorColor1.blue() * SCALE_FACTOR),
                hsvValues1);
        Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
                (int) (sensorColor2.green() * SCALE_FACTOR),
                (int) (sensorColor2.blue() * SCALE_FACTOR),
                hsvValues2);


        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm) 1",
                String.format(Locale.US, "%.02f", sensorDistance1.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha1", sensorColor1.alpha());
        telemetry.addData("Red1  ", sensorColor1.red());
        telemetry.addData("Green1", sensorColor1.green());
        telemetry.addData("Blue1 ", sensorColor1.blue());
        telemetry.addData("Hue1", hsvValues1[0]);

        telemetry.addData("Distance (cm)2",
                String.format(Locale.US, "%.02f", sensorDistance2.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha2", sensorColor2.alpha());
        telemetry.addData("Red2  ", sensorColor2.red());
        telemetry.addData("Green2", sensorColor2.green());
        telemetry.addData("Blue2 ", sensorColor2.blue());
        telemetry.addData("Hue2", hsvValues2[0]);

        waitForStart();


    }

}

