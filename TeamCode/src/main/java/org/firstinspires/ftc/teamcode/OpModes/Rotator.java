package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@Config
public class Rotator {
    Telemetry telemetry = null;

    private Servo rotatorServo = null;
    private NormalizedColorSensor intakeColorSensor = null;
    private NormalizedColorSensor rightColorSensor = null;
    private NormalizedColorSensor leftColorSensor = null;

    public static double ROTATORSERVO_FIRST_POSITION = 0;
    public static double ROTATORSERVO_SECOND_POSITION = 0.75;
    public static double ROTATORSERVO_THIRD_POSITION = 0.8;
    float intakeGain = 2;//TODO test to see if happy with value
    float leftGain = 2;//TODO test to see if happy with value
    float rightGain = 2;//TODO test to see if happy with value

    float[] intakeColorHSV = new float[3];
    float[] rightColorHSV = new float[3];
    float[] leftColorHSV = new float[3];


    public Rotator(HardwareMap hwmap, Telemetry telemetry){
        this.telemetry = telemetry;

        rotatorServo = hwmap.get(Servo.class, "rs");

        rotatorServo.setDirection(Servo.Direction.FORWARD); //TODO test to see if we are happy

        intakeColorSensor = hwmap.get(NormalizedColorSensor.class, "ics");
        leftColorSensor = hwmap.get(NormalizedColorSensor.class, "leftcs");
        rightColorSensor = hwmap.get(NormalizedColorSensor.class, "rightcs");

        intakeColorSensor.setGain(intakeGain);
        leftColorSensor.setGain(leftGain);
        rightColorSensor.setGain(rightGain);

        //rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);

    }

    public void init(){
        rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);
    }

    public void rotatorServoFirstPosition(){
        rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);
    }

    public void rotatorServoSecondPosition(){
        rotatorServo.setPosition(ROTATORSERVO_SECOND_POSITION);
    }

    public void rotatorServoThirdPosition(){
        rotatorServo.setPosition(ROTATORSERVO_THIRD_POSITION);
    }

    public void readColorSensors(){
        NormalizedRGBA intakeColor = intakeColorSensor.getNormalizedColors();
        NormalizedRGBA rightColor = rightColorSensor.getNormalizedColors();
        NormalizedRGBA leftColor = leftColorSensor.getNormalizedColors();
        Color.colorToHSV(intakeColor.toColor(), intakeColorHSV);
        Color.colorToHSV(rightColor.toColor(), rightColorHSV);
        Color.colorToHSV(leftColor.toColor(), leftColorHSV);
        telemetry.addData("HSV intake", Arrays.toString(intakeColorHSV));
        telemetry.addData("HSV left", Arrays.toString(leftColorHSV));
        telemetry.addData("HSV right", Arrays.toString(rightColorHSV));
    }
}
