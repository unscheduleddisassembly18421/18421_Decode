package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@Config
public class Rotator {
    Telemetry telemetry = null;
    double position;
    double targetPosition = 0;
    public static double kp = 0.01;
    public static double kd = 0;
    public static double ki = 0;

    PIDController rotatorPower = new PIDController(kp,kd,ki);

    private final double DEGREES_PER_VOLT = 360/3.3;

    double ROTATOR_OFF = 0;

    private CRServo rotatorServo = null;
    private NormalizedColorSensor intakeColorSensor = null;
    private NormalizedColorSensor rightColorSensor = null;
    private NormalizedColorSensor leftColorSensor = null;

    private AnalogInput ai = null;

    public static double ROTATORSERVO_FIRST_POSITION = 0;
    public static double ROTATORSERVO_SECOND_POSITION = 0.75;
    public static double ROTATORSERVO_THIRD_POSITION = 0.8;
    float intakeGain = 2;
    float leftGain = 2;
    float rightGain = 2;

    float[] intakeColorHSV = new float[3];
    float[] rightColorHSV = new float[3];
    float[] leftColorHSV = new float[3];


    public Rotator(HardwareMap hwmap, Telemetry telemetry){
        this.telemetry = telemetry;

        rotatorServo = hwmap.get(CRServo.class, "rs");
        ai = hwmap.get(AnalogInput.class,"ai");
        position = getPosition();
        rotatorServo.setDirection(CRServo.Direction.REVERSE);

        intakeColorSensor = hwmap.get(NormalizedColorSensor.class, "ics");
        leftColorSensor = hwmap.get(NormalizedColorSensor.class, "leftcs");
        rightColorSensor = hwmap.get(NormalizedColorSensor.class, "rightcs");

        intakeColorSensor.setGain(intakeGain);
        leftColorSensor.setGain(leftGain);
        rightColorSensor.setGain(rightGain);

        //rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);

    }

    public void init(){
        setPower(ROTATOR_OFF);
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

    double getPosition(){
        return ai.getVoltage()*DEGREES_PER_VOLT;
    }

    public void setPower(double power){
        rotatorServo.setPower(power);
    }

    public void setPosition(double angle){
        targetPosition = angle;
    }


    public void update(){
        rotatorPower.setPIDConstants(kp,kd,ki);
        double power = rotatorPower.calculate(targetPosition,getPosition());
        setPower(power);
        telemetry.addData("Rotator Position",getPosition());
        telemetry.addData("Rotator Power",power);
    }
}
