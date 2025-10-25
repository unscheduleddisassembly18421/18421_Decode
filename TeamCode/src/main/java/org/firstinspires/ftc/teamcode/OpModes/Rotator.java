package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Config
public class Rotator {
    Telemetry telemetry = null;
    double position;
    double targetPosition = 0;
    public static double kp = 0.005;
    public static double kd = 0;
    public static double ki = 0;


    public static double firstAngle = 25;
    public static double secondAngle = 145;
    public static double thirdAngle = 267;
    public static double firstShootingAngle = 205;
    public static double secondShootingAngle = 332;
    public static double thirdShootingAngle = 90;

    public static double intakeTolerance = 6;

    PIDController rotatorPower = new PIDController(kp,kd,ki);

    private final double DEGREES_PER_VOLT = 360/3.3;

    double ROTATOR_OFF = 0;

    private CRServo rotatorServo = null;
    private NormalizedColorSensor intakeColorSensor = null;
    private NormalizedColorSensor rightColorSensor = null;
    private NormalizedColorSensor leftColorSensor = null;

    private Servo leftLight = null;
    private Servo rightLight = null;

    public static double GREEN_COLOR = 0.5;
    public static double RED_COLOR = 0.277;

    private AnalogInput ai = null;

    float intakeGain = 4;
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

        rightLight = hwmap.get(Servo.class, "rightil");
        leftLight = hwmap.get(Servo.class, "leftil");

        rightLight.setDirection(Servo.Direction.FORWARD);
        leftLight.setDirection(Servo.Direction.FORWARD);

        intakeColorSensor.setGain(intakeGain);
        leftColorSensor.setGain(leftGain);
        rightColorSensor.setGain(rightGain);

        //rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);

    }

    public void init(){
        setPower(ROTATOR_OFF);
    }

    //color sensor stuff
    public void readColorSensors(){
        double intakeDistance = ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM);
        NormalizedRGBA intakeColor = intakeColorSensor.getNormalizedColors();
        NormalizedRGBA rightColor = rightColorSensor.getNormalizedColors();
        NormalizedRGBA leftColor = leftColorSensor.getNormalizedColors();
        Color.colorToHSV(intakeColor.toColor(), intakeColorHSV);
        Color.colorToHSV(rightColor.toColor(), rightColorHSV);
        Color.colorToHSV(leftColor.toColor(), leftColorHSV);
        telemetry.addData("HSV intake", Arrays.toString(intakeColorHSV));
        telemetry.addData("HSV left", Arrays.toString(leftColorHSV));
        telemetry.addData("HSV right", Arrays.toString(rightColorHSV));
        telemetry.addData("intake distance", intakeDistance);
    }

    public boolean detectedBall(){
        return ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) < intakeTolerance;
    }

    //PID servo stuff
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

    //indicator light stuff
    public void leftLightGreen(){
        leftLight.setPosition(GREEN_COLOR);
    }

    public void leftLightRed(){
        leftLight.setPosition(RED_COLOR);
    }

    public void rightLightGreen(){
        rightLight.setPosition(GREEN_COLOR);
    }

    public void rightLightRed(){
        rightLight.setPosition(RED_COLOR);
    }


    //actions
    public class turnToFirstShootingAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPosition(firstShootingAngle);
            return false;
        }
    }
    public Action turnToFirstShootingAngle(){
        return new turnToFirstShootingAngle();
    }

    public class turnToSecondShootingAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPosition(secondShootingAngle);
            return false;
        }
    }
    public Action turnToSecondShootingAngle(){
        return new turnToFirstShootingAngle();
    }

    public class turnToThirdShootingAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPosition(thirdShootingAngle);
            return false;
        }
    }
    public Action turnToThirdShootingAngle(){
        return new turnToFirstShootingAngle();
    }

    public class TurnToFirstAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPosition(firstAngle);
            return false;
        }
    }
    public Action turnToFirstAngle(){
        return new TurnToFirstAngle();
    }

    public class TurnToSecondAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPosition(secondAngle);
            return false;
        }
    }
    public Action turnToSecondAngle(){
        return new TurnToSecondAngle();
    }

    public class TurnToThirdAngle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPosition(thirdAngle);
            return false;
        }
    }
    public Action turnToThirdAngle(){
        return new TurnToThirdAngle();
    }
}
