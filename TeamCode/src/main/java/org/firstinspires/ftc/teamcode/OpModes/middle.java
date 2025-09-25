package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class {
    Telemetry telemetry = null;

    private Servo rotatorServo = null;

    public static double ROTATORSERVO_FIRST_POSITION = 0;
    public static double ROTATORSERVO_SECOND_POSITION = 0.4;
    public static double ROTATORSERVO_THIRD_POSITION = 0.8;

    public outake(HardwareMap hwmap, Telemetry telemetry){
        this.telemetry = telemetry;

        rotatorServo = hwmap.get(Servo.class, "rs");

        rotatorServo.setDirection(Servo.Direction.FORWARD);

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
}
