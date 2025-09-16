package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class outake {

    Telemetry telemetry = null;

    //hardware
    private DcMotorEx launcherMotor = null;
    private Servo shooterServo = null;
    private Servo rotatorServo = null;

    //Positions
    public static double SHOOTERSERVO_START_POSITION  = 0;
    public static double SHOOTERSERVO_FIRING_POSITION = 0.8;
    public static double LAUNCHERMOTOR_VELOCITY_ON = 9000;
    public static double ROTATORSERVO_FIRST_POSITION = 0;
    public static double ROTATORSERVO_SECOND_POSITION = 0.4;
    public static double ROTATORSERVO_THIRD_POSITION = 0.8;

    //constructor
    public outake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooterServo = hwmap.get(Servo.class, "ss");
        launcherMotor = hwmap.get(DcMotorEx.class, "lm");
        rotatorServo = hwmap.get(Servo.class, "ss");

        shooterServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        rotatorServo.setDirection(Servo.Direction.FORWARD);

        shooterServo.setPosition(SHOOTERSERVO_START_POSITION);
        rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    //commands
    public void shooterServoFire(){
        shooterServo.setPosition(SHOOTERSERVO_FIRING_POSITION);
    }

    public void shooterServoOpen(){
        shooterServo.setPosition(SHOOTERSERVO_START_POSITION);
    }

    public void launcherMotorOn(){
        launcherMotor.setVelocity(LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotorOff(){
        launcherMotor.setVelocity(0);
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
