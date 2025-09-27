package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class Outake {

    Telemetry telemetry = null;

    //hardware
    private  DcMotorEx launcherMotor = null;
    private DcMotor elavatorMotor = null;
    private Servo hoodServo = null;

    //Positions
    public static double HOODSERVO_START_POSITION  = 0;
    public static double LAUNCHERMOTOR_VELOCITY_ON = 9000;
    public static double ELAVATORMOTOR_POWER_ON = 1;


    //constructor
    public Outake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        hoodServo = hwmap.get(Servo.class, "hs");
        launcherMotor = hwmap.get(DcMotorEx.class, "Lm");
        elavatorMotor = hwmap.get(DcMotor.class, "em");

        hoodServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        elavatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo.setPosition(HOODSERVO_START_POSITION);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elavatorMotor.setPower(0);
    }

    public void init(){
        hoodServo.setPosition(HOODSERVO_START_POSITION);
    }


    //commands
    public void launcherMotorOn(){
        launcherMotor.setVelocity(LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotorOff(){
        launcherMotor.setVelocity(0);
    }

    public void elavatorMotorON(){
        elavatorMotor.setPower(ELAVATORMOTOR_POWER_ON);
    }

    public void elavatorMotorOff(){
        elavatorMotor.setPower(0);
    }



}
