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
    private  DcMotorEx launcherMotor1 = null;
    private DcMotorEx launcherMotor2 = null;
    private DcMotor elavatorMotor = null;
    private Servo hoodServo1 = null;
    private Servo hoodServo2 = null;

    //Positions
    public static double HOODSERVO_START_POSITION  = 0;
    public static double HOODSERVO_SHOOT_POSITION = 0.5;
    public static double LAUNCHERMOTOR_VELOCITY_ON = 15000;
    public static double ELAVATORMOTOR_POWER_ON = 1;


    //constructor
    public Outake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        hoodServo1 = hwmap.get(Servo.class, "hs1 ");
        hoodServo2 = hwmap.get(Servo.class, "hs2");
        launcherMotor1 = hwmap.get(DcMotorEx.class, "Lm1");
        launcherMotor2 = hwmap.get(DcMotorEx.class, "Lm2");
        elavatorMotor = hwmap.get(DcMotor.class, "em");

        hoodServo1.setDirection(Servo.Direction.FORWARD);
        launcherMotor1.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor1.setDirection(DcMotor.Direction.FORWARD);
        elavatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo1.setPosition(HOODSERVO_START_POSITION);

        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elavatorMotor.setPower(0);
    }

    public void init(){
        hoodServo1.setPosition(HOODSERVO_START_POSITION);
    }


    //commands
    public void launcherMotor1On(){
        launcherMotor1.setVelocity(LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor1Off(){
        launcherMotor1.setVelocity(0);
    }

    public void  launcherMotor2On(){
        launcherMotor2.setVelocity(LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor2Off(){
        launcherMotor2.setVelocity(0);
    }

    public void elavatorMotorON(){
        elavatorMotor.setPower(ELAVATORMOTOR_POWER_ON);
    }

    public void elavatorMotorOff(){
        elavatorMotor.setPower(0);
    }

    public void  hoodServoStart(){
        hoodServo1.setPosition(HOODSERVO_START_POSITION);
    }


    public void  hoodServoShoot(){
        hoodServo1.setPosition(HOODSERVO_SHOOT_POSITION);
    }

}
