package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    Telemetry telemetry = null;

    //Hardware
    private DcMotor intakeMotor = null;
    private DcMotorEx launcherMotor = null;
    private Servo shooterServo = null;


    //Positions
    public static double SHOOTERSERVO_START_POSITION = 0;
    public static double SHOOTERSERVO_FIRING_POSITION = 0.25;
    public static double LAUNCHERMOTOR_VELOCITY_ON = 9000;
    public static double INTAKEMOTOR_POWER_ON = 0.5;


    //Constructor
    public Intake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooterServo = hwmap.get(Servo.class, "ss");
        launcherMotor = hwmap.get(DcMotorEx.class, "lm");
        intakeMotor = hwmap.get(DcMotor.class, "im");


        shooterServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterServo.setPosition(SHOOTERSERVO_START_POSITION);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        launcherMotorOff();
        intakeMotorOff();
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

    public void intakeMotorOn(){
        intakeMotor.setPower(INTAKEMOTOR_POWER_ON);
    }

    public void intakeMotorOff(){
        intakeMotor.setPower(0);
    }


}
