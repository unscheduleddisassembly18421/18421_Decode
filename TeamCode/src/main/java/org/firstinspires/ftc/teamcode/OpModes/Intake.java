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
    private Servo lidServo = null;
    private DcMotor conveyorMotor = null;


    //Positions
    public static double LIDSERVO_CLOSED_POSITION = 0;
    public static double LIDSERVO_OPEN_POSITION = 0.5;
    public static double LAUNCHERMOTOR_VELOCITY_ON = 9000;
    public static double INTAKEMOTOR_POWER_ON = 0.5;
    public static double CONVEYORMOROR_POWER_ON = 0.5;


    //Constructor
    public Intake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;
        lidServo = hwmap.get(Servo.class, "ls");
        launcherMotor = hwmap.get(DcMotorEx.class, "lm");
        intakeMotor = hwmap.get(DcMotor.class, "im");
        conveyorMotor = hwmap.get(DcMotor.class, "cm");


        lidServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);

        lidServo.setPosition(LIDSERVO_CLOSED_POSITION);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        launcherMotorOff();
        intakeMotorOff();
        setConveyorMotorOff();
    }

    public void lidServoOpen(){

        lidServo.setPosition(LIDSERVO_OPEN_POSITION);
    }

    public void lidServoClosed(){
        lidServo.setPosition(LIDSERVO_CLOSED_POSITION);
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

    public void conveyorMotorOn(){
        conveyorMotor.setPower(CONVEYORMOROR_POWER_ON);
    }

    public void setConveyorMotorOff(){
        conveyorMotor.setPower(0);
    }

}
