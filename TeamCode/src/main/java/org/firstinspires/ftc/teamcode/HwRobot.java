package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Intake;
import org.firstinspires.ftc.teamcode.OpModes.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.Rotator;

public class HwRobot {
    public MecanumDrive drive;
    public Intake intake = null;
    public Outtake outtake = null;
    public Rotator rotator = null;
    Telemetry telemetry = null;
    HardwareMap hardwareMap = null;
    Pose2d BlueWallRight = new Pose2d(0,0,0);

    public HwRobot(Telemetry t, HardwareMap hwm){
        hardwareMap = hwm;
        telemetry = t;
    }

    public void init(){
        drive = new MecanumDrive(hardwareMap, BlueWallRight);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        rotator = new Rotator(hardwareMap, telemetry);
        rotator.init();
        outtake.init();
    }

    public Action activateShooter(){
        return outtake.activateShooter();
    }

    public Action checkShooterVelocity(){
        return outtake.checkShooterVelocity();
    }

    public Action turnElavatorMotorOn(){
        return outtake.turnElavatorMotorOn();
    }


    public Action turnToFirstShootingAngle(){
        return rotator.turnToFirstShootingAngle();
    }
}
