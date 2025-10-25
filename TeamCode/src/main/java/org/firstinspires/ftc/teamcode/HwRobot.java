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

    public Action openHoodServo(){
        return outtake.openHoodServo();
    }


    public Action turnToFirstShootingAngle(){
        return rotator.turnToFirstShootingAngle();
    }

    public Action turnToSecondShootingAngle(){
        return rotator.turnToSecondShootingAngle();
    }

    public Action turnToThirdShootingAngle(){
        return rotator.turnToThirdShootingAngle();
    }

    public Action turnToFirstAngle(){
        return rotator.turnToFirstAngle();
    }

    public Action turnToSecondAngle(){
        return rotator.turnToSecondAngle();
    }

    public Action turnToThirdAngle(){
        return rotator.turnToThirdAngle();
    }

    public Action turnOnIntake(){
        return intake.turnOnIntake();
    }

    //right front is in EH 0
    //right back is in EH 1
    //left back is in CH 0
    //left front is in CH 1
    //hood servo is in CH 1
    //left indicator light is in CH 5
    //right indicator light is in CH 4
    //pinpoint is in I2C bus 0 CH
    //rotater servo is in SH 0
    //elavator motor is in EH 2
    //intake motor is in EH 3
    //launcher motor 1 is in CH 3
    //launcher motor 2 is in CH 2
    //analog input is in CH analog input 0
    //right color sensor is in EH I2C 0
    //left color sensor is in CH I2C
    //intake color sensor is in CH I2C
}
