package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    Telemetry telemetry = null;

    //Hardware
    private DcMotor intakeMotor = null;

    public static double INTAKEMOTOR_POWER_ON = 0.5;
    public static double INTAKEMOTOR_POWER_OFF = 0;


    //Constructor
    public Intake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hwmap.get(DcMotor.class, "im");

        //initial directions and positions
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotorOff();
    }


    public void intakeMotorOff(){
        intakeMotor.setPower(INTAKEMOTOR_POWER_OFF);
    }

    public void intakeMotorOn(){
        intakeMotor.setPower(INTAKEMOTOR_POWER_ON);
    }


}
