package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    Telemetry telemetry = null;

    //Hardware
    private DcMotor intakeMotor = null;

    public static double INTAKEMOTOR_POWER_ON = 0.7;
    public static double INTAKEMOTOR_POWER_OFF = 0;

    //Constructor
    public Intake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hwmap.get(DcMotor.class, "intakem");

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

    public class TurnOnIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotorOn();
            return false;
        }
    }

    public Action turnOnIntake(){
        return new TurnOnIntake();
    }


}
