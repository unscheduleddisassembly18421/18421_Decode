

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.DriverControl;
import org.firstinspires.ftc.teamcode.OpModes.Outtake;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name= "Automonous")
public class Automonous extends LinearOpMode {
    public enum AutoSelector {TBD}
    public AutoSelector autoSelector = AutoSelector.TBD; // hi
    public HwRobot r = null;
    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(0, 0, 0);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        r = new HwRobot(telemetry,hardwareMap);
        r.init();


        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));
        r.drive.localizer.setPose(redStartFar);


        while (opModeInInit()){

        }

        //Make the trajectories here
        TrajectoryActionBuilder redStartToShootingPosition = r.drive.actionBuilder(redStartFar)//firstPath
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55, 15,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();


        TrajectoryActionBuilder redFarShootingPositionToSomething = redStartToShootingPosition.fresh()//secondPath
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();


        TrajectoryActionBuilder redFarShootingPositionToSomethingTwo = redFarShootingPositionToSomething.fresh()//thirdPath
                .splineToLinearHeading(new Pose2d(-10, 38,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(90)),Math.toRadians(335))
                .endTrajectory();
        //TrajectoryActionBuilder redFarShootingPositionToSomethingthree =




        //build trajectories

        Action RedMoveToShooting = redStartToShootingPosition.build();
        Action RedFarShootingPosition = redFarShootingPositionToSomething.build();


        //Action *NameOfPath* = nameOfPath.build();

        Action firstPath = redStartToShootingPosition.build();
        Action secondPath = redFarShootingPositionToSomething.build();
        Action thirdPath = redFarShootingPositionToSomethingTwo.build();
        //Action fourthpath =




        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        r.activateShooter(),
                        RedMoveToShooting,
                        r.checkShooterVelocity(),
                        r.turnElavatorMotorOn(),
                        r.turnToFirstShootingAngle(),
                        r.turnToSecondShootingAngle(),
                        r.turnToThirdShootingAngle(),
                        firstPath,
                        secondPath,
                        thirdPath,
                        new SleepAction(5)

                )



        );
    }

}

