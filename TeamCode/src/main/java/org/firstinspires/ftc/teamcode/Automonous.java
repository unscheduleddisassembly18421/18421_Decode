

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

import org.firstinspires.ftc.teamcode.OpModes.DriverControl;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name= "Automonous")
public class Automonous extends LinearOpMode {
    public enum AutoSelector {TBD}
    public AutoSelector autoSelector = AutoSelector.TBD; // hi
    DriverControl r;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));
        r.drive.localizer.setPose(redStartFar);


        while (opModeInInit()){

        }

        //Make the trajectories here
        TrajectoryActionBuilder redStartToShootingPosition = r.drive.actionBuilder(redStartFar)//firstpath
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(337.6)),Math.toRadians(337.6))
                .endTrajectory();


        TrajectoryActionBuilder redFarShootingPositionToSomething = redStartToShootingPosition.fresh()//secondpath
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(157.6)),Math.toRadians(157.6))
                .endTrajectory();


        TrajectoryActionBuilder redFarShootingPositionToSomethingtwo = redStartToShootingPosition.fresh()//thirdpath
                .splineToLinearHeading(new Pose2d(-10, 38,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(90)),Math.toRadians(335))
                .endTrajectory();







        //build trajectories

        Action RedMoveToShooting = redStartToShootingPosition.build();
        Action RedFarShootingPosition = redFarShootingPositionToSomething.build();

        //Action *NameOfPath* = nameOfPath.build();

        Action firstpath = redStartToShootingPosition.build();
        Action secondpath = redFarShootingPositionToSomething.build();
        Action thirdpath = redFarShootingPositionToSomethingtwo.build();





        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        RedMoveToShooting,
                        new SleepAction(5),
                        RedFarShootingPosition,

                        new SleepAction(5)

                )



        );
    }

}

