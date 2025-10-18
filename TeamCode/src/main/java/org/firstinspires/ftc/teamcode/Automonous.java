

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

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name= "Auto ")
public class Automonous extends LinearOpMode {
    public enum AutoSelector {TBD}
    public AutoSelector autoSelector = AutoSelector.TBD; // hi
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));


        while (opModeInInit()){

        }

        //Make the trajectories here
        TrajectoryActionBuilder redStartToShootingPosition = drive.actionBuilder(redStartFar)//firstAuto
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(38, 30,Math.toRadians(90)),Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(36, 35, Math.toRadians(180)), Math.toRadians(270))
            .endTrajectory();

        TrajectoryActionBuilder redFarShootingPositionToSomething = redStartToShootingPosition.fresh();















        //build trajectories

        Action RedMoveToShooting = redStartToShootingPosition.build();
        Action RedFarShootingPosition = redFarShootingPositionToSomething.build();







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

