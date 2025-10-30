

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
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name= "Automonous")
public class Automonous extends LinearOpMode {
    public enum AutoSelector {RED_FAR, RED_NEAR, BLUE_FAR, BLUE_NEAR}
    public AutoSelector autoSelector = AutoSelector.RED_FAR;
    public HwRobot r = null;
    public static double SHOOTING_DELAY = 0.45;
    public static double SELECTOR_DELAY_TIME = 0.4;
            private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(0, 0, 0);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        r = new HwRobot(telemetry, hardwareMap);
        r.init();


        while (opModeInInit()) {

            double time = runtime.seconds();
            telemetry.addData(" AUTO SELECTED", autoSelector);
            telemetry.addLine("D-Pad Up for Red Far");
            telemetry.addLine("D-Pad Right for Red Near");
            telemetry.addLine("D-Pad Down for Blue Far");
            telemetry.addLine("D-Pad Left for Blue Near");

            if (gamepad1.dpad_up) {

                autoSelector = AutoSelector.RED_FAR;

            } else if (gamepad1.dpad_right) {

                autoSelector = AutoSelector.RED_NEAR;

            } else if (gamepad1.dpad_down) {

                autoSelector = AutoSelector.BLUE_FAR;

            } else if (gamepad1.dpad_left) {

                autoSelector = AutoSelector.BLUE_NEAR;
            }
            telemetry.update();


        }

        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-63, 12, Math.toRadians(0));

        Pose2d blueStartFar = new Pose2d(63, -12, Math.toRadians(180));

        Pose2d blueStartNear = new Pose2d(-63, -12, Math.toRadians(0));

       //autoSelector = AutoSelector.RED_FAR;

        if (autoSelector == AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);
        } else if (autoSelector == AutoSelector.RED_NEAR) {
            r.drive.localizer.setPose(redStartNear);
        } else if (autoSelector == AutoSelector.BLUE_FAR) {
            r.drive.localizer.setPose(blueStartFar);
        } else if (autoSelector == AutoSelector.BLUE_NEAR) {
            r.drive.localizer.setPose(blueStartNear);
        } //maybe use else is fine?  Check this later


        //Make the trajectories here




        // RED FAR
        TrajectoryActionBuilder redFarToShootingPositionFirstPath = r.drive.actionBuilder(redStartFar)//firstPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55, 15,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();


        TrajectoryActionBuilder redFarToShootingPositionSecondPath = redFarToShootingPositionFirstPath.fresh()//secondPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();


        TrajectoryActionBuilder redFarToShootingPositionThirdPath = redFarToShootingPositionSecondPath.fresh()//thirdPathFarRed
                .splineToLinearHeading(new Pose2d(-10, 38,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(50, 12,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();

        //RED NEAR
        TrajectoryActionBuilder redNearShootingPositionFirstPath = r.drive.actionBuilder(redStartNear)//firstPathNearRed
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(18)
                .turnTo(Math.toRadians(145))
                .endTrajectory();

        TrajectoryActionBuilder redNearShootingPositionSecondPath = redNearShootingPositionFirstPath.fresh()//secondPathNearRed
                .strafeToLinearHeading(new Vector2d(12, 12),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(145))
                .endTrajectory();

        TrajectoryActionBuilder redNearShootingPositionThirdPath = redNearShootingPositionSecondPath.fresh()//thirdPathNearRed
                .strafeToLinearHeading(new Vector2d(36, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(135))
                .endTrajectory();

        //BLUE FAR
        TrajectoryActionBuilder blueFarShootingPositionFirstPath = r.drive.actionBuilder(blueStartFar)//firstPathFarBlue
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, -35,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-55)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, -15,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        TrajectoryActionBuilder blueFarShootingPositionSecondPath = blueFarShootingPositionFirstPath.fresh()//secondPathFarBlue
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, -35,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(53, -15,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        TrajectoryActionBuilder blueFarShootingPositionThirdPath = blueFarShootingPositionSecondPath.fresh()//thirdPathFarBlue
                .splineToLinearHeading(new Pose2d(-12, -55,Math.toRadians(270)),Math.toRadians(270))
                .setTangent(210)
                .splineToLinearHeading(new Pose2d(53, -15,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        //BLUE NEAR
        TrajectoryActionBuilder blueNearShootingPositionFirstPath = r.drive.actionBuilder(blueStartNear)//firstPathNearBlue
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-18)
                .turnTo(Math.toRadians(220))
                .endTrajectory();

        TrajectoryActionBuilder blueNearShootingPositionSecondPath = blueNearShootingPositionFirstPath.fresh()//secondPathNearBlue
                .strafeToLinearHeading(new Vector2d(12, -12),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-12)
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270))
                .turnTo(Math.toRadians(220))
                .endTrajectory();

        TrajectoryActionBuilder blueNearShootingPositionThirdPath = blueNearShootingPositionSecondPath.fresh()//thirdPathNearBlue
                .strafeToLinearHeading(new Vector2d(36, -12), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-12)
                .setTangent(Math.toRadians(220))
                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(220))
                .endTrajectory();

        //build trajectories

        //RED FAR
        Action RedFarMoveToShootingFirstPath = redFarToShootingPositionFirstPath.build();
        Action RedFarMoveToShootingSecondPath = redFarToShootingPositionSecondPath.build();
        Action RedFarMoveToShootingThirdPath = redFarToShootingPositionThirdPath.build();
        //RED NEAR
        Action RedNearMoveToShootingFirstPath = redNearShootingPositionFirstPath.build();
        Action RedNearMoveToShootingSecondPath = redNearShootingPositionSecondPath.build();
        Action RedNearMoveToShootingThirdPath = redNearShootingPositionThirdPath.build();
        //BLUE FAR
        Action BlueFarMoveToShootingFirstPath = blueFarShootingPositionFirstPath.build();
        Action BlueFarMoveToShootingSecondPath = blueFarShootingPositionSecondPath.build();
        Action BlueFarMoveToShootingThirdPath = blueFarShootingPositionThirdPath.build();
        //BLUE NEAR
        Action BlueNearMoveToShootingFirstPath = blueNearShootingPositionFirstPath.build();
        Action BlueNearMoveToShootingSecondPath = blueNearShootingPositionSecondPath.build();
        Action BlueNearMoveToShootingThirdPath = blueNearShootingPositionThirdPath.build();




        //Action *NameOfPath* = nameOfPath.build();

        //RED FAR
        Action firstPathRedFar = redFarToShootingPositionFirstPath.build();
        Action secondPathRedFar = redFarToShootingPositionSecondPath.build();
        Action thirdPathRedFar = redFarToShootingPositionThirdPath.build();
        //RED NEAR
        Action firstPathRedNear = redNearShootingPositionFirstPath.build();
        Action secondPathRedNear = redNearShootingPositionSecondPath.build();
        Action thirdPathRedNear = redNearShootingPositionThirdPath.build();
        //BLUE FAR
        Action firstPathBlueFar = blueFarShootingPositionFirstPath.build();
        Action secondPathBlueFar = blueFarShootingPositionSecondPath.build();
        Action thirdPathBlueFar = blueFarShootingPositionThirdPath.build();
        //BLUE NEAR
        Action firstPathBlueNear = blueNearShootingPositionFirstPath.build();
        Action secondPathBlueNear = blueNearShootingPositionSecondPath.build();
        Action thirdPathBlueNear = blueNearShootingPositionThirdPath.build();


        waitForStart();

        if (autoSelector == AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new SequentialAction(
                            shoot(),

                            new SleepAction(5),
                            RedFarMoveToShootingFirstPath,
                            RedFarMoveToShootingSecondPath,
                            RedFarMoveToShootingThirdPath

                            //r.checkShooterVelocity(),
                            //r.openHoodServo(),
                            //new SleepAction(5),

                            //r.checkShooterVelocity(),
                            //r.openHoodServo()

                    )
            );

        }
        else if (autoSelector == AutoSelector.RED_NEAR) {
            Actions.runBlocking(
                    new SequentialAction(

                            new SleepAction(5),
                            RedNearMoveToShootingFirstPath,
                            RedNearMoveToShootingSecondPath,
                            RedNearMoveToShootingThirdPath
                    )
            );

        }
        else if (autoSelector == AutoSelector.BLUE_FAR) {
            Actions.runBlocking(
                    new SequentialAction(

                            new SleepAction(5),
                            BlueFarMoveToShootingFirstPath,
                            BlueFarMoveToShootingSecondPath,
                            BlueFarMoveToShootingThirdPath
                    )
            );

        }
        else if(autoSelector == AutoSelector.BLUE_NEAR){
            Actions.runBlocking(
                    new SequentialAction(

                            new SleepAction(5),
                            BlueNearMoveToShootingFirstPath,
                            BlueNearMoveToShootingSecondPath,
                            BlueNearMoveToShootingThirdPath
                    )
            );

        }
        else
            r.drive.localizer.setPose(blueStartNear);
        }


    {


    }
    public Action shoot(){
        return new SequentialAction(
                r.activateShooter(),
                r.checkShooterVelocity(),
                r.openHoodServo(),
                r.turnElavatorMotorOn(),
                r.turnToFirstShootingAngle(),
                new SleepAction(SHOOTING_DELAY),
                r.turnToSecondShootingAngle(),
                new SleepAction(SHOOTING_DELAY),
                r.turnToThirdShootingAngle(),
                new SleepAction(SHOOTING_DELAY)
        );
    }
}

