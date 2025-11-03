

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.INTAKE_OFF_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.IntakeState.INTAKE1;
import static org.firstinspires.ftc.teamcode.Variables.firstAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdAngle;

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

import org.firstinspires.ftc.teamcode.OpModes.DriverControl;

@Config
@Autonomous(name= "Automonous")
public class Automonous extends LinearOpMode {
    public enum AutoSelector {RED_FAR, RED_NEAR, BLUE_FAR, BLUE_NEAR}
    public AutoSelector autoSelector = AutoSelector.RED_FAR;
    public HwRobot r = null;
    public static double SHOOTING_DELAY = 0.45;
    public static double SELECTOR_DELAY_TIME = 0.4;

    Variables.IntakeState intakeState = Variables.IntakeState.READY;
    public ElapsedTime intakeClock = new ElapsedTime();

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


        }

        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-63, 12, Math.toRadians(0));

        Pose2d blueStartFar = new Pose2d(63, -12, Math.toRadians(180));

        Pose2d blueStartNear = new Pose2d(-63, -12, Math.toRadians(0));

        if (autoSelector == AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);
        } else if (autoSelector == AutoSelector.RED_NEAR) {
            r.drive.localizer.setPose(redStartNear);
        } else if (autoSelector == AutoSelector.BLUE_FAR) {
            r.drive.localizer.setPose(blueStartFar);
        } else {
            r.drive.localizer.setPose(blueStartNear);
        }


        //Make the trajectories here




        // RED FAR
        TrajectoryActionBuilder redFarMoveToShootingPosition = r.drive.actionBuilder(redStartFar)
                .turnTo(Math.toRadians(148))
                .endTrajectory();

        TrajectoryActionBuilder redFarFirstPath = redFarMoveToShootingPosition.fresh()//firstPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55, 15,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();


        TrajectoryActionBuilder redFarSecondPath = redFarFirstPath.fresh()//secondPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();


        TrajectoryActionBuilder redFarThirdPath = redFarSecondPath.fresh()//thirdPathFarRed
                .splineToLinearHeading(new Pose2d(-10, 38,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(50, 12,Math.toRadians(160)),Math.toRadians(160))
                .endTrajectory();

        //RED NEAR
        TrajectoryActionBuilder redNearFirstPath = r.drive.actionBuilder(redStartNear)//firstPathNearRed
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(18)
                .turnTo(Math.toRadians(145))
                .endTrajectory();

        TrajectoryActionBuilder redNearSecondPath = redNearFirstPath.fresh()//secondPathNearRed
                .strafeToLinearHeading(new Vector2d(12, 12),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(145))
                .endTrajectory();

        TrajectoryActionBuilder redNearThirdPath = redNearSecondPath.fresh()//thirdPathNearRed
                .strafeToLinearHeading(new Vector2d(36, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(135))
                .endTrajectory();

        //BLUE FAR
        TrajectoryActionBuilder blueFarMoveToShootingPosition = r.drive.actionBuilder(blueStartFar)//blueFarMoveToShootingPosition
                .turn(Math.toRadians(228))
                .endTrajectory();

        TrajectoryActionBuilder blueFarFirstPath = r.drive.actionBuilder(blueStartFar)//firstPathFarBlue
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, -35,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-55)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, -15,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        TrajectoryActionBuilder blueFarSecondPath = blueFarFirstPath.fresh()//secondPathFarBlue
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, -35,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(53, -15,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        TrajectoryActionBuilder blueFarThirdPath = blueFarSecondPath.fresh()//thirdPathFarBlue
                .splineToLinearHeading(new Pose2d(-12, -55,Math.toRadians(270)),Math.toRadians(270))
                .setTangent(210)
                .splineToLinearHeading(new Pose2d(53, -15,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        //BLUE NEAR
        TrajectoryActionBuilder blueNearFirstPath = r.drive.actionBuilder(blueStartNear)//firstPathNearBlue
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-18)
                .turnTo(Math.toRadians(220))
                .endTrajectory();

        TrajectoryActionBuilder blueNearSecondPath = blueNearFirstPath.fresh()//secondPathNearBlue
                .strafeToLinearHeading(new Vector2d(12, -12),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-12)
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270))
                .turnTo(Math.toRadians(220))
                .endTrajectory();

        TrajectoryActionBuilder blueNearThirdPath = blueNearSecondPath.fresh()//thirdPathNearBlue
                .strafeToLinearHeading(new Vector2d(36, -12), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-12)
                .setTangent(Math.toRadians(220))
                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(220))
                .endTrajectory();

        //build trajectories
        //Action *NameOfPath* = nameOfPath.build();

        switch (intakeState){
            case READY:
                r.rotator.leftLightRed();
                r.rotator.rightLightRed();
                r.rotator.setPosition(firstAngle);
                r.intake.intakeMotorOn();
                intakeState = Variables.IntakeState.INTAKE1;

                break;

            case INTAKE1:
                if(r.rotator.detectedBall()){
                    intakeState = Variables.IntakeState.INTAKE2;
                    intakeClock.reset();
                }
                break;
            case INTAKE2:
                r.rotator.setPosition(secondAngle);
                if(intakeClock.milliseconds() > INTAKE_DELAY && r.rotator.detectedBall()){
                    intakeClock.reset();
                    intakeState = Variables.IntakeState.INTAKE3;
                }
                break;

            case INTAKE3:
                r.rotator.setPosition(thirdAngle);
                if(intakeClock.milliseconds() > INTAKE_DELAY && r.rotator.detectedBall()){
                    intakeClock.reset();
                    intakeState = Variables.IntakeState.FULL;
                }
                break;

            case FULL:
                if(intakeClock.milliseconds() > INTAKE_OFF_DELAY) {
                    r.intake.intakeMotorOff();
                    r.rotator.leftLightGreen();
                    r.rotator.rightLightGreen();
                    intakeState = Variables.IntakeState.FIRING;
                }

                break;
        }


        //RED FAR
        Action RedFarGoToShootingPosition = redFarMoveToShootingPosition.build();
        Action RedFarMoveToShootingFirstPath = redFarFirstPath.build();
        Action RedFarMoveToShootingSecondPath = redFarSecondPath.build();
        Action RedFarMoveToShootingThirdPath = redFarThirdPath.build();
        //RED NEAR
        Action RedNearMoveToShootingFirstPath = redNearFirstPath.build();
        Action RedNearMoveToShootingSecondPath = redNearSecondPath.build();
        Action RedNearMoveToShootingThirdPath = redNearThirdPath.build();
        //BLUE FAR
        Action BlueFarGoToShootingPosition = blueFarMoveToShootingPosition.build();
        Action BlueFarMoveToShootingFirstPath = blueFarFirstPath.build();
        Action BlueFarMoveToShootingSecondPath = blueFarSecondPath.build();
        Action BlueFarMoveToShootingThirdPath = blueFarThirdPath.build();
        //BLUE NEAR
        Action BlueNearMoveToShootingFirstPath = blueNearFirstPath.build();
        Action BlueNearMoveToShootingSecondPath = blueNearSecondPath.build();
        Action BlueNearMoveToShootingThirdPath = blueNearThirdPath.build();




        waitForStart();

        if (autoSelector == AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new SequentialAction(
                            RedFarGoToShootingPosition,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            RedFarMoveToShootingFirstPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            RedFarMoveToShootingSecondPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            RedFarMoveToShootingThirdPath,
                            shoot()


                    )
            );
        }
        else if (autoSelector == AutoSelector.RED_NEAR) {
            Actions.runBlocking(
                    new SequentialAction(

                            new SleepAction(1),
                            r.turnOnIntake(),
                            RedNearMoveToShootingFirstPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            RedNearMoveToShootingSecondPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            RedNearMoveToShootingThirdPath,
                            shoot()

                    )
            );

        }
        else if (autoSelector == AutoSelector.BLUE_FAR) {
            Actions.runBlocking(
                    new SequentialAction(
                            BlueFarGoToShootingPosition,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            BlueFarMoveToShootingFirstPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            BlueFarMoveToShootingSecondPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            BlueFarMoveToShootingThirdPath,
                            shoot()
                    )
            );


        }
        else if (autoSelector == AutoSelector.BLUE_NEAR){
            Actions.runBlocking(
                    new SequentialAction(

                            new SleepAction(1),
                            r.turnOnIntake(),
                            RedNearMoveToShootingFirstPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            BlueNearMoveToShootingSecondPath,
                            shoot(),

                            new SleepAction(1),
                            r.turnOnIntake(),
                            BlueNearMoveToShootingThirdPath,
                            shoot()
                    )
            );

        }else
            r.drive.localizer.setPose(blueStartNear);




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

