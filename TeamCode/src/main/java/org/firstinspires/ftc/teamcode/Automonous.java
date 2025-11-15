

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.INTAKE_OFF_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.firstAngle;
import static org.firstinspires.ftc.teamcode.Variables.firstShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdShootingAngle;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

    public enum IntakeState {
        READY, INTAKE1, INTAKE2, INTAKE3, FULL, FIRING
    }

    IntakeState intakeState = IntakeState.READY;
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
            telemetry.update();

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
                .lineToX(56)
                .turnTo(Math.toRadians(160))
                .endTrajectory();

        TrajectoryActionBuilder redFarFirstPath = redFarMoveToShootingPosition.fresh()//firstPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55, 15,Math.toRadians(155)),Math.toRadians(155))
                .endTrajectory();


        TrajectoryActionBuilder redFarSecondPath = redFarFirstPath.fresh()//secondPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, 35,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(155)),Math.toRadians(155))
                .endTrajectory();


        TrajectoryActionBuilder redFarThirdPath = redFarSecondPath.fresh()//thirdPathFarRed
                .splineToLinearHeading(new Pose2d(-10, 38,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(50, 12,Math.toRadians(155)),Math.toRadians(155))
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
//                    new TranslationalVelConstraint(40)
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(145))
                .endTrajectory() ;

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
                .splineToLinearHeading(new Pose2d(55, -12,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        TrajectoryActionBuilder blueFarSecondPath = blueFarFirstPath.fresh()//secondPathFarBlue
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, -35,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(53, -12,Math.toRadians(210)),Math.toRadians(210))
                .endTrajectory();

        TrajectoryActionBuilder blueFarThirdPath = blueFarSecondPath.fresh()//thirdPathFarBlue
                .splineToLinearHeading(new Pose2d(-10, -38,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-55)
                .splineToLinearHeading(new Pose2d(55, -15,Math.toRadians(205)),Math.toRadians(205))
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


        //switch (intakeState){
           // case READY:
                //r.rotator.leftLightRed();
                //r.rotator.rightLightRed();
                //r.rotator.setPosition(firstAngle);
                //r.intake.intakeMotorOn();
                //intakeState = IntakeState.INTAKE1;

                //break;

            //case INTAKE1:
               // if(r.rotator.detectedBall()){
                    //intakeState = IntakeState.INTAKE2;
                    //intakeClock.reset();
                //}
                //break;
           // case INTAKE2:
               // r.rotator.setPosition(secondAngle);
    //            if(intakeClock.milliseconds() > INTAKE_DELAY && r.rotator.detectedBall()){
      //              intakeClock.reset();
        //            intakeState = IntakeState.INTAKE3;
          //      }
            //    break;

     //       case INTAKE3:
     //           r.rotator.setPosition(thirdAngle);
     //           if(intakeClock.milliseconds() > INTAKE_DELAY && r.rotator.detectedBall()){
     //               intakeClock.reset();
     //               intakeState = IntakeState.FULL;
     //           }
     //           break;

     //       case FULL:
     //           if(intakeClock.milliseconds() > INTAKE_OFF_DELAY) {
     //               r.intake.intakeMotorOff();
      //              r.rotator.leftLightGreen();
     //               r.rotator.rightLightGreen();
     //               intakeState = IntakeState.FIRING;
     //           }

       //         break;
        //}

        //build trajectories
        //Action *NameOfPath* = nameOfPath.build();


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
                    new ParallelAction(
                        r.updateRotator(),
//                        new SequentialAction(
//                                new InstantAction(()->r.rotator.setPosition(firstShootingAngle)),
//                                new SleepAction(2),
//                                new InstantAction(()->r.rotator.setPosition(thirdShootingAngle)),
//                                new SleepAction(2),
//                                new InstantAction(()->r.rotator.setPosition(secondShootingAngle))


                          new SequentialAction(//can do turn to first angle here to speed up time
                              RedFarGoToShootingPosition,
                              shoot(),

                              new SleepAction(1),
                                new ParallelAction(
                                    RedFarMoveToShootingFirstPath,
                                    intake()
                                ),
                                shoot(),

                                new SleepAction(1),
                                new ParallelAction(
                                        RedFarMoveToShootingSecondPath,
                                        intake()
                                ),
                                 shoot(),

                               new SleepAction(1),
                                new ParallelAction(
                                        RedFarMoveToShootingThirdPath,
                                        intake()
                              ),
                               shoot()


                        )
                    )
                    );

        }
        //Im pushing htis again
        else if (autoSelector == AutoSelector.RED_NEAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.updateRotator(),
                            new SequentialAction(
                                    RedFarGoToShootingPosition,
                                    nearShoot(),
                                    new SleepAction(1),
                                    new ParallelAction(
                                            intake(),
                                            RedNearMoveToShootingFirstPath
                                    ),

                            new SleepAction(1),

                            nearShoot(),
                            new SleepAction(1),

                            new ParallelAction(
                                    intake(),
                                    RedNearMoveToShootingSecondPath
                            ),

                            new SleepAction(1),

                            nearShoot(),
                            new SleepAction(1),
                            new ParallelAction(
                                    intake(),
                                    RedNearMoveToShootingThirdPath
                            ),
                            new SleepAction(1),
                            nearShoot()

                    ))
            );

        }
        else if (autoSelector == AutoSelector.BLUE_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.updateRotator(),
                            new SequentialAction(
                                    //BlueFarGoToShootingPosition,
                                    shoot(),
                                    new SleepAction(1),

                                new ParallelAction(

                                        BlueFarMoveToShootingFirstPath,
                                        shoot()
                                ),


                                    new SleepAction(1),

                                new ParallelAction(
                                    intake(),
                                    BlueFarMoveToShootingSecondPath
                                ),
                                new SleepAction(1),
                                shoot(),

                                new SleepAction(1),

                                new ParallelAction(
                                        intake(),
                                        BlueFarMoveToShootingThirdPath
                                ),
                                new SleepAction(1),
                                shoot()
                            )
            ));


        }
        else if (autoSelector == AutoSelector.BLUE_NEAR){
            Actions.runBlocking(
                    new ParallelAction(
                            r.updateRotator(),
                            new SequentialAction(
                                    BlueFarGoToShootingPosition,
                                    nearShoot(),
                                    new SleepAction(1),
                                    new ParallelAction(
                                           intake(),
                                           BlueFarMoveToShootingFirstPath
                                    ),

                            new SleepAction(1),
                            nearShoot(),

                            new SleepAction(1),

                            new ParallelAction(
                                    intake(),
                                    BlueNearMoveToShootingSecondPath
                            ),

                            new SleepAction(1),
                            nearShoot(),
                            new SleepAction(1),

                            new ParallelAction(
                                    intake(),
                                    BlueNearMoveToShootingThirdPath
                            ),
                            new SleepAction(1),
                            nearShoot()
                    )));

        }else
            r.drive.localizer.setPose(blueStartNear);




    }
    //TODO figure out why shoot, shoot, wait, shoot
    public Action shoot(){
        return new SequentialAction(
                r.turnToSecondAngle(),
                r.activateShooter(),
                r.openHoodServo(),
                new SleepAction(0.6),
                r.turnElavatorMotorOn(),
                r.checkShooterVelocity(),
                r.turnToFirstShootingAngle(),
                new SleepAction(0.4), //.45
                r.checkShooterVelocity(),
                r.turnToThirdShootingAngle(),
                new SleepAction(0.4),
                r.checkShooterVelocity(),
                r.turnToSecondShootingAngle(),
                new SleepAction(0.4),
                r.checkShooterVelocity(),
                //r.turnOffShooter(),
                new SleepAction(0.5),
                r.turnElavatorMotorOff(),
                r.closeHoodServo()
        );
    }

    public Action nearShoot(){
        return new SequentialAction(
                r.turnToSecondAngle(),
                r.activateShooterNear(),
                r.openHoodServoNear(),
                new SleepAction(0.4),
                r.turnElavatorMotorOn(),
                r.checkShooterVelocityNear(),
                r.turnToFirstShootingAngle(),
                new SleepAction(0.4), //.45
                r.checkShooterVelocityNear(),
                r.turnToThirdShootingAngle(),
                new SleepAction(0.4),
                r.checkShooterVelocity(),
                r.turnToSecondShootingAngle(),
                new SleepAction(0.4),
                r.checkShooterVelocityNear(),
                //r.turnOffShooter(),
                new SleepAction(0.5),
                r.turnElavatorMotorOff(),
                r.closeHoodServo()

        );
    }
    
    public Action intake(){
        return new SequentialAction(
                r.turnOnIntake(),
                r.turnToFirstAngle(),
                new SleepAction(0.2),
                r.waitForBall(),
                new SleepAction(0.2),
                r.turnToThirdAngle(),
                new SleepAction(0.2),
                r.waitForBall(),
                new SleepAction(0.2),
                r.turnToSecondAngle(),
                new SleepAction(0.2),
                r.waitForBall(),
                new SleepAction(1),
                r.turnOffIntake(),
                r.reverseIntake(),
                new SleepAction(0.5),
                r.turnOffIntake()
        );
    }
}

