/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * Demonstrates an empty iterative OpMode
 */
@Config
@TeleOp(name = "Driver Control", group = "Concept")
//@Disabled
public class DriverControl extends OpMode {
  //test
  public static double firstAngle = 25;
  public static double secondAngle = 145;
  public static double thirdAngle = 267;
  public static double firstShootingAngle = 205;
  public static double secondShootingAngle = 332;
  public static double thirdShootingAngle = 90;
  public static double indexingAngle = 107;

  public static double SHOOTER_DELAY = 500;
  public static double RELOAD_DELAY = 1000;

  public static double INTAKE_DELAY = 310;
  public static double INTAKE_OFF_DELAY = 600;

  public static double WIGGLE_DELAY = 100;

  private ElapsedTime runtime = new ElapsedTime();

  private ElapsedTime wiggletime = new ElapsedTime();

  Pose2d BlueWallRight = new Pose2d(-48,32,0);
  //Pose2d RightWallleft = new Pose2d(-48, -32, Math.toRadians(180));

  MecanumDrive drive = null;
  Intake intake = null;
  Outtake outtake = null;
  Rotator rotator = null;


  Gamepad g1 = new Gamepad();
  Gamepad g2 = new Gamepad();

  Gamepad previousG1 = new Gamepad();
  Gamepad previousG2 = new Gamepad();

  boolean intakeToggle = false;

  boolean shooterToggle = false;

  boolean elavatorToggle = false;

  boolean hoodToggle = false;

  public static double power = 0;

  public enum ShooterState {
    READY, FIRE1, FIRE2, FIRE3, RELOAD
  }

  public enum IntakeState {
    READY, INTAKE1, INTAKE2, FULL, FIRING
  }

  ShooterState shooterState = ShooterState.READY;
  IntakeState intakeState = IntakeState.READY;

  public ElapsedTime shooterClock = new ElapsedTime();
  public ElapsedTime intakeClock = new ElapsedTime();


  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    drive = new MecanumDrive(hardwareMap, BlueWallRight);
    intake = new Intake(hardwareMap, telemetry);
    outtake = new Outtake(hardwareMap, telemetry);
    rotator = new Rotator(hardwareMap, telemetry);
  }

  @Override
  public void init_loop() {
  }


  @Override
  public void start() {
    rotator.init();
    outtake.init();
    runtime.reset();
    shooterClock.reset();
    intakeClock.reset();
  }

  /**
   * This method will be called repeatedly during the period between when
   * the START button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {
    previousG2.copy(g2);
    previousG1.copy(g1);
    g1.copy(gamepad1);
    g2.copy(gamepad2);


    telemetry.addData("Status", "Run Time: " + runtime.toString());

    drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ),
            -gamepad1.right_stick_x
    ));

    //NEWCODE


    if (g1.a && ! previousG1.a){
      intakeToggle = ! intakeToggle;
    }

    if (g1.bWasPressed()){
     shooterToggle = ! shooterToggle;
    }

    if (g1.xWasPressed()){
      elavatorToggle = ! elavatorToggle;
    }

    if(g1.yWasPressed()){
      hoodToggle = ! hoodToggle;
    }

    if(g1.dpad_up){
      rotator.setPosition(firstAngle);
    }
    if(g1.dpad_down){
      rotator.setPosition(secondAngle);
    }
    if(g1.dpad_left){
      rotator.setPosition(thirdAngle);
    }


    //Intake
    //if (intakeToggle){
    //intake.intakeMotorOn();
    //}
    //else{
      //intake.intakeMotorOff();
    //}

    //Launcher flywheel
//    if (shooterToggle){
//      outtake.launcherMotor1On();
//      outtake.launcherMotor2On();
//    }
//    else{
//      outtake.launcherMotor1Off();
//      outtake.launcherMotor2Off();
//    }

//    if(elavatorToggle){
//      outtake.elavatorMotorON();
//    }
//    else{
//      outtake.elavatorMotorOff();
//    }

   // if(hoodToggle){
     // outtake.hoodServoShoot();
    //}
    //else{
      //outtake.hoodServoStart();
    //}


    //TODO figure out why doesn't work if pressed with intake and why intake doesn't work if preloaded with one ball
    switch(shooterState) {
      case READY:

        //outtake.elavatorMotorOff();

        if (gamepad1.rightBumperWasPressed()) {
          outtake.launcherMotor2On();
          outtake.launcherMotor1On();
          outtake.hoodServoShoot();
          outtake.elavatorMotorON();
          //rotator.setPosition(firstAngle);
          shooterState = ShooterState.FIRE1;
        }
        break;

      case FIRE1:

        if (outtake.launchMotorsAtVelocity()) {
          rotator.setPosition(firstShootingAngle);
          outtake.elavatorMotorON();
          shooterClock.reset();
          shooterState = ShooterState.FIRE2;
        }
        break;

      case FIRE2:
        //if (!outtake.launchMotorsAtVelocity()) {
          //outtake.elavatorMotorOff();
        //}

        if (outtake.launchMotorsAtVelocity() && shooterClock.milliseconds() > SHOOTER_DELAY) {
          rotator.setPosition(secondShootingAngle);
          //outtake.elavatorMotorON();
          shooterClock.reset();
          shooterState = ShooterState.FIRE3;
        }
        break;

      case FIRE3:

        //if (!outtake.launchMotorsAtVelocity()) {
          //outtake.elavatorMotorOff();
        //}

        if (outtake.launchMotorsAtVelocity() && shooterClock.milliseconds() > SHOOTER_DELAY) {
          rotator.setPosition(thirdShootingAngle);
          //outtake.elavatorMotorON();
          shooterClock.reset();
          shooterState = ShooterState.RELOAD;
        }
          break;

          case RELOAD:
            if (outtake.launchMotorsAtVelocity() && shooterClock.milliseconds() > RELOAD_DELAY) {
              outtake.elavatorMotorOff();
              outtake.launcherMotor1Off();
              outtake.launcherMotor2Off();
              outtake.hoodServoStart();
              rotator.setPosition(firstAngle);
              intakeState = IntakeState.READY;
              shooterState = ShooterState.READY;
            }
            break;
        }


    switch (intakeState){
      case READY:
        if(g1.leftBumperWasPressed()){
          rotator.setPosition(firstAngle);
          intake.intakeMotorOn();
        }

        if(rotator.detectedBall()){
          intakeState = IntakeState.INTAKE1;
          intakeClock.reset();
        }
        break;
      case INTAKE1:
        rotator.setPosition(secondAngle);
        if(intakeClock.milliseconds() > INTAKE_DELAY && rotator.detectedBall()){
          intakeClock.reset();
          intakeState = IntakeState.INTAKE2;
        }

        break;

      case INTAKE2:
        rotator.setPosition(thirdAngle);
        if(intakeClock.milliseconds() > INTAKE_DELAY && rotator.detectedBall()){
          intakeClock.reset();
          intakeState = IntakeState.FULL;
        }
        break;

      case FULL:
        if(intakeClock.milliseconds() > INTAKE_OFF_DELAY) {
          intake.intakeMotorOff();
          shooterState = ShooterState.READY;
          intakeState = IntakeState.FIRING;
        }

        break;

      case FIRING:
        if (shooterState == ShooterState.RELOAD){
          intakeState = IntakeState.READY;
        }


        break;


    }


    rotator.readColorSensors();


    drive.updatePoseEstimate();
    rotator.update();

    Pose2d pose = drive.localizer.getPose();
    telemetry.addData("x", pose.position.x);
    telemetry.addData("y", pose.position.y);
    telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
    telemetry.addData("shooter state", shooterState);
    telemetry.addData("intake state", intakeState);
    telemetry.addData("motors at velocity", outtake.launchMotorsAtVelocity());
    telemetry.addData("launcher1 motors velocity", outtake.getVelocity1());
    telemetry.addData("launcher2 motors velocity", outtake.getVelocity2());
    telemetry.addData("ball detected", rotator.detectedBall());
    telemetry.update();

    TelemetryPacket packet = new TelemetryPacket();
    packet.fieldOverlay().setStroke("#3F51B5");
    Drawing.drawRobot(packet.fieldOverlay(), pose);
    FtcDashboard.getInstance().sendTelemetryPacket(packet);

  }

  /**
   * This method will be called once, when this OpMode is stopped.
   * <p>
   * Your ability to control hardware from this method will be limited.
   */
  @Override
  public void stop() {

  }
}
