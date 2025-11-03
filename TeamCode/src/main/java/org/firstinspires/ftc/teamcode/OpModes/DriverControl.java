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

import static org.firstinspires.ftc.teamcode.Variables.INTAKEREVERSE_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.INTAKE_OFF_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.RELOAD_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.SHOOTER_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.firstAngle;
import static org.firstinspires.ftc.teamcode.Variables.firstShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdShootingAngle;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.HwRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Variables;

/*
 * Demonstrates an empty iterative OpMode
 */
@Config
@TeleOp(name = "Driver Control", group = "Concept")
//@Disabled
public class DriverControl extends OpMode {
  //test
  public HwRobot r = null;
  private ElapsedTime runtime = new ElapsedTime();




  Gamepad g1 = new Gamepad();
  Gamepad g2 = new Gamepad();

  Gamepad previousG1 = new Gamepad();
  Gamepad previousG2 = new Gamepad();

  boolean intakeToggle = false;

  boolean shooterToggle = false;

  boolean elevatorToggle = false;

  boolean hoodToggle = false;

  public static double power = 0;

  public enum ShooterState {
    READY, FIRE1, FIRE2, FIRE3, RELOAD
  }

  public enum GreenPosition{
    RIGHT, MIDDLE, LEFT
  }

  ShooterState shooterState = ShooterState.READY;
  Variables.IntakeState intakeState = Variables.IntakeState.READY;
  GreenPosition greenPosition;

  public ElapsedTime shooterClock = new ElapsedTime();
  public ElapsedTime intakeClock = new ElapsedTime();


  @Override
  public void init() {
    r = new HwRobot(telemetry, hardwareMap);
    r.init();
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    telemetry.addData("Status", "Initialized");

  }

  @Override
  public void init_loop() {
    if(gamepad1.dpad_left){
      greenPosition = GreenPosition.LEFT;
    }
    if(gamepad1.dpad_right){
      greenPosition = GreenPosition.RIGHT;
    }
    if(gamepad1.dpad_down){
      greenPosition = GreenPosition.MIDDLE;
    }
    telemetry.addData("green position", greenPosition);
  }


  @Override
  public void start() {

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

    r.drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ),
            -gamepad1.right_stick_x
    ));

    //NEWCODE


    //if (g1.a && ! previousG1.a){
      //intakeToggle = ! intakeToggle;
    //}

    //if (g1.bWasPressed()){
     //shooterToggle = ! shooterToggle;
    //}

    //if (g1.xWasPressed()){
      //elevatorToggle = !   elevatorToggle;
    //}

    //if(g1.yWasPressed()){
      //hoodToggle = ! hoodToggle;
    //}

    //if(g1.dpad_up){
      //r.rotator.setPosition(firstAngle);
    //}
    //if(g1.dpad_down){
      //r.rotator.setPosition(secondAngle);
    //}
    //if(g1.dpad_left){
      //r.rotator.setPosition(thirdAngle);
    //}


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


    switch(shooterState) {
      case READY:
        //outtake.r.elavatorMotorOff();
        if (g1.right_bumper && !previousG1.right_bumper) {
          r.outtake.launcherMotor2OnFar();
          r.outtake.launcherMotor1OnFar();
          r.outtake.hoodServoShoot();
          r.outtake.elavatorMotorON();
          //rotator.setPosition(firstAngle);
          shooterState = ShooterState.FIRE1;
        }
        break;

      case FIRE1:

        if (r.outtake.launchMotorsAtVelocity()) {
          r.rotator.setPosition(firstShootingAngle);
          r.outtake.elavatorMotorON();
          shooterClock.reset();
          shooterState = ShooterState.FIRE2;
        }
        break;

      case FIRE2:
        //if (!outtake.launchMotorsAtVelocity()) {
          //outtake.r.elavatorMotorOff();
        //}

        if (r.outtake.launchMotorsAtVelocity() && shooterClock.milliseconds() > SHOOTER_DELAY) {
          r.rotator.setPosition(secondShootingAngle);
          //outtake.r.elavatorMotorON();
          shooterClock.reset();
          shooterState = ShooterState.FIRE3;
        }
        break;

      case FIRE3:

        //if (!outtake.launchMotorsAtVelocity()) {
          //outtake.r.elavatorMotorOff();
        //}

        if (r.outtake.launchMotorsAtVelocity() && shooterClock.milliseconds() > SHOOTER_DELAY) {
          r.rotator.setPosition(thirdShootingAngle);
          //outtake.r.elavatorMotorON();
          shooterClock.reset();
          shooterState = ShooterState.RELOAD;
        }
          break;

          case RELOAD:
            if (r.outtake.launchMotorsAtVelocity() && shooterClock.milliseconds() > RELOAD_DELAY) {
              r.outtake.elavatorMotorOff();
              r.outtake.launcherMotor1Off();
              r.outtake.launcherMotor2Off();
              r.outtake.hoodServoStart();
              r.rotator.setPosition(firstAngle);
              intakeState = Variables.IntakeState.READY;
              shooterState = ShooterState.READY;
            }
            break;
        }


    switch (intakeState){
      case READY:
        r.rotator.leftLightRed();
        r.rotator.rightLightRed();
        if(g1.left_bumper && !previousG1.left_bumper){
          r.rotator.setPosition(firstAngle);
          r.intake.intakeMotorOn();
          intakeState = Variables.IntakeState.INTAKE1;
        }
        break;

      case INTAKE1:
        if(r.rotator.detectedBall() || g1.xWasPressed()){
          intakeState = Variables.IntakeState.INTAKE2;
          intakeClock.reset();
        }

        if(g1.bWasPressed()){
          r.intake.intakeMotorOut();
        }
        break;
      case INTAKE2:
        r.rotator.setPosition(secondAngle);
        if(intakeClock.milliseconds() > INTAKE_DELAY && (r.rotator.detectedBall() || g1.xWasPressed())){
          intakeClock.reset();
          intakeState = Variables.IntakeState.INTAKE3;
        }
        if(g1.bWasPressed()){
          r.intake.intakeMotorOut();
        }
        break;

      case INTAKE3:
        r.rotator.setPosition(thirdAngle);
        if(intakeClock.milliseconds() > INTAKE_DELAY && (r.rotator.detectedBall()) || g1.xWasPressed()){
          intakeClock.reset();
          intakeState = Variables.IntakeState.FULL;
        }
        if(g1.bWasPressed()){
          r.intake.intakeMotorOut();
        }
        break;

      case FULL:
        if(intakeClock.milliseconds() > INTAKE_OFF_DELAY) {
          r.intake.intakeMotorOff();
          r.rotator.leftLightGreen();
          r.rotator.rightLightGreen();
          shooterState = ShooterState.READY;
          intakeState = Variables.IntakeState.FIRING;
        }

        break;

      case FIRING:
        if (shooterState == ShooterState.RELOAD){
          intakeState = Variables.IntakeState.READY;
        }
        break;
    }


    r.rotator.readColorSensors();


    r.drive.updatePoseEstimate();
    r.rotator.update();

    Pose2d pose = r.drive.localizer.getPose();
    telemetry.addData("x", pose.position.x);
    telemetry.addData("y", pose.position.y);
    telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
    telemetry.addData("shooter state", shooterState);
    telemetry.addData("intake state", intakeState);
    telemetry.addData("motors at velocity", r.outtake.launchMotorsAtVelocity());
    telemetry.addData("launcher1 motors velocity", r.outtake.getVelocity1());
    telemetry.addData("launcher2 motors velocity", r.outtake.getVelocity2());
    telemetry.addData("ball detected", r.rotator.detectedBall());
    telemetry.addData("green Position", greenPosition);
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
