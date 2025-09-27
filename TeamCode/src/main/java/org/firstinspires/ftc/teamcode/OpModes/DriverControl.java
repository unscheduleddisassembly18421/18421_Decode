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

@TeleOp(name = "Driver Control", group = "Concept")
//@Disabled
public class DriverControl extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  Pose2d BlueWallRight = new Pose2d(-48,32,0);
  Pose2d RightWallleft = new Pose2d(-48, -32, Math.toRadians(180));

  MecanumDrive drive = null;
  Intake intake = null;
  outake outake = null;
  middle middle = null;


  Gamepad g1 = new Gamepad();
  Gamepad g2 = new Gamepad();

  Gamepad previousG1 = new Gamepad();
  Gamepad previousG2 = new Gamepad();

  boolean intakeToggle = false;

  boolean shooterToggle = false;

  /**
   * This method will be called once, when the INIT button is pressed.
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    drive = new MecanumDrive(hardwareMap, BlueWallRight);
    intake = new Intake(hardwareMap, telemetry);
    outake = new outake(hardwareMap, telemetry);
  }

  /**
   * This method will be called repeatedly during the period between when
   * the INIT button is pressed and when the START button is pressed (or the
   * OpMode is stopped).
   */
  @Override
  public void init_loop() {
  }

  /**
   * This method will be called once, when the START button is pressed.
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /**
   * This method will be called repeatedly during the period between when
   * the START button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {
    previousG2.copy(g1);
    previousG1.copy(g2);
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


    //Intake
    if (intakeToggle){
      intake.intakeMotorOn();
    }
    else{
      intake.intakeMotorOff();
    }

    //Launcher flywheel
    if (shooterToggle){
      outake.launcherMotorOn();
    }
    else{
      outake.launcherMotorOff();
    }


    drive.updatePoseEstimate();

    Pose2d pose = drive.localizer.getPose();
    telemetry.addData("x", pose.position.x);
    telemetry.addData("y", pose.position.y);
    telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
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
