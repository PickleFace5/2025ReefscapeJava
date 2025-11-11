// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.Elastic;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  // Match time publisher for Elastic dashboard
  private final DoublePublisher matchTimePub;

  public Robot() {
    // Ignore joystick warning if we're not on a real field
    DriverStation.silenceJoystickConnectionWarning(!DriverStation.isFMSAttached());

    // Init everything lol
    m_robotContainer = new RobotContainer();

    // Logging
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Create USB camera stream
    CameraServer.startAutomaticCapture();

    // Open port for remote layout downloading from Elastic
    WebServer.start(5800, Filesystem.getDeployDirectory().toString());

    DataLogManager.log("Robot initialized");

    NetworkTable dashboardNT = NetworkTableInstance.getDefault().getTable("Elastic");
    matchTimePub = dashboardNT.getDoubleTopic("Match Time").publish();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update match time display
    matchTimePub.set(Timer.getMatchTime());
  }

  @Override
  public void disabledInit() {
    // Incrase throttle to decrease overheating isues
    m_robotContainer.setVisionThrottle(150);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    // Have the cameras lock tf in
    m_robotContainer.setVisionThrottle(0);
  }

  @Override
  public void autonomousInit() {
    DataLogManager.log("Autonomous period started");

    // Schedule selected auto
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      DataLogManager.log("Selected Auto: " + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }

    // Automatically switch tab in Elastic
    Elastic.selectTab("Autonomous");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    // Switch Elastic tabs when leaving auto
    DataLogManager.log("Autonomous period ended");
    Elastic.selectTab("Teleop");
  }

  @Override
  public void teleopInit() {
    // End the auto command as a safety precaution
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    DataLogManager.log("Teleoperated period started");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // Congratulate the drive team :)
    DataLogManager.log("Teleoperated period ended");
    if (DriverStation.isFMSAttached()) {
      Elastic.sendNotification(
          new Elastic.Notification(
              Elastic.NotificationLevel.INFO,
              "Good match!",
              DriverStation.getReplayNumber() > 1 ? "(again)" : ""));
    }
  }

  @Override
  public void testInit() {
    // Start SignalLogging + switch tabs for debugging
    DataLogManager.log("Test period started");
    CommandScheduler.getInstance().cancelAll();
    Elastic.selectTab("Debug");
    SignalLogger.start();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
