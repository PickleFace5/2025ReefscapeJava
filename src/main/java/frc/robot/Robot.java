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
  private final DoublePublisher matchTimePub;

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(!DriverStation.isFMSAttached());
    m_robotContainer = new RobotContainer();

    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    CameraServer.startAutomaticCapture();

    WebServer.start(5800, Filesystem.getDeployDirectory().toString());

    DataLogManager.log("Robot initialized");

    NetworkTable dashboardNT = NetworkTableInstance.getDefault().getTable("Elastic");
    matchTimePub = dashboardNT.getDoubleTopic("Match Time").publish();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    matchTimePub.set(Timer.getMatchTime());
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setVisionThrottle(150);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robotContainer.setVisionThrottle(0);
  }

  @Override
  public void autonomousInit() {
    DataLogManager.log("Autonomous period started");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      DataLogManager.log("Selected Auto: " + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }

    Elastic.selectTab("Autonomous");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    DataLogManager.log("Autonomous period ended");
    Elastic.selectTab("Teleop");
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    DataLogManager.log("Teleoperated period started");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
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
