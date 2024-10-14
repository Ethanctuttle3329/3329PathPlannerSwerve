package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  //Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  //Subsystem
  private final Swerve s_Swerve = new Swerve();

  //Trigger
  private final Trigger robotCentric = driver.leftBumper();

  public RobotContainer() {
    //drive command
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> robotCentric.getAsBoolean()));
  }
}