// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.SimDrivetrain;

public class RobotContainer {
  
  CommandXboxController controller = new CommandXboxController(0);

  SimDrivetrain drivetrainSubsystem = new SimDrivetrain();

  // A put together command, you can do whatever you want to controll it. Just slow it down.
  Command driveCommand = Commands.run(() -> {
    drivetrainSubsystem.drive(-controller.getLeftY() / 3.0, -controller.getRightX() / 5.0);
  }, drivetrainSubsystem);
  
  public RobotContainer() {
    this.configureBindings();
  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(driveCommand);
  }

  public Command getAutonomousCommand() {
    // Delete this when removing Pathplanner
    /*return AutoBuilder.buildAuto("test");*/

    return new InstantCommand();
  }
}
