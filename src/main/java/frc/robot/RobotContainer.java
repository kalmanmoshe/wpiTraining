// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ElevatorCommands.ElevatorMagneticHoming;
import frc.robot.commands.ElevatorCommands.ElevatorSetPosition;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CommandPS5Controller drivercontroller = new CommandPS5Controller(0);
  private ElevatorSubsystem elevator;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevator = new ElevatorSubsystem(Constants.ElevatorConstants.ElevatorConfiguration);
    configureBindings();
  }
  private void configureBindings() {
    // Configure the trigger bindings
    drivercontroller.square().onTrue(new ElevatorMagneticHoming(elevator));

    drivercontroller.L1().onTrue(new ElevatorSetPosition(elevator, Constants.ElevatorConstants.HEIGHT_LEVEL_1));
    drivercontroller.L2().onTrue(new ElevatorSetPosition(elevator, Constants.ElevatorConstants.HEIGHT_LEVEL_2));
    drivercontroller.R1().onTrue(new ElevatorSetPosition(elevator, Constants.ElevatorConstants.HEIGHT_LEVEL_3));
    drivercontroller.R2().onTrue(new ElevatorSetPosition(elevator, Constants.ElevatorConstants.HEIGHT_LEVEL_4));
  }
}
