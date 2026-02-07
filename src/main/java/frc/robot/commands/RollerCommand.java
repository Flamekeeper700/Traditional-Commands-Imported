// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANRollerSubsystem;
//"import java.util.function.DoubleSupplier;" old?

// Command to run the roller with joystick inputs
public class RollerCommand extends Command {
  private final double forward;
//TODO: Figure out what "private final double reverse" does
  @SuppressWarnings("unused")
  private final double reverse;

  // private final CANRollerSubsystem rollerSubsystem;
  private final CANRollerSubsystem rollerSubsystem;


  public RollerCommand(
    double forward, double reverse, CANRollerSubsystem rollerSubsystem) {
    this.forward = forward;
    this.reverse = reverse;
    this.rollerSubsystem = rollerSubsystem;

    addRequirements(this.rollerSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    // Run the roller motor at the desired speed
    rollerSubsystem.runRoller(forward);
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
