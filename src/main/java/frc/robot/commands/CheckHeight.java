// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
public class CheckHeight extends Command {
  private final Lift m_Lift;
  int tolerance = 10;  // Ends command if the lift is +/- tolerance
  /** Creates a new CheckHeight. */
  public CheckHeight(Lift lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Lift = lift;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Lift.currentHeight() < m_Lift.getCommandedHeight() + tolerance && m_Lift.currentHeight() > m_Lift.getCommandedHeight() - tolerance) {
      return true;
    }
    return false;
  }
}
