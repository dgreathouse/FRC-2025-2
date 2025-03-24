// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;

public class CoralSpinOffCommand extends Command {
  /** Creates a new CoralSpinCommand. */
  CoralLiftState m_state;
  double m_timeout = 0.0;


  public CoralSpinOffCommand() { 
    addRequirements(g.ROBOT.coralLift);


  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    g.ROBOT.coralLift.spin(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // g.ROBOT.coralLift.spin(.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
