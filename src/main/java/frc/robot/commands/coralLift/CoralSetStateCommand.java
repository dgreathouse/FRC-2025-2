// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralSetStateCommand extends InstantCommand {
  CoralLiftState m_state;
  public CoralSetStateCommand(CoralLiftState _state) {
    m_state = _state;
    addRequirements(g.ROBOT.coralLift);
  }

  @Override
  public void execute() {
    g.CORALLIFT.state = m_state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    g.CORALLIFT.state = m_state;
  }
}
