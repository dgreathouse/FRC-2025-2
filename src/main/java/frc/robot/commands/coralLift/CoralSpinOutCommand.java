// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;

public class CoralSpinOutCommand extends Command {
  /** Creates a new CoralSpinCommand. */
  CoralLiftState m_state;
  double m_timeout = 0.0;
  Timer m_timer = new Timer();

  public CoralSpinOutCommand(CoralLiftState _state, double _timeout) { 
    addRequirements(g.ROBOT.coralLift);
    m_state = _state;
    m_timeout = _timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    g.CORALLIFT.state = m_state;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    g.ROBOT.coralLift.spinOut(m_state);
    g.ROBOT.coralLift.rotate(m_state);
    g.ROBOT.coralLift.moveLiftToPosition(m_state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.hasElapsed(m_timeout)){
      g.ROBOT.coralLift.spin(0);
      return m_timer.hasElapsed(m_timeout);
    } else{
      return false;
    }
  }
}
