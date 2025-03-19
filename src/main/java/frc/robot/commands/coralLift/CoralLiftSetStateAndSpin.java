// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralLift;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;
import frc.robot.subsystems.CoralLift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralLiftSetStateAndSpin extends Command {
  /** Creates a new CoralLiftSetStateAndSpin. */
  CoralLiftState m_state;
  double m_timeout;
  Timer m_timer = new Timer();

  public CoralLiftSetStateAndSpin(CoralLiftState _state, double _timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_timeout = _timeout;
    m_state = _state;
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
    g.ROBOT.coralLift.rotate(m_state);
    g.ROBOT.coralLift.moveLiftToPosition(m_state);
    g.ROBOT.coralLift.spinOut(m_state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_timeout)) {
      g.ROBOT.coralLift.spin(0);
      return m_timer.hasElapsed(m_timeout);
    } else {
      return false;
    }
  }
}
