package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;

public class CoralLiftSetStateAndSpin extends Command {

  CoralLiftState m_state;
  double m_timeout;
  Timer m_timer = new Timer();

  public CoralLiftSetStateAndSpin(CoralLiftState _state, double _timeout) {

    m_timeout = _timeout;
    m_state = _state;
  }

  @Override
  public void initialize() {
    m_timer.restart();
    g.CORALLIFT.state = m_state;
  }

  @Override
  public void execute() {
    g.ROBOT.coralLift.rotate(m_state);
    g.ROBOT.coralLift.moveLiftToPosition(m_state);
    g.ROBOT.coralLift.spinOut(m_state);
  }

  @Override
  public void end(boolean interrupted) {
  }

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
