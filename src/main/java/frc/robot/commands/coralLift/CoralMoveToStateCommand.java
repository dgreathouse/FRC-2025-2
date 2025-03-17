package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;


public class CoralMoveToStateCommand extends Command {
  CoralLiftState m_state;
  /** Creates a new CoralMoveToStateCommand. */
  public CoralMoveToStateCommand(CoralLiftState _state) {
    m_state = _state;
    addRequirements(g.ROBOT.coralLift); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    g.ROBOT.coralLift.rotate(m_state);
    g.ROBOT.coralLift.moveLiftToPosition(m_state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
