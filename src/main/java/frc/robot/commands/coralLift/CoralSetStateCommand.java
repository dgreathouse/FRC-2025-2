package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;

public class CoralSetStateCommand extends InstantCommand {
  CoralLiftState m_state;
  
  public CoralSetStateCommand(CoralLiftState _state) {
    m_state = _state;
    addRequirements(g.ROBOT.coralLift);
  }

  @Override
  public void initialize() {
    g.CORALLIFT.state = m_state;
  }
}
