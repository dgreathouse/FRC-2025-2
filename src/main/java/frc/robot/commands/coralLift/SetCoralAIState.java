package frc.robot.commands.coralLift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.AI;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;

public class SetCoralAIState extends InstantCommand {
  RobotAlignStates m_robotAlignState;
  CoralLiftState m_coralLiftState;
  AprilTagAlignState m_aprilTagAlignState;

  public SetCoralAIState(RobotAlignStates _robotAlignState, CoralLiftState _coralLiftState, AprilTagAlignState _aprilTagAlignState) {
    addRequirements(g.ROBOT.coralLift);
    m_robotAlignState = _robotAlignState;
    m_coralLiftState = _coralLiftState;
    m_aprilTagAlignState = _aprilTagAlignState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AI.ReefModel.setReefState(m_robotAlignState, m_coralLiftState, m_aprilTagAlignState);
  }
}
