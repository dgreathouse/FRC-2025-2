package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralLift.CoralLiftSetStateAndSpin;
import frc.robot.commands.coralLift.CoralMoveToStateCommand;
import frc.robot.commands.coralLift.CoralSetStateCommand;
import frc.robot.commands.coralLift.CoralSpinInCommand;
import frc.robot.commands.coralLift.CoralSpinOutCommand;
import frc.robot.commands.coralLift.SetCoralAIState;
import frc.robot.commands.drive.AutoDriveDelay;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoRotateToPose;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.AutoIDUtility;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.g;

public class AutoRL_Station extends SequentialCommandGroup {
  int tagID;
  AprilTagAlignState aprilTagAlignState;
  CoralLiftState coralLiftState;

  public AutoRL_Station(int _tagID, AprilTagAlignState _aprilTagAlignState, CoralLiftState _coralLiftState) {
    tagID = _tagID;
    aprilTagAlignState = _aprilTagAlignState;
    coralLiftState = _coralLiftState;

    addCommands(
        new SetCoralAIState(g.ROBOT.vision.getRobotAlignState(_tagID), _coralLiftState, _aprilTagAlignState),
        new AutoDriveDelay(),
        new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(tagID, aprilTagAlignState), .3, 1),
        new ParallelCommandGroup(
            new CoralMoveToStateCommand(_coralLiftState, 2),
            new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(tagID, aprilTagAlignState), 0.5, 2)),
        new CoralSpinOutCommand(coralLiftState, 1),
        new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getStationTagID(_tagID), AprilTagAlignState.LEFT), .3,1),
        new ParallelCommandGroup(
            new CoralMoveToStateCommand(CoralLiftState.START, 2),
            new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getStationTagID(_tagID), AprilTagAlignState.LEFT),0.5, 3)
        ),
        new CoralSpinInCommand(coralLiftState, 3),
        new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getNextReefTagID(_tagID), AprilTagAlignState.RIGHT),.3, 1),
        new ParallelCommandGroup(
             new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getNextReefTagID(_tagID), AprilTagAlignState.RIGHT), .5, 3),
             new CoralMoveToStateCommand(CoralLiftState.L3, 1)
        ),
        new CoralSpinOutCommand(coralLiftState, 1)
        // new CoralLiftSetStateAndSpin(CoralLiftState.ALGAE_LOW, 1),
        // new CoralMoveToStateCommand(CoralLiftState.START, 2)


    );
  }
}
