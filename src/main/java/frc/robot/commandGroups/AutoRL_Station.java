package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coralLift.CoralLiftSetStateAndSpin;
import frc.robot.commands.coralLift.CoralMoveToStateCommand;
import frc.robot.commands.coralLift.CoralSpinInCommand;
import frc.robot.commands.coralLift.CoralSpinOffCommand;
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
    //addRequirements(g.ROBOT.drive, g.ROBOT.coralLift);
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
        new CoralSpinOutCommand(coralLiftState, .5),
        new ParallelDeadlineGroup(
          new CoralMoveToStateCommand(CoralLiftState.START, 1),
           new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getStationTagID(_tagID), AprilTagAlignState.CENTER), .3,1)
        ),
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getStationTagID(_tagID), AprilTagAlignState.CENTER),0.6, 2.2),
        new ParallelDeadlineGroup(
          new AutoDriveDelay(2),
          new CoralSpinInCommand(CoralLiftState.START, 2.0)
            
        ),
        new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getNextReefTagID(_tagID), aprilTagAlignState),.3, 1),
        new CoralSpinOffCommand(),
        new ParallelDeadlineGroup (
             new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getNextReefTagID(_tagID), aprilTagAlignState), .6, 2.25),
             new CoralMoveToStateCommand(CoralLiftState.L3, 2.25)
        ),
        new CoralSpinOutCommand(CoralLiftState.L3, 0.5),
        new CoralLiftSetStateAndSpin(CoralLiftState.ALGAE_LOW, 2),
        new ParallelDeadlineGroup(
          new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(AutoIDUtility.getStationTagID(_tagID), AprilTagAlignState.CENTER),0.6, 2.2),
          new CoralMoveToStateCommand(CoralLiftState.START, 1.25)
        )



    );
  }
}
