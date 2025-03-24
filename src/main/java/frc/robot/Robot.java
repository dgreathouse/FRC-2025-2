// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commandGroups.AutoRightOrLeft;
import frc.robot.commandGroups.AutoCenter;
import frc.robot.commandGroups.AutoDoNothing;
import frc.robot.commandGroups.AutoRL_Station;
import frc.robot.commands.coralLift.CoralLiftDefaultCommand;
import frc.robot.commands.coralLift.CoralReverseCommand;
import frc.robot.commands.drive.AutoDriveDefaultCommand;
import frc.robot.commands.drive.DrivetrainDefaultCommand;
import frc.robot.lib.AI;
import frc.robot.lib.AprilTagAlignState;

import frc.robot.lib.CoralLiftState;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private Notifier m_telemetry = new Notifier(this::updateDashboard);
  private static List<IUpdateDashboard> m_dashboardUpdaters = new ArrayList<>();
  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand();
  private AutoDriveDefaultCommand m_autoDriveDefaultCommand = new AutoDriveDefaultCommand();
  private CoralLiftDefaultCommand m_coralLiftDefaultCommand = new CoralLiftDefaultCommand();

  public Robot() {
    g.ROBOT.drive.setDefaultCommand(m_autoDriveDefaultCommand); 
    g.ROBOT.coralLift.setDefaultCommand(m_coralLiftDefaultCommand);

    configureBindings();

    m_autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());

    m_autoChooser.addOption("Blue Right RL2", new AutoRightOrLeft(22,AprilTagAlignState.RIGHT, CoralLiftState.L2));
    m_autoChooser.addOption("Blue Right LL2", new AutoRightOrLeft(22,AprilTagAlignState.LEFT, CoralLiftState.L2));
    m_autoChooser.addOption("Blue Left RL2",  new AutoRightOrLeft(20,AprilTagAlignState.RIGHT, CoralLiftState.L2));
    m_autoChooser.addOption("Blue Left LL2",  new AutoRightOrLeft(20,AprilTagAlignState.LEFT, CoralLiftState.L2));
    m_autoChooser.addOption("Red Left RL2",  new AutoRightOrLeft(11,AprilTagAlignState.RIGHT, CoralLiftState.L2));
    m_autoChooser.addOption("Red Left LL2",  new AutoRightOrLeft(11,AprilTagAlignState.LEFT, CoralLiftState.L2));
    m_autoChooser.addOption("Red Right RL2",  new AutoRightOrLeft(9,AprilTagAlignState.RIGHT, CoralLiftState.L2));
    m_autoChooser.addOption("Red Right LL2",  new AutoRightOrLeft(9,AprilTagAlignState.LEFT, CoralLiftState.L2));
    m_autoChooser.addOption("Red Center RL3", new AutoCenter(10,AprilTagAlignState.RIGHT, CoralLiftState.L3));
    m_autoChooser.addOption("Red Center LL3", new AutoCenter(10,AprilTagAlignState.LEFT, CoralLiftState.L3));
    m_autoChooser.addOption("Blue Center RL3", new AutoCenter(21,AprilTagAlignState.RIGHT, CoralLiftState.L3));
    m_autoChooser.addOption("Blue Center LL3", new AutoCenter(21,AprilTagAlignState.LEFT, CoralLiftState.L3));
    m_autoChooser.addOption("Blue Left L23", new AutoRL_Station(20,AprilTagAlignState.LEFT, CoralLiftState.L2));
    m_autoChooser.addOption("Blue Right L23", new AutoRL_Station(22,AprilTagAlignState.LEFT, CoralLiftState.L2));
    SmartDashboard.putData("Autonomouse Play", m_autoChooser);
    SmartDashboard.putNumber("Auto/AutoDelay_sec" ,0);

    // Start telemetry in a slower rate than the main loop
    m_telemetry.startPeriodic(g.ROBOT.TELEMETRY_RATE_sec);
    AI.initData();
    
  }
  private void configureBindings() {
    // Driver controls
    g.OI.DRIVER_RESET_YAW.onTrue( new InstantCommand(() -> g.ROBOT.drive.resetYaw(0.0), g.ROBOT.drive));
    g.OI.DRIVER_MODE_ANGLEFIELDCENTRIC.onTrue( new InstantCommand(() -> { g.DRIVETRAIN.driveMode = DriveMode.ANGLE_FIELD_CENTRIC;}, g.ROBOT.drive));
    g.OI.DRIVER_MODE_FIELDCENTRIC.onTrue( new InstantCommand(() -> { g.DRIVETRAIN.driveMode = DriveMode.FIELD_CENTRIC; }, g.ROBOT.drive));
    g.OI.DRIVER_MODE_ROBOTCENTRIC.onTrue( new InstantCommand(() -> { g.DRIVETRAIN.driveMode = DriveMode.ROBOT_CENTRIC;}, g.ROBOT.drive));
    g.OI.DRIVER_TOGGLE_DRIVETRAIN_ENABLE.onTrue( new InstantCommand( () -> { g.SWERVE.isEnabled = !g.SWERVE.isEnabled; }, g.ROBOT.drive));
    g.OI.DRIVER_STATION_RIGHT.onTrue( new InstantCommand(() -> {g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_RIGHT); }, g.ROBOT.drive));
    g.OI.DRIVER_STATION_LEFT.onTrue( new InstantCommand(() -> {g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_LEFT);}, g.ROBOT.drive));
    //g.OI.DRIVER_TOGGLE_AUTO_DRIVE.onTrue(new InstantCommand(() -> {g.DRIVETRAIN.isAutoDriveEnabled = !g.DRIVETRAIN.isAutoDriveEnabled;}));
    g.OI.DRIVER_MODE_SPEED_TOGGLE.onTrue(new InstantCommand(() -> {g.ROBOT.drive.toggleSpeed();}));
    g.OI.DRIVER_CORAL_REVERSE.onTrue(new CoralReverseCommand());


    // Operator controls
    //g.OI.OPERATOR_STATION_LEFT_DEFENSE.onTrue(new InstantCommand(()-> {g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_LEFT_DEFENSE); }, g.ROBOT.drive));
   // g.OI.OPERATOR_STATION_RIGHT_DEFENSE.onTrue(new InstantCommand(()-> {g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_RIGHT_DEFENSE); }, g.ROBOT.drive));
    //g.OI.OPERATOR_CORAL_L1.onTrue(new InstantCommand(()-> {g.CORALLIFT.state = CoralLiftState.L1;}, g.ROBOT.coralLift));
    g.OI.OPERATOR_CORAL_L2.onTrue(new InstantCommand(()-> { g.CORALLIFT.state = CoralLiftState.L2; }, g.ROBOT.coralLift));
    g.OI.OPERATOR_CORAL_L3.onTrue(new InstantCommand(()-> { g.CORALLIFT.state = CoralLiftState.L3; }, g.ROBOT.coralLift));
    g.OI.OPERATOR_ALGAE_HIGH.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.ALGAE_HIGH; }, g.ROBOT.coralLift ));
    g.OI.OPERATOR_ALGAE_LOW.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.ALGAE_LOW; }, g.ROBOT.coralLift ));
    g.OI.OPERATOR_CORAL_START.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.START; }, g.ROBOT.coralLift ));
    g.OI.OPERATOR_LIFT_CLIMB_UP.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.LIFT_CLIMB_UP; }, g.ROBOT.coralLift ));
    g.OI.OPERATOR_LIFT_CLIMB_DOWN.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.LIFT_CLIMB_DOWN; }, g.ROBOT.coralLift ));
    g.OI.OPERATOR_AUTO_DRIVE_TOGGLE.onTrue(new InstantCommand(() -> { g.ROBOT.drive.toggleIsAutoDriveEnabled(); }, g.ROBOT.drive));
    g.OI.OPERATOR_CANCEL_AUTODRIVE.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.cancelAutoDriveToPose(); }, g.ROBOT.drive));
    g.OI.OPERATOR_RESET_LEFT_AI.onTrue(new InstantCommand(() -> { AI.ReefModel.resetReefState(AprilTagAlignState.LEFT); }, g.ROBOT.drive));
    g.OI.OPERATOR_RESET_RIGHT_AI.onTrue(new InstantCommand(() -> { AI.ReefModel.resetReefState(AprilTagAlignState.RIGHT); }, g.ROBOT.drive));

    //Button board
    g.OI.BB_ALGAE_HIGH.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.ALGAE_HIGH; }, g.ROBOT.coralLift ));
    g.OI.BB_ALGAE_LOW.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.ALGAE_LOW; }, g.ROBOT.coralLift ));
    g.OI.BB_ALGAE_START.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.START; }, g.ROBOT.coralLift ));
    g.OI.BB_CORAL_L3.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.L3; }, g.ROBOT.coralLift ));
    g.OI.BB_CORAL_L2.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.L2; }, g.ROBOT.coralLift ));
    g.OI.BB_CORAL_L1.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.L1; }, g.ROBOT.coralLift ));
    g.OI.BB_CORAL_START.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.START; }, g.ROBOT.coralLift ));

     g.OI.BB_ROBOT_BACK.onTrue(new InstantCommand(() ->{ AI.StateInput.setState(RobotAlignStates.BACK); }, g.ROBOT.drive, g.ROBOT.coralLift ));
     g.OI.BB_ROBOT_BACK_RIGHT.onTrue(new InstantCommand(() ->{ AI.StateInput.setState(RobotAlignStates.BACK_RIGHT); }, g.ROBOT.drive, g.ROBOT.coralLift  ));
     g.OI.BB_ROBOT_BACK_LEFT.onTrue(new InstantCommand(() ->{ AI.StateInput.setState(RobotAlignStates.BACK_LEFT); }, g.ROBOT.drive, g.ROBOT.coralLift  ));
     g.OI.BB_ROBOT_FRONT.onTrue(new InstantCommand(() ->{ AI.StateInput.setState(RobotAlignStates.FRONT); }, g.ROBOT.drive, g.ROBOT.coralLift  ));
     g.OI.BB_ROBOT_FRONT_RIGHT.onTrue(new InstantCommand(() ->{ AI.StateInput.setState(RobotAlignStates.FRONT_RIGHT); }, g.ROBOT.drive, g.ROBOT.coralLift  ));
     g.OI.BB_ROBOT_FRONT_LEFT.onTrue(new InstantCommand(() ->{ AI.StateInput.setState(RobotAlignStates.FRONT_LEFT); }, g.ROBOT.drive, g.ROBOT.coralLift  ));
     g.OI.BB_LIFT_CLIMB_UP.onTrue(new InstantCommand(() ->{ g.ROBOT.coralLift.moveWithVoltage(6,CoralLiftState.LIFT_CLIMB_UP); }, g.ROBOT.coralLift ));
     g.OI.BB_LIFT_CLIMB_UP.onFalse(new InstantCommand(() ->{ g.ROBOT.coralLift.moveWithVoltage(0,CoralLiftState.LIFT_CLIMB_UP);}, g.ROBOT.coralLift ));


     //g.OI.BB_LIFT_CLIMB_UP.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.LIFT_CLIMB_UP; }, g.ROBOT.coralLift ));
     g.OI.BB_LIFT_CLIMB_DOWN.onTrue(new InstantCommand(() ->{ g.CORALLIFT.state = CoralLiftState.LIFT_CLIMB_DOWN; }, g.ROBOT.coralLift ));
     
     g.OI.BB_ROBOT_STATION_LEFT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_LEFT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_STATION_RIGHT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_RIGHT); }, g.ROBOT.drive ));

     g.OI.BB_ROBOT_RIGHT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.RIGHT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_LEFT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.LEFT); }, g.ROBOT.drive ));

     g.OI.BB_APRIL_LEFT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setAprilTagAlignment(AprilTagAlignState.LEFT); }, g.ROBOT.drive));
     g.OI.BB_APRIL_RIGHT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setAprilTagAlignment(AprilTagAlignState.RIGHT); }, g.ROBOT.drive));
     g.OI.BB_APRIL_CENTER.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setAprilTagAlignment(AprilTagAlignState.CENTER); }, g.ROBOT.drive));

  }
  public static void addDashboardUpdater(IUpdateDashboard updater) {
    m_dashboardUpdaters.add(updater);
  }
  private void updateDashboard() {
    for (IUpdateDashboard updater : m_dashboardUpdaters) {
      updater.updateDashboard();
    }
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    g.ROBOT.drive.setDefaultCommand(m_autoDriveDefaultCommand);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    g.ROBOT.drive.setDefaultCommand(m_drivetrainDefaultCommand);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
