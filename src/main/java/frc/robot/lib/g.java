// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CoralLift;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class g {
  public static class ROBOT {


    public static final Pigeon2 gyro_pigeon2 = new Pigeon2(g.CAN_IDS_CANIVORE.PIGEON2, g.CAN_IDS_CANIVORE.NAME);
    public static final AHRS gyro_navx = new AHRS(NavXComType.kMXP_SPI);
    public static final PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);
    public static volatile boolean isPrimaryGyroActive = true;
    public static volatile Rotation2d rotationPrimary = new Rotation2d();
    public static volatile Rotation2d rotationSecondary = new Rotation2d();
    public static volatile double angleActual_deg;
    public static volatile Rotation2d angleActual_Rot2d = new Rotation2d();
    public static volatile double angleRobotTarget_deg;
    public static volatile double angleDriveTarget_deg;
    public static volatile double speedDriveTarget_mPsec;
    public static volatile Pose2d pose2d = new Pose2d();

    public static volatile Pose3d pose3d = new Pose3d();
    public static volatile Field2d field2d = new Field2d();

    public static final Pose2d POSE_START_ZERO = new Pose2d(0, 0, new Rotation2d());
    public static final Pose2d POSE_START_LEFT = new Pose2d(7.1374, 6.9088, new Rotation2d());
    public static final Pose2d POSE_START_RIGHT = new Pose2d(7.1374, 1.2192, new Rotation2d());
    public static final Pose2d POSE_START_CENTER = new Pose2d(7.1374, 4.064, new Rotation2d());
    public static final double TELEMETRY_RATE_sec = 0.02;
    public static final long ODOMETRY_RATE_ms = 5;
    public static volatile double centerDistanceToFrontBumper_m = 0.565;// 0.535;
    public static volatile double centerDistanceToBackBumper_m = 0.42211;
    public static volatile double centerDistanceToStationSide_m = 0.609;
    public static final double MAX_BATTERY_SUPPLY_volts = 12.8;

    public static RobotAlignStates alignmentState = RobotAlignStates.UNKNOWN;

    public static CoralLift coralLift = new CoralLift();
    public static Drivetrain drive = new Drivetrain();
    public static VisionProcessor vision = new VisionProcessor();

 
  }

    public static class CAN_IDS_ROBORIO {
        public static final String NAME = "rio";
        public static final int CORAL_LEFT_MOTOR = 5;
        public static final int CORAL_RIGHT_MOTOR = 6;
        public static final int CORAL_ROTATE_MOTOR = 60;
        public static final int LIFT_MOTOR = 10;
    }

    public static class CAN_IDS_CANIVORE {
        // 10-25 taken for drivetrain in DriveTrain.java
        public static final String NAME = "CANivore";
        public static final int PIGEON2 = 5;

    }
      /** CV contains the data for Conversion Variables. 
   * These are used in cases that a quick conversion is needed instead of getting the conversion is some other way.  */
  public static class CV {
    public static final double DEGREES_TO_RADIANS = 0.017453292519943295;
    public static final double RADIANS_TO_DEGREES = 57.29577951308232;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double MPS_TO_FEETPERSEC = 3.28084;
  }
  /** The OI class contains data associated with Operator Interfaces, such as PS5 Controllers */
  public static class OI {
    // Driver controller
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static CommandPS5Controller driverController = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);

    public static final double THUMBSTICK_AXIS_ANGLE_DEADBAND = 0.7; 
    public static final Trigger DRIVER_MODE_FIELDCENTRIC = driverController.povLeft();
    public static final Trigger DRIVER_MODE_ROBOTCENTRIC = driverController.povRight();
    public static final Trigger DRIVER_MODE_ANGLEFIELDCENTRIC = driverController.povDown();
    public static final Trigger DRIVER_TOGGLE_AUTO_DRIVE = driverController.povUp();
    public static final Trigger DRIVER_CORAL_IN = driverController.R1();
    public static final Trigger DRIVER_CORAL_OUT = driverController.L1();
    public static final Trigger DRIVER_CORAL_REVERSE = driverController.triangle();
    public static final Trigger DRIVER_RESET_YAW = driverController.create();
    public static final Trigger DRIVER_TOGGLE_DRIVETRAIN_ENABLE = driverController.touchpad();
    public static final Trigger DRIVER_STATION_RIGHT = driverController.circle();
    public static final Trigger DRIVER_STATION_LEFT = driverController.square();
    // public static final Trigger DRIVER_ALGAE_OUT = driverController.triangle();
    // public static final Trigger DRIVER_ALGAE_IN = driverController.cross();
    public static final Trigger DRIVER_MODE_SPEED_TOGGLE = driverController.PS();
     
    // Operator controller
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static CommandPS5Controller operatorController = new CommandPS5Controller(OPERATOR_CONTROLLER_PORT);
   public static final Trigger OPERATOR_RESET_LEFT_AI = operatorController.square();
   public static final Trigger OPERATOR_RESET_RIGHT_AI = operatorController.circle();
    //public static final Trigger OPERATOR_CORAL_L1 = operatorController.povUp();
    public static final Trigger OPERATOR_CORAL_L2 = operatorController.cross();
    public static final Trigger OPERATOR_CORAL_L3 = operatorController.triangle();
    public static final Trigger OPERATOR_ALGAE_HIGH = operatorController.povUp();
    public static final Trigger OPERATOR_ALGAE_LOW = operatorController.povDown();
    public static final Trigger OPERATOR_CORAL_START = operatorController.touchpad();
    public static final Trigger OPERATOR_LIFT_CLIMB_UP = operatorController.R1();
    public static final Trigger OPERATOR_LIFT_CLIMB_DOWN = operatorController.L1();
    public static final Trigger OPERATOR_AUTO_DRIVE_TOGGLE = operatorController.povRight();
    public static final Trigger OPERATOR_CANCEL_AUTODRIVE = operatorController.L3();



    // Button Board
    public static final int BUTTON_BOARD_PORT = 2;
    public static CommandJoystick buttonBoard = new CommandJoystick(BUTTON_BOARD_PORT);

    public static final Trigger BB_ALGAE_BARGE = buttonBoard.button(18);
    public static final Trigger BB_ALGAE_HIGH = buttonBoard.button(17);
    public static final Trigger BB_ALGAE_LOW = buttonBoard.button(16);
    public static final Trigger BB_ALGAE_PROCESSOR = buttonBoard.button(15);
    public static final Trigger BB_ALGAE_FLOOR = buttonBoard.button(13);
    public static final Trigger BB_ALGAE_FLORAL = buttonBoard.button(14);
    public static final Trigger BB_ALGAE_START = buttonBoard.button(25);

    public static final Trigger BB_CORAL_L4 = buttonBoard.button(12);
    public static final Trigger BB_CORAL_L3 = buttonBoard.button(11);
    public static final Trigger BB_CORAL_L2 = buttonBoard.button(10);
    public static final Trigger BB_CORAL_L1 = buttonBoard.button(9);
    public static final Trigger BB_LIFT_CLIMB_UP = buttonBoard.button(8);
    public static final Trigger BB_LIFT_CLIMB_DOWN = buttonBoard.button(7);
    public static final Trigger BB_CORAL_START = buttonBoard.button(30);

    //public static final Trigger BB_LIFT_RAISE = buttonBoard.button(7);
    //public static final Trigger BB_LIFT_LOWER = buttonBoard.button(8);

    public static final Trigger BB_ROBOT_BACK = buttonBoard.button(3);
    public static final Trigger BB_ROBOT_FRONT = buttonBoard.button(2);
    public static final Trigger BB_ROBOT_RIGHT = buttonBoard.button(20);
    public static final Trigger BB_ROBOT_LEFT = buttonBoard.button(23);
    public static final Trigger BB_ROBOT_BACK_RIGHT = buttonBoard.button(21);
    public static final Trigger BB_ROBOT_BACK_LEFT = buttonBoard.button(24);
    public static final Trigger BB_ROBOT_FRONT_RIGHT = buttonBoard.button(19);
    public static final Trigger BB_ROBOT_FRONT_LEFT = buttonBoard.button(22);
    public static final Trigger BB_ROBOT_STATION_RIGHT = buttonBoard.button(26);
    public static final Trigger BB_ROBOT_STATION_LEFT = buttonBoard.button(29);

    public static final Trigger BB_APRIL_RIGHT = buttonBoard.button(27);
    public static final Trigger BB_APRIL_LEFT = buttonBoard.button(28);
    public static final Trigger BB_APRIL_CENTER = buttonBoard.button(1);

    // Smartdashboard buttons that are on the screen
    public static boolean isGyroPrimaryActive = true;
  }
  /** Swerve stores data for the Swerve drives. This class contains other classes like DRIVE and STEER */
  public static class SWERVE {
    /* CONTANTS */
    public static final int COUNT = 3;
    /* data */
    public static volatile double totalSwerveCurrent_amps = 0;
    public static volatile boolean isEnabled = true;
    public static final SwerveModule[] modules = new SwerveModule[COUNT];
    public static final SwerveModulePosition[] positions = new SwerveModulePosition[COUNT];
    public static final double CAN_UPDATE_FREQ_hz = 200;
    /** The data for the SWERVE Drive motors */
    public static class DRIVE {

      private static final double MOTOR_PINION_TEETH = 10.0;
      private static final double GEAR_1_TEETH = 34.0;
      private static final double GEAR_2_DRIVE_TEETH = 28.0;
      private static final double GEAR_2_DRIVEN_TEETH = 18.0;
      private static final double GEAR_BEVEL_DRIVE_TEETH = 15.0;
      private static final double GEAR_BEVEL_DRIVEN_TEETH = 45.0;
      public static final double GEAR_RATIO = (GEAR_1_TEETH / MOTOR_PINION_TEETH)
          * (GEAR_2_DRIVEN_TEETH / GEAR_2_DRIVE_TEETH)
          * (GEAR_BEVEL_DRIVEN_TEETH / GEAR_BEVEL_DRIVE_TEETH);
      public static final double WHEEL_DIAMETER_m = 0.1016; // .10287
      private static final double WHEEL_CIRCUMFERENCE_m = Math.PI * WHEEL_DIAMETER_m;
      public static final double MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm = GEAR_RATIO / WHEEL_CIRCUMFERENCE_m;
      private static final double MOTOR_MAX_VELOCITY_rotPmin = 5800.0;
      private static final double MOTOR_MAX_VELOCITY_rotPsec = MOTOR_MAX_VELOCITY_rotPmin / 60.0;
      private static final double WHEEL_MAX_VELOCITY_rotPsec = MOTOR_MAX_VELOCITY_rotPsec / GEAR_RATIO;
      private static final double MOTOR_PEAK_EFFICIENCY_percent = 80;
      public static final double MAX_VELOCITY_mPsec = WHEEL_CIRCUMFERENCE_m
          * WHEEL_MAX_VELOCITY_rotPsec
          * MOTOR_PEAK_EFFICIENCY_percent
          / 100.0;
      public static final double MAX_ANGULAR_VELOCITY_radPsec = MAX_VELOCITY_mPsec * (1 / CHASSIS.WHEELBASE_mPrad);

      public static double PID_KP = 1.0;
      public static double PID_KI = 0.20;
      public static double PID_KV = g.ROBOT.MAX_BATTERY_SUPPLY_volts / MAX_VELOCITY_mPsec; // 2.8256;
      public static double PID_KS = 0.01;


      public static final double STATOR_CURRENT_LIMIT_amps = 80;
      public static final double SUPPLY_CURRENT_LIMIT_amps = 60;
      

    }
    /** The data for the SWERVE Steer motors */
    public static class STEER {
      public static double PID_KP = 0.2;
      public static double PID_KI = 0.15;

      private static final double MOTOR_PINION_TEETH = 8.0;
      private static final double MOTOR_DRIVE_GEAR_TEETH = 24.0;
      private static final double GEAR_1_DRIVE_TEETH = 14.0;
      private static final double GEAR_1_DRIVEN_TEETH = 72.0;
      private static final double CANCODER_GEAR_RATIO = 1.0;
      public static final double GEAR_RATIO = 1
          / ((MOTOR_PINION_TEETH / MOTOR_DRIVE_GEAR_TEETH)
              * (GEAR_1_DRIVE_TEETH / GEAR_1_DRIVEN_TEETH));
      public static final double GEAR_RATIO_TO_CANCODER = GEAR_RATIO * CANCODER_GEAR_RATIO;
      public static final double STATOR_CURRENT_LIMIT_amps = 50;
      public static final double SUPPLY_CURRENT_LIMIT_amps = 50;
    }
  }
   public static class CHASSIS {
    public static final double FRONT_SWERVE_X_POSITION_m = 352.44 / 1000.0;
    public static final double FRONT_SWERVE_Y_POSITION_m = 0.0;
    public static final double BACK_LEFT_SWERVE_X_POSITION_m = -255.18 / 1000.0;
    public static final double BACK_LEFT_SWERVE_Y_POSITION_m = 243.1 / 1000.0;
    public static final double BACK_RIGHT_SWERVE_X_POSITION_m = -255.18 / 1000.0;
    public static final double BACK_RIGHT_SWERVE_Y_POSITION_m = -243.1 / 1000.0;
    public static final double WHEELBASE_DIAMETER_m = 0.705;
    private static final double WHEELBASE_CIRCUMFERENCE_m = Math.PI * WHEELBASE_DIAMETER_m;
    private static final double WHEELBASE_mPrad = WHEELBASE_CIRCUMFERENCE_m / (2 * Math.PI);
  }

  public static class DRIVETRAIN {
    public static final double TURN_KP = 0.45;
    public static final double TURN_KI = 0.20;
    public static final double TURN_KD = 0.0;
    public static final double TURN_DEADBAND_rad = 0.02;
    public static final double AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m = 0.0254; // 1 inches
    public static final double AUTO_DRIVE_POSE_ANGLE_TOLERANCE_deg = 1.0; 
    public static volatile DriveMode driveMode = DriveMode.FIELD_CENTRIC;
    public static final Translation2d ZERO_CENTER_OF_ROTATION_m = new Translation2d();
    
    public static volatile Translation2d centerOfRotation_m = new Translation2d();
    public static volatile double driveSpeedActual_mps = 0.0;
    public static volatile double driveSpeedRequested_mps = 0.0;
    public static volatile double driveSpeedError_mps = 0.0;
    public static final Vector<N3> STD_DEV_HIGH = VecBuilder.fill(0.15,0.15,0.15);
    public static final Vector<N3> STD_DEV_LOW = VecBuilder.fill(0.25,0.25,0.25);
    public static final double DRIVE_SPEED_LOW_mps = 2;
    public static double turnPIDErrorDerivative = Double.POSITIVE_INFINITY;
    public static boolean isAutoDriveEnabled = true;
  }
  public static class CORAL {
    public static final double ROTATE_GEAR_RATIO = 4*4*3*(84/34); // TODO: find gear ratio
    public static final double INTAKE_RANGE_MIN_mm = 0.1;
    public static volatile double spinnerHoldPosition = 0.0;
    //public static volatile CoralArmState armState = CoralArmState.START;
    public static volatile double spinSpeed = 0.6;
    public static final double ARM_BACK_LIMIT = -70;
    public static final double ARM_FROUNT_LIMIT = 60;
    public static final double SPINNER_MOTOR_MAX_VELOCITY_rotPsec = 7200.0 / 60.0;

    
  }
  public static class LIFT {
    public static final double SPROCKET_PITCH_mm = 44.4754;
    public static final double SPROCKET_CIRCUMFERENCE_mm = Math.PI * SPROCKET_PITCH_mm;
    public static final double GEARBOX_RATIO = 36.0;
    public static final double MOTOR_ROTATIONS_TO_LIFT_DISTANCE_rotPmm = GEARBOX_RATIO / SPROCKET_CIRCUMFERENCE_mm;
    public static final double MOTOR_MAX_VELOCITY_rotPsec = 5800.0 / 60.0;
    public static final double MAX_VELOCITY_mmPsec = MOTOR_MAX_VELOCITY_rotPsec / MOTOR_ROTATIONS_TO_LIFT_DISTANCE_rotPmm;
    public static final double MAX_HEIGHT_mm = 400;
    public static final double MAX_TIME_TO_RAISE_sec = MAX_HEIGHT_mm / MAX_VELOCITY_mmPsec;
    public static final double MIN_HEIGHT_mm = 0;
  
  }
    public static class VISION {
    public static volatile AprilTagAlignState aprilTagAlignState = AprilTagAlignState.NONE;
    public static volatile int aprilTagRequestedID = 0;
    public static volatile double aprilTagAngle_deg = 0;
    public static volatile double aprilTagDistance_m = 0;
    public static volatile boolean isTargetAprilTagFound = false;
    public static volatile TagFoundState tagState = TagFoundState.EMPTY;
    public static volatile Pose2d aprilTagRequestedPose = new Pose2d();
    public static volatile boolean isGlobalPoseResetActive = true; // Not implemented
    public static final double AMBIGUITY_SETPOINT = 0.075;
    public static final double TARGET_DISTANCE_AUTO_MAX_m = 2.5;
    public static final double TARGET_DISTANCE_AUTO_MIN_m = 1;
    public static final double TARGET_DISTANCE_VISION_MAX_m = 3;
    public static final double TARGET_DISTANCE_VISION_MIN_m = 0.5;
    public static volatile double leftTargetAmbiguity = -1.0;
    public static volatile double rightTargetAmbiguity = -1.0;
    public static volatile Field2d field2d = new Field2d();
    public static volatile Optional<Pose2d> pose2d = Optional.empty();
    public static double initTargetIDAngle = 0.0;
  }
  public static class CORALLIFT{
    public static CoralLiftState state = CoralLiftState.ZERO;
  }
  public static class FIELD{
    public static double TAG_TO_POST_m = 0.16429;
  }
}
