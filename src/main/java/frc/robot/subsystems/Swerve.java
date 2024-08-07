//CREATED USING A TEMPLATE FROM https://github.com/dirtbikerxz/BaseTalonFXSwerve
package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final SwerveDriveOdometry swerveOdometry; // Odometry; Used to estimate position of the robot on the field.
                                                    // Primarily used for autonomous.
  private final SwerveModule[] swerveMods; // Array to hold the modules of the swerve; makes it easier to apply methods
                                           // as we can use loops.
 // Our gyro. Used for measuring rotation and heading.
  private final static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final Field2d field = new Field2d();

  

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public Swerve() {
    gyro.reset();
    gyro.calibrate();
    // Quick way for us to apply the settings of each module, while keeping ID's and
    // angle offsets specific to each one.
    
    swerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants),
      new SwerveModule(1, Constants.Swerve.Mod1.constants),
      new SwerveModule(2, Constants.Swerve.Mod2.constants),
      new SwerveModule(3, Constants.Swerve.Mod3.constants)
  };
    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    // Uses kinematics, gyro, and module positions. Only time this would change is
    // if we are using a different gyro.
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, gyro.getRotation2d(), getModulePositions());


    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  

  /**
   * One of the most important methods. Using the boolean fieldRelative, it
   * creates chassis speeds for us to apply to the modules.
   * We use the desired angle and desired translation (meaning x and y change bc
   * swerve can move diagonally and side to side)
   * We then use those values to create swerve module states we can apply to each
   * module.
   * 
   * VARIABLES TO UNDERSTAND:
   * fieldRelative - driving relative to the field. Meaning regardless of robot
   * heading it moves in tandem to field's cooordinate plane.
   * translation - the position on a coordinate plane, this is needed bc we want
   * to find the quickest way from x1, y1, to x2, y2
   * 
   * isOpenLoop - basically just asking if it is teleop or autonomous. Visit the
   * setDesiredState method in SwerveModule to learn more.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getHeading())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // Setting each module to the desired state, (Wanted speed and angle)
    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds to SwerveModuleStates
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Desaturate wheel speeds to ensure they are within the allowable range
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxSpeed);

    // Set the desired state for each swerve module
    for (SwerveModule mod : swerveMods) {
        mod.setDesiredState(moduleStates[mod.moduleNumber], false); // assuming closed-loop control
    }
}

  public ChassisSpeeds getRobotRelativeSpeeds() {
    var frontLeftState = swerveMods[0].getState();
    var frontRightState = swerveMods[1].getState();
    var backLeftState = swerveMods[2].getState();
    var backRightState = swerveMods[3].getState();

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);

    // Getting individual speeds
    double forward = chassisSpeeds.vxMetersPerSecond;
    double sideways = chassisSpeeds.vyMetersPerSecond;
    double angular = chassisSpeeds.omegaRadiansPerSecond;

    return chassisSpeeds;

  }

  /**
   * Gets heading of the robot.
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }


  public void resetOdometry(Pose2d pose){
    swerveOdometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()), new SwerveModulePosition[] {
      swerveMods[0].getPosition(),
      swerveMods[1].getPosition(),
      swerveMods[2].getPosition(),
      swerveMods[3].getPosition(), 
  }, pose); 
}

  
  /**
   * Gets position of the robot on the field. (2D)
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * 
   * @returns the position of each module
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveMods) {
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  /**
   * 
   * @returns The speed and angle of each module.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : swerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getGyroYaw(), getModulePositions());
    field.setRobotPose(swerveOdometry.getPoseMeters());
    for (SwerveModule mod : swerveMods) {

      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);

    }
  }

  // If you need a reminder of Yaw, Pitch, and Roll this website does a good job:
  // https://automaticaddison.com/yaw-pitch-and-roll-diagrams-using-2d-coordinate-systems/#:~:text=The%20robot%20turns%20slightly%20to,z%2Daxis%20is%20yaw%20rotation.
  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

}