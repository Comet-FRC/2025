//CREATED USING A TEMPLATE FROM https://github.com/dirtbikerxz/BaseTalonFXSwerve
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry; // Odometry; Used to estimate position of the robot on the field. Primarily used for autonomous.
    private final SwerveModule[] swerveMods; // Array to hold the modules of the swerve; makes it easier to apply methods as we can use loops.
    public final Pigeon2 gyro; // Our gyro. Used for measuring rotation and heading.

    public Swerve() {
        // Gyro Configuration; We implement the Pigeon and configure/apply settings into the Pigeon as well as zeroing it.
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID); // Not decided what Gyro we will use yet.
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        // Quick way for us to apply the settings of each module, while keeping ID's and angle offsets specific to each one.
        swerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS),
                new SwerveModule(1, Constants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS),
                new SwerveModule(2, Constants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS),
                new SwerveModule(3, Constants.Swerve.Mod3.SWERVE_MODULE_CONSTANTS)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        // Uses kinematics, gyro, and module positions. Only time this would change is if we are using a different gyro.
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, getHeading(),
                getModulePositions());

    }

    
    /**
     * One of the most important methods. Using the boolean fieldRelative, it creates chassis speeds for us to apply to the modules.
     * We use the desired angle and desired translation (meaning x and y change bc swerve can move diagonally and side to side) 
     * We then use those values to create swerve module states we can apply to each module.
     * 
     * VARIABLES TO UNDERSTAND:
     * fieldRelative - driving relative to the field. Meaning regardless of robot heading it moves in tandem to field's cooordinate plane. 
     * tranlation - the position on a coordinate plane, this is needed bc we want to find the quickest way from x1, y1, to x2, y2
     *   (yes we use the  Pythagorean theorem, who knew it would acc be useful)
     * isOpenLoop - basically just asking if it is teleop or autonomous. Visit the setDesiredState method in SwerveModule to learn more.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation, 
                getHeading()
            )
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        //Setting each module to the desired state, (Wanted speed and angle)
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Gets heading of the robot.
    */
    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    /**
     * Gets position of the robot on the field. (2D)
    */
    private Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    


    /**
     * 
     * @returns the position of each module
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod: swerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * 
     * @returns The speed and angle of each module.
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods){
          states[mod.moduleNumber] = mod.getState();
        }
        return states;
      }


      public void resetModulesToAbsolute(){
        for(SwerveModule mod: swerveMods){
            mod.resetToAbsolute();
        }
      }

    @Override
    public void periodic(){
    swerveOdometry.update(getGyroYaw(), getModulePositions());
    for(SwerveModule mod : swerveMods){

      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);   

    }
  }

  //If you need a reminder of Yaw, Pitch, and Roll this website does a good job: https://automaticaddison.com/yaw-pitch-and-roll-diagrams-using-2d-coordinate-systems/#:~:text=The%20robot%20turns%20slightly%20to,z%2Daxis%20is%20yaw%20rotation.
  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }
      

}