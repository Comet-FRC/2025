package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Limelight;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveMods;
    public final Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID); //WIP
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

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

        swerveOdometry = new SwerveDriveOdometry(
                Constants.Swerve.SWERVE_KINEMATICS,
                getHeading(),
                getModulePositions()
        );

    }

    @Override
    public void periodic() {
        swerveOdometry.update(getHeading(), getModulePositions());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Absolute", mod.getAbsoluteAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Relative", mod.getAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  //takes the coordinate on field wants to go to, the rotation of it, whether or not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, getHeading())
              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
  //sets to top speed if above top speed
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

  //set states for all 4 modules
  setModuleStates(swerveModuleStates, isOpenLoop);

  }



    public double getYaw() {
        return (Constants.Swerve.INVERT_GYRO) ?
                Constants.Swerve.MAXIMUM_ANGLE - (gyro.getYaw().getValueAsDouble()) :
                gyro.getYaw().getValueAsDouble();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    

    

    public void setOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void resetOdometry() {
        setOdometry(new Pose2d());
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        zeroGyro(Constants.Swerve.GYRO_OFFSET);
    }

    public void zeroGyro(double rotation) {
        gyro.setYaw(rotation);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, true);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }
}