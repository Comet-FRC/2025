//CREATED USING A TEMPLATE FROM https://github.com/dirtbikerxz/BaseTalonFXSwerve
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.lib.SwerveModuleConstants;
import frc.robot.util.math.Conversions;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    

    //MOTOR AND ENCODER DECLARATION
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    public CANcoder angleEncoder;

    //Basically a way of doing PID with a drivetrain but does not rely on feedback from encoders/CANCoders to calculate drive
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_S, Constants.Swerve.DRIVE_V, Constants.Swerve.DRIVE_A);


    //drive motor control requests; talonFX specific
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    //angle motor control requests; talonFX specific
    private final PositionVoltage anglePosition = new PositionVoltage(0);


    public SwerveModule(int ModuleNumber, SwerveModuleConstants moduleConstants) 
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        //Angle Motor
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);

        //Angle Encoder Config
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);

        //Drive Motor Config
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);

        resetToAbsolute();

    }
    
    /**
     * Sets speeds and angles of motors.
     * 
     * 
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); //this command basically finds the quickest way to rotate to the desired angle

        //this command then rotates the wheels using the current angle position (anglePosition) and then desired angle's rotations
        angleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations())); 

        setSpeed(desiredState, isOpenLoop); 
        /*
        Setting the speed; open loop refers to how the desired state is being reached.
        CLOSED LOOP = using feedback from encoders to see if it has reached desired position and maintaing same velocity. For driving the robot in auton, closedLoop is used.
        OPEN LOOP = no feedback, just set velocity and pray it works. For driving the robot during TeleOp, openLoop is used.
        */
    }

    /**
     *  Getting the state of the module and converting the numbers into speed and degrees
     */
    public SwerveModuleState getState()
    {
        //Using conversions from math we can move from rotations to meters, incorporating the wheel circumference. We also use Rotation2d to convert to 0-360.
        return new SwerveModuleState(Conversions.RPSToMPS(driveMotor.getVelocity().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), Rotation2d.fromRotations(angleMotor.getPosition().getValue()));

    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    /**
     * Returns distance measured by the wheel, based on the wheels circumference (may be subject to change) and the angle of the swerve drive.
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(angleMotor.getPosition().getValue())
        );
    }


    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }


    /**
     * This method can be thought as a way to align the wheels on the swerve drive to an absolute position.
     */
    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToTalon(waitForCANcoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.ANGLE_GEAR_RATIO);
        angleMotor.setPosition(absolutePosition);
    }
    
    private Rotation2d waitForCANcoder(){
        /* wait for up to 250ms for a new CANcoder position */
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().waitForUpdate(250).getValue());
    }
}
