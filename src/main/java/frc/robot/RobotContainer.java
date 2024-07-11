package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Subsystems */
    private final Swerve swerve = new Swerve();
   

    public final static CommandXboxController driverXbox = new CommandXboxController(0);
     

    public final static CommandXboxController opXbox = new CommandXboxController(1);

    private static SendableChooser<Command> autoChooser;
 
   


  

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */

    public RobotContainer() {

        

        // Register default commands/controller axis commands
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driverXbox.getLeftY(), // Ordinate Translation
                        () -> -driverXbox.getLeftX(), // Coordinate Translation
                        () -> -driverXbox.getRightX(), // Rotation
                        driverXbox.b() // Robot-centric trigger
                )
        );

        
        configureButtonBindings();


        autoChooser = AutoBuilder.buildAutoChooser();


    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    

      

    private void configureButtonBindings() {
        /* Driver Controls */

        

    }
    

    





    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

        //For single path following
        //return AutoBuilder.followPathCommand(path);

    }

   

    
}