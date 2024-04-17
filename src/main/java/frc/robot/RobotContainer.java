package frc.robot;

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
    }

    

      

    private void configureButtonBindings() {
        /* Driver Controls */

        

    }
    

    





    public Command getAutonomousCommand() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
    }

   

    
}