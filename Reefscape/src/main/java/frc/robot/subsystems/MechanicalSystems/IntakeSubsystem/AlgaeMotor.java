/**OverView
 * This will control the top section of rollers that are to intake the algae
 */
package frc.robot.subsystems.MechanicalSystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeMotor extends SubsystemBase{
    private IntakeSubsystem intakeSubsystem;  // Reference to intakeSubsystem

    public AlgaeMotor(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;  // Set the reference to intakeSubsystem

        //TODO add motors and PID plus FF for good contorl

    }

     // Method to get wrist position
     public double getAlgaeMotorPosition(){
        return 0.0;  // Placeholder for wrist position
    }

    // Method to get wrist position
    public double getAlgaeMotorVelocity(){
        return 0.0;  // Placeholder for wrist Velocity
    }


            // Method to get wrist position
    public double getAlgaeMotorAccleration(){
        return 0.0;  // Placeholder for wrist Accleration
    }
}
