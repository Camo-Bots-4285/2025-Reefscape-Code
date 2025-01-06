/**OverView
 * This code will control when the intake move in or out
 * 
 */
package frc.robot.subsystems.MechanicalSystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MovementMotor extends SubsystemBase {

    private IntakeSubsystem intakeSubsystem;  // Reference to intakeSubsystem

    public MovementMotor(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;  // Set the reference to intakeSubsystem

        //TODO add motors and PID plus FF for good contorl
    }

      // Method to get MovementMotor position
      public double getMovementMotorPosition(){
        return 0.0;  // Placeholder for MovementMotor position
    }

    // Method to get MovementMotor position
    public double getMovementMotorVelocity(){
        return 0.0;  // Placeholder for MovementMotor Velocity
    }


            // Method to get MovementMotor position
    public double getMovementMotorAccleration(){
        return 0.0;  // Placeholder for MovementMotor Accleration
    }
}