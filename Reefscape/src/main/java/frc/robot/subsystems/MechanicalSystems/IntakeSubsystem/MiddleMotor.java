/**OverView
 * This will control the middle section of rollers that are while intake the algae or coral
 */
package frc.robot.subsystems.MechanicalSystems.IntakeSubsystem;

public class MiddleMotor extends IntakeSubsystem {
    
    private IntakeSubsystem intakeSubsystem;  // Reference to intakeSubsystem

    public MiddleMotor(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;  // Set the reference to intakeSubsystem

        //TODO add motors and PID plus FF for good contorl
    }

          // Method to get MiddleMotor position
          public double getMiddleMotorPosition(){
            return 0.0;  // Placeholder for MiddleMotor position
        }
    
        // Method to get MiddleMotor position
        public double getMiddleMotorVelocity(){
            return 0.0;  // Placeholder for MiddleMotor Velocity
        }
    
    
                // Method to get MiddleMotor position
        public double getMiddleMotorAccleration(){
            return 0.0;  // Placeholder for MiddleMotor Accleration
        }
}
