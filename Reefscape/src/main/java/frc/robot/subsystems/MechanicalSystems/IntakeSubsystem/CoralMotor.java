/**OverView
 * This will control the bottom section of rollers that are to intake the coral
 */
package frc.robot.subsystems.MechanicalSystems.IntakeSubsystem;

public class CoralMotor extends IntakeSubsystem {
    private IntakeSubsystem intakeSubsystem;  // Reference to intakeSubsystem

    public CoralMotor(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;  // Set the reference to intakeSubsystem

        //TODO add motors and PID plus FF for good contorl
        
    }

    
        // Method to getCoralMotor position
        public double getMiddleMotorPosition(){
            return 0.0;  // Placeholder forCoralMotor position
        }
    
        // Method to getCoralMotor position
        public double getMiddleMotorVelocity(){
            return 0.0;  // Placeholder forCoralMotor Velocity
        }
    
    
        // Method to getCoralMotor position
        public double getMiddleMotorAccleration(){
            return 0.0;  // Placeholder forCoralMotor Accleration
        }
}
