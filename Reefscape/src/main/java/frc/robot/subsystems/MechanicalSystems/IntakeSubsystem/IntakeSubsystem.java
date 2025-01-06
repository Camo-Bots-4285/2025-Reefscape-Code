/**OverView
 * This will control the all values of the intake making sure there is no 
 * confusion oon which game piece is being pick up.
 */
package frc.robot.subsystems.MechanicalSystems.IntakeSubsystem;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private MovementMotor movementMotorSubsystem;
    private CoralMotor coralMotorSubsystem;
    private MiddleMotor middleMotorSubsystem;
    private AlgaeMotor algaeMotorSubsystem;
    
   public static double DesiredPosition = IntakeConstants.Home_Intake;
   public static double DesiredGamePiece = 0;


 // Constructor, accepting MovmentMotor, CoralMotor,MiddleMotor, and AlgaeMoter as a parameters
 public IntakeSubsystem() {
    this.movementMotorSubsystem = movementMotorSubsystem;
    this.coralMotorSubsystem = coralMotorSubsystem;
    this.middleMotorSubsystem = middleMotorSubsystem;
    this.algaeMotorSubsystem = algaeMotorSubsystem;
 }

 public void setDisinatedPosition(int DesiredPosition){


 }

 public double getGamePieceToIntake(){
   return 0.0;
 }

}


   