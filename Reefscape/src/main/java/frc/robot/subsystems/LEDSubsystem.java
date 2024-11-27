package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer;


public class LEDSubsystem extends SubsystemBase {

  /** Creates a new LEDSubsystem. */

  private DigitalOutput led_is_blue;
  private DigitalOutput led_is_red;
  private DigitalOutput led_is_win;
  // private DigitalOutput one_true;
  // private DigitalOutput ten_true;
  // private DigitalOutput hundred_true;
  // private DigitalOutput thousand_true;

  public LEDSubsystem() {
    led_is_blue = new DigitalOutput(LEDConstants.DIO_LED_IS_BLUE);
    led_is_red = new DigitalOutput(LEDConstants.DIO_LED_IS_RED);
    led_is_win = new DigitalOutput(LEDConstants.DIO_LED_WIN);

    led_is_blue.set(true);
    led_is_red.set(true);
    led_is_win.set(true);

    //These value will be used for matrix comunication with Matts Pie
    //Matrix communication will allow you to multiply for by two for every port add as opossied to adding one
    //So here is the problem with Matrix communication you can not send more then one combination at a time.

    //Solution 1) make smaller matrices that each are responsabe for differnt feilds you make run out of ports this way

    //Solution 2) make the RIO and the PIE have a converstion the PIE can ask a question (send a matrix)
    // and the Rio will interperat that matrix and send back the corresponding result if you put it in a loop that continuesly runs
    //The more you layer this the better it will work. For example the RIO tells PIE it is getting ready for auto so the PIE
    //will ask what allaince color and if the robot needs to get moved. Then when it goes to TeleOp it may ask new questions
    //like if it has a game piece. it does not need to reask alliance color becasue that does no change between TeleOp and Auto

   //Solution 3)Program LED on the RIO this could slow down the RIO but might be easier depending on how well MATT knows C++ ;D

    //2 inputs will give you 4 possible out comes
    //3 inputs will give you 8 possible out comes
    //4 inputs will give you 16 possible out comes
    //5 inputes will give you 32 possible out comes
    //6 inputes will give you 64 possible out comes

    // one_true = new DigitalOutput(LEDConstants.DIO_One);
    // ten_true = new DigitalOutput(LEDConstants.DIO_Ten);
    // hundred_true = new DigitalOutput(LEDConstants.DIO_Hundred);
    // thousand_true = new DigitalOutput(LEDConstants.DIO_Thousand);

    // one_true.set(true);
    // ten_true.set(true);
    // hundred_true.set(true);
    // thousand_true.set(true);
  }

  @Override
  public void periodic() {
    boolean ally = Constants.isRed;
    if (ally) {
        if (Constants.isRed == true) {
            //System.out.println("Alliance Red");
            set_red(LEDConstants.DIO_ENABLE);
            set_blue(LEDConstants.DIO_DISABLE);
        }
        if (Constants.isRed == false) {
            //System.out.println("Alliance Blue");
            set_red(LEDConstants.DIO_DISABLE);
            set_blue(LEDConstants.DIO_ENABLE);
        }
    }
    else {
       // System.out.println("I'm unset!");
        set_red(LEDConstants.DIO_DISABLE);
        set_blue(LEDConstants.DIO_DISABLE);
        //set_win(LEDConstants.DIO_DISABLE);
    }
  

  //This is a table to map out all the posible combinations of number and what they mean
  //1)  00000 = CloseEnough
  //2)  00001 = MoveRight
  //3)  00010 = MoveLeft
  //4)  00011 = MoveBackward
  //5)  00100 = MoveForwards
  //6)  00101 = RotateCounterClockWise
  //7)  00110 = RotateClockWise
  //8)  00111 =
  //9)  01000 =
  //10) 01001 =
  //12) 01010 =
  //13) 01011 =
  //14) 01100 =
  //15) 01101 =
  //16) 01110 =
  //17) 01111 =
  //18) 10000 =
  //19) 10001 =
  //20) 10010 =
  //21) 10011 =
  //22) 10100 =
  //23) 10101 =
  //24) 10110 =
  //25) 10111 =
  //26) 11000 =
  //27) 11001 =
  //28) 11010 =
  //29) 11011 =
  //30) 11100 =
  //31) 11101 =
  //32) 11110 =
  //33) 11111 = 
  


  // if(SelfDriving.AutoSetUpCloseEnough == true){
  //   set_One(LEDConstants.DIO_DISABLE);
  //   set_Ten(LEDConstants.DIO_DISABLE);
  //   set_Hundred(LEDConstants.DIO_DISABLE);
  //   set_Thousand(LEDConstants.DIO_DISABLE);
  // }
  // if(SelfDriving.AutoSetUpMoveRight == true){
  //   set_One(LEDConstants.DIO_ENABLE);
  //   set_Ten(LEDConstants.DIO_DISABLE);
  //   set_Hundred(LEDConstants.DIO_DISABLE);
  //   set_Thousand(LEDConstants.DIO_DISABLE);
  // }
  // if(SelfDriving.AutoSetUpMoveLeft == true){
  //   set_One(LEDConstants.DIO_DISABLE);
  //   set_Ten(LEDConstants.DIO_ENABLE);
  //   set_Hundred(LEDConstants.DIO_DISABLE);
  //   set_Thousand(LEDConstants.DIO_DISABLE);
  // }
  // if(SelfDriving.AutoSetUpMoveBackward == true){
  //   set_One(LEDConstants.DIO_ENABLE);
  //   set_Ten(LEDConstants.DIO_ENABLE);
  //   set_Hundred(LEDConstants.DIO_DISABLE);
  //   set_Thousand(LEDConstants.DIO_DISABLE);
  // }
  // if(SelfDriving.AutoSetUpMoveForward == true){
  //   set_One(LEDConstants.DIO_DISABLE);
  //   set_Ten(LEDConstants.DIO_DISABLE);
  //   set_Hundred(LEDConstants.DIO_DISABLE);
  //   set_Thousand(LEDConstants.DIO_DISABLE);
  // }
  //  if(SelfDriving.AutoSetUpRotateCounterClockWise == true){
  //   set_One(LEDConstants.DIO_ENABLE);
  //   set_Ten(LEDConstants.DIO_DISABLE);
  //   set_Hundred(LEDConstants.DIO_ENABLE);
  //   set_Thousand(LEDConstants.DIO_DISABLE);
  // }
  //  if(SelfDriving.AutoSetUpRotateCloskWise == true){
  //   set_One(LEDConstants.DIO_DISABLE);
  //   set_Ten(LEDConstants.DIO_ENABLE);
  //   set_Hundred(LEDConstants.DIO_ENABLE);
  //   set_Thousand(LEDConstants.DIO_DISABLE);
  // }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stop() {
    /*
     * Stubbed for now.
     */
  }

  public void set_blue(boolean blue_state) {
    led_is_blue.set(blue_state);
  }

  public void set_red(boolean red_state) {
    led_is_red.set(red_state);
  }

  public void set_win(boolean win_state) {
    led_is_win.set(win_state);
  }

  //   public void set_One(boolean one_state) {
  //   one_true.set(one_state);
  // }

  // public void set_Ten(boolean ten_state) {
  //   ten_true.set(ten_state);
  // }

  // public void set_Hundred(boolean hundred_state) {
  //   hundred_true.set(hundred_state);
  // }

  //  public void set_Thousand(boolean  thousand_state) {
  //    thousand_true.set(thousand_state);
  // }
}
