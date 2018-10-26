/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.*;

/**
 * This is the OpMode for the driver period
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Template: Iterative OpMode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class PrimaryOpMode_Interactive extends OpMode
{
  /* Declare OpMode members. */
  //Clamp is obvious arm clamp
  //Arm is self explanatory
  private DcMotor leftMotor = null;
  private DcMotor rightMotor = null;
  private DcMotor armControl = null;
  private DcMotor clampControl = null;


  static final double     COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
  static final double     DRIVE_GEAR_REDUCTION    = (4.55/1.75);     // This is < 1.0 if geared UP
  static final double     WHEEL_DIAMETER_INCHES   = 4.55;     // For figuring circumference
  static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
          (WHEEL_DIAMETER_INCHES * 3.1415);
  static final double     Clamp_Open_Speed        = .5;
  static final double     WHEEL_CIRCUMFRENCE      = WHEEL_DIAMETER_INCHES * 3.1415;





  // private DcMotor leftMotor = null;
  // private DcMotor rightMotor = null;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init()
  {
    /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
    leftMotor    = hardwareMap.dcMotor.get("left_drive");
    rightMotor   = hardwareMap.dcMotor.get("right_drive");
    armControl   = hardwareMap.dcMotor.get("arm_drive");
    clampControl = hardwareMap.dcMotor.get("the_clapper");

    clampControl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armControl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    clampControl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    // Send telemetry message to signify robot waiting;
    telemetry.addData("Say", "Hello Driver");    //
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop()
  {
    //encoderDrive(Clamp_Open_Speed, (.4*WHEEL_CIRCUMFRENCE)); //Not done
  }


  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start()
  {
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop()
  {


    // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
    //Alternate code rightMotor.setPower(-gamepad1.right_stick_y);
    //Problem when motor doesn't turn off for aforementioned code
    //Sticks are switched because of issue with direction and joystick
    double left =   gamepad1.left_stick_y;
    double right = -gamepad1.right_stick_y;

    // To set joystick movement config for all directions
        /*
        double right = -throttle + direction;
        double left = throttle - direction;
        */
    rightMotor.setPower(right * 0.55);
    leftMotor.setPower(left * 0.55);

    //For flipping the clamps
    if (gamepad2.right_bumper)
      armControl.setPower(-.3);
    else if (gamepad2.left_bumper)
      armControl.setPower(0.3);
    else
      armControl.setPower(0);

    //For opening clamps to get cube
    if (gamepad2.b)
      clampControl.setPower(.2);
    else if (gamepad2.x)
      clampControl.setPower(-.2);
    else
      clampControl.setPower(0);
  }
  /*
   * Code to run ONCE after the driver hits STOP
   */

  @Override
  public void stop() {


  }
  public void encoderDrive(double speed, double distance)
  {
    int startPosition = clampControl.getCurrentPosition();
    int endPosition   = startPosition + (int)distance;

    clampControl.setTargetPosition(endPosition);
    clampControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    clampControl.setPower(Math.abs(speed));
  }
}
