/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode shows how to use a color sensor in a generic
 * way, regardless of which particular make or model of color sensor is used. The OpMode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 * There will be some variation in the values measured depending on the specific sensor you are using.
 *
 * You can increase the gain (a multiplier to make the sensor report higher values) by holding down
 * the A button on the gamepad, and decrease the gain by holding down the B button on the gamepad.
 *
 * If the color sensor has a light which is controllable from software, you can use the X button on
 * the gamepad to toggle the light on and off. The REV sensors don't support this, but instead have
 * a physical switch on them to turn the light on and off, beginning with REV Color Sensor V2.
 *
 * If the color sensor also supports short-range distance measurements (usually via an infrared
 * proximity sensor), the reported distance will be written to telemetry. As of September 2020,
 * the only color sensors that support this are the ones from REV Robotics. These infrared proximity
 * sensor measurements are only useful at very small distances, and are sensitive to ambient light
 * and surface reflectivity. You should use a different sensor if you need precise distance measurements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "RAGraph", group = "Sensor")
@Config
public class RAGraph extends LinearOpMode {

  /** The colorSensor field will contain a reference to our color sensor hardware object */
  NormalizedColorSensor colorSensor;

  /** The relativeLayout field is used to aid in providing interesting visual feedback
   * in this sample application; you probably *don't* need this when you use a color sensor on your
   * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
  View relativeLayout;
  float first, second, third, fourth, fifth, sixth, seventh, eighth, ninth, tenth, eleventh, twelfth, thirteenth;
  float fourteenth, fifteenth, sixteenth, seventeenth, eighteenth, nineteenth, twentieth;
  public static float averageSat = 0;

  /*
   * The runOpMode() method is the root of this OpMode, as it is in all LinearOpModes.
   * Our implementation here, though is a bit unusual: we've decided to put all the actual work
   * in the runSample() method rather than directly in runOpMode() itself. The reason we do that is
   * that in this sample we're changing the background color of the robot controller screen as the
   * OpMode runs, and we want to be able to *guarantee* that we restore it to something reasonable
   * and palatable when the OpMode ends. The simplest way to do that is to use a try...finally
   * block around the main, core logic, and an easy way to make that all clear was to separate
   * the former from the latter in separate methods.
   */
  @Override public void runOpMode() {

    // Get a reference to the RelativeLayout so we can later change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    try {
      runSample(); // actually execute the sample
    } finally {
      // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
      // as pure white, but it's too much work to dig out what actually was used, and this is good
      // enough to at least make the screen reasonable again.
      // Set the panel back to the default color
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.WHITE);
        }
      });
      }
  }

  protected void runSample() {
    // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
    // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
    // can give very low values (depending on the lighting conditions), which only use a small part
    // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
    // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
    // colors will report at or near 1, and you won't be able to determine what color you are
    // actually looking at. For this reason, it's better to err on the side of a lower gain
    // (but always greater than  or equal to 1).
    float gain = 2;

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];

    // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
    // state of the X button on the gamepad
    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;

    // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
    // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
    // the values you get from ColorSensor are dependent on the specific sensor you're using.
    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }

    // Wait for the start button to be pressed.
    waitForStart();

    // Loop until we are asked to stop
    while (opModeIsActive()) {
      // Explain basic gain information via telemetry
      telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
      telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

      // Update the gain value if either of the A or B gamepad buttons is being held
      if (gamepad1.a) {
        // Only increase the gain by a small amount, since this loop will occur multiple times per second.
        gain += 0.005;
      } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
        gain -= 0.005;
      }

      // Tell the sensor our desired gain value (normally you would do this during initialization,
      // not during the loop)
      colorSensor.setGain(gain);

      // Check the status of the X button on the gamepad
      xButtonCurrentlyPressed = gamepad1.x;

      // If the button state is different than what it was, then act
      if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
        // If the button is (now) down, then toggle the light
        if (xButtonCurrentlyPressed) {
          if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(!light.isLightOn());
          }
        }
      }
      xButtonPreviouslyPressed = xButtonCurrentlyPressed;

      // Get the normalized colors from the sensor
      NormalizedRGBA colors = colorSensor.getNormalizedColors();

      /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
       * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
       * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
       * for an explanation of HSV color. */

      // Update the hsvValues array by passing it to Color.colorToHSV()
      Color.colorToHSV(colors.toColor(), hsvValues);

      averageSat = first + second+third+fourth+fifth+sixth+seventh+eighth+ninth+tenth+eleventh+twelfth+thirteenth + fourteenth+fifteenth+sixteenth+ seventeenth+ eighteenth+ nineteenth+ twentieth;
      averageSat /= 20;
      if(first == 0) {
        first = hsvValues[1];
      } else {
        first = second;
      }
      if(second == 0) {
        second = hsvValues[1];
      } else {
        second = third;
      }
      if(third == 0) {
        third = hsvValues[1];
      } else {
        third = fourth;
      }
      if(fourth == 0) {
        fourth = hsvValues[1];
      } else {
        fourth = fifth;
      }
      if(fifth == 0) {
        fifth = hsvValues[1];
      } else {
        fifth = sixth;
      }
      if(sixth == 0) {
        sixth = hsvValues[1];
      } else {
        sixth = seventh;
      }
      if(seventh == 0) {
        seventh = hsvValues[1];
      } else {
        seventh = eighth;
      }
      if(eighth == 0) {
        eighth = hsvValues[1];
      } else {
        eighth = ninth;
      }
      if(ninth == 0) {
        ninth = hsvValues[1];
      } else {
        ninth = tenth;
      }
      if(tenth == 0) {
        tenth = hsvValues[1];
      } else {
        tenth = eleventh;
      }
      if(eleventh == 0) {
        eleventh = hsvValues[1];
      } else {
        eleventh = twelfth;
      }
      if(twelfth == 0) {
        twelfth = hsvValues[1];
      } else {
        twelfth = thirteenth;
      }
      if(thirteenth == 0) {
        thirteenth = hsvValues[1];
      } else {
        thirteenth = fourteenth;
      }
      if(fourteenth == 0) {
        fourteenth = hsvValues[1];
      } else {
        fourteenth = fifteenth;
      }
      if(fifteenth == 0) {
        fifteenth = hsvValues[1];
      } else {
        fifteenth = sixteenth;
      }
      if(sixteenth == 0) {
        sixteenth = hsvValues[1];
      } else {
        sixteenth = seventeenth;
      }
      if(seventeenth == 0) {
        seventeenth = hsvValues[1];
      } else {
        seventeenth = eighteenth;
      }
      if(eighteenth == 0) {
        eighteenth = hsvValues[1];
      } else {
        eighteenth = nineteenth;
      }
      if(nineteenth == 0) {
        nineteenth = hsvValues[1];
      } else {
        nineteenth = twentieth;
      }
      twentieth = hsvValues[1];




      telemetry.addLine()
              .addData("Saturation", "%.3f", hsvValues[1]);
      telemetry.addLine()
              .addData("first", "%.3f", first)
              .addData("second", "%.3f", second)
              .addData("first", "%.3f", third)
              .addData("first", "%.3f", fourth)
              .addData("first", "%.3f", fifth)
              .addData("first", "%.3f", sixth)
              .addData("first", "%.3f", seventh)
              .addData("first", "%.3f", eighth)
              .addData("first", "%.3f", ninth)
              .addData("first", "%.3f", tenth)
              .addData("first", "%.3f", eleventh)
              .addData("first", "%.3f", twelfth)
              .addData("first", "%.3f", thirteenth)
              .addData("first", "%.3f", fourteenth)
              .addData("first", "%.3f", fifteenth)
              .addData("first", "%.3f", sixteenth)
              .addData("first", "%.3f", seventeenth)
              .addData("first", "%.3f", eighteenth)
              .addData("first", "%.3f", nineteenth)
              .addData("first", "%.3f", twentieth);
      telemetry.addLine()
                      .addData("avg: ","%.3f", averageSat);


      telemetry.update();

    }
  }
}
