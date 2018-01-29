/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double SQUARE_LENGTH = 30.48;
  private static final double ANGLE_THRESHOLD = 10;
  private static final double LIGHTSENS_OFFSET = 3.5;
  private Odometer odometer;
  private SampleProvider lt;
  private float[] ltdata;
  private int lightVal;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(SampleProvider lt, float[] ltdata) throws OdometerExceptions {
	  this.lt =lt;
	  this.ltdata = ltdata;
    this.odometer = Odometer.getOdometer();

  }
  public boolean correctionTrigger() {
	   lt.fetchSample(ltdata, 0);
	   lightVal = (int)(ltdata[0] *100);
	   if( lightVal < 10) {
		   //Sound.beep();
		   return true;
	   }
	  return false;
  }
  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    double position[];
    double head = 0;
    int currentYQuad,currentXQuad;
    double newy, newx = 0;
    while (true) {
      correctionStart = System.currentTimeMillis();
      position = odometer.getXYT();
      head = position[2];
      
      if(correctionTrigger()) {
    	  if(head > 350 || head < 10) {// going up or down
    		  currentYQuad = (int) ((position[1] + 10)/ SQUARE_LENGTH);
    		  newy = SQUARE_LENGTH * currentYQuad-LIGHTSENS_OFFSET;
    		  odometer.setY(newy);
    		  Sound.buzz();
    	  }
    	  else if(Math.abs(head-180) < ANGLE_THRESHOLD) {
    		  currentYQuad = (int) ((position[1] + 10)/ SQUARE_LENGTH);
    		  newy = SQUARE_LENGTH * currentYQuad+LIGHTSENS_OFFSET;
    		  odometer.setY(newy);
    		  Sound.buzz();  
    	  }
    	  else if((Math.abs(head-90) < ANGLE_THRESHOLD)){
    		  currentXQuad = (int)((position[0] + 10)/SQUARE_LENGTH);
    		  newx = SQUARE_LENGTH * currentXQuad-LIGHTSENS_OFFSET;
    		  odometer.setX(newx);
    		  Sound.beep();
    	  }
    	  else {//right or left
    		  currentXQuad = (int)((position[0] + 10)/SQUARE_LENGTH);
    		  newx = SQUARE_LENGTH * currentXQuad+LIGHTSENS_OFFSET;
    		  odometer.setX(newx);
    		  Sound.beep();
    	  }
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
