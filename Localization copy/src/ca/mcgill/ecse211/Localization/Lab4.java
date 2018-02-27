package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Odometry.*;
import ca.mcgill.ecse211.Localization.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Audio;

public class Lab4 {
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final EV3ColorSensor colorSensorBlock = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.05;
	public static final double TILE_SIZE = 30.48;
	public static Odometer odometer;
	private static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private static EV3UltrasonicSensor usSensorBlock = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	public static Audio noisemaker = LocalEV3.get().getAudio();
	private static UltrasonicLocalizer usLocalizer;
	private static Navigation nav;
	private static LightLocalizer liLocalizer;
	private static int[] LL = {1,1};
	private static int[] UR = {3,3};
	public static double xyt[] = new double[3];
	public static double currentX; 			//current values represent robot's X,Y,theta
	public static double currentY;
	public static double currentTheta;
	public static double deltaTheta;
	public static int[][] waypoints = {{LL[0], LL[1]}, {UR[0], LL[1]}, {UR[0], UR[1]}, {LL[0], UR[1]}, {LL[0], LL[1]}};
	public static double path_angle;		//angle that is calculated from the normal
	public static int wpCtr = 0;        //counter for order of the waypoints
	public static boolean foundBlock;
	public static Searchin search;
	private static Waypoints waypointer;
	public static float[] sample = new float[3];



	public static void main(String[] args) throws OdometerExceptions{
		int buttonChoice;
		nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK, odometer);
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		waypointer = new Waypoints(nav, odometer);
		search = new Searchin(leftMotor, rightMotor, odometer, nav, usSensorBlock, usSensor, colorSensorBlock);
		Display odometryDisplay = new Display(lcd);
		
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
		do {
			// clear the display
			lcd.clear();
			// ask the user for rising edge or falling edge
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising | Falling", 0, 2);
			lcd.drawString(" Edge  |  Edge  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_DOWN && buttonChoice != Button.ID_UP);
		
		if(buttonChoice == Button.ID_UP){
			/*
			 * IF UP:
			 * - Starting position of bot is assumed to be 0,0,0
			 * - This attempts to use the waypoint navigation as a thread and the light polling inside this class
			 */
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread waypointThread = new Thread(waypointer);
			waypointThread.start();
			
			try {
				Thread.sleep(10000); //sleep to wait for it to reach LL
			} catch (Exception e) {}
			
			while(true){
				float dist = search.getDistanceLeft();
				if(dist < 50 && dist > 8){ //if the left US reads a block or false positive
					foundBlock = true;
					waypointThread.interrupt();
					//block has been detected
					leftMotor.stop(true);
					rightMotor.stop();
					nav.turnTo(-90); //turn right on a dime to get front US to face the block
					try{
						Thread.sleep(1500); //wait for turn to complete
					}catch (Exception e){
						
					}
					float frontDist = search.getDistanceFront();
					if(frontDist < 50 && dist > 8){//check if front US sensor also reads a block
						leftMotor.forward();
						rightMotor.forward();
						while(true){ //as the bot is approaching the block, repeatedly poll the light sensor
							colorSensorBlock.getRGBMode().fetchSample(sample, 0);
							if(sample[0] > 0.01){ //UNFINISHED COLOR DETECTION
								leftMotor.stop(true);
								rightMotor.stop();
								Lab4.noisemaker.systemSound(3);
								break;
							}
						}
					}else{ //WAS A FALSE POSITIVE -- TURN BACK TO CORRECT DIRECTION
						nav.turnTo(90); //turn right on a dime
						try{
							Thread.sleep(1500);//wait for turn to complete
						}catch (Exception e){
							
						}
						leftMotor.forward();
						rightMotor.forward();
						
						try{
							Thread.sleep(1000);
						}catch (Exception e){}
						
						leftMotor.stop(true);
						rightMotor.stop();
					}
					Waypoints.wpCtr--;//decrement Waypoints waypoint counter to hopefully get it to turn to
										//and head to the proper waypoint
				}
			}
		} else if (buttonChoice == Button.ID_LEFT) {
			/*
			 * THIS OPTION PERFROMS FULL DEMO
			 * - Uses Rising edge US localization
			 * - Uses light localization to orient at 0,0,0
			 * - Performs waypoint navigation in this class and uses Searching thread for light polling
			 */
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
			usLocalizer = new UltrasonicLocalizer(odometer, nav, usSensor, 2, leftMotor, rightMotor);
			try{
				Thread.sleep(1000); //sleep thread to give ultrasonic localizer time to instantiate
			} catch (Exception e){
				noisemaker.systemSound(4);
			}
			usLocalizer.Localize();

			//begin light sensor localization
			liLocalizer = new LightLocalizer(odometer, nav, colorSensor, leftMotor, rightMotor);
			liLocalizer.Localize();

			do { //WAIT FOR USER INPUT BEFORE HEADING TO LL AND SEARCHING
				buttonChoice = Button.waitForAnyPress(); 
			} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_DOWN && buttonChoice != Button.ID_UP);
			
			Thread searchThread = new Thread(search); //does not start running thread here, starts when going to second waypoint

			while(wpCtr < waypoints.length){
				if(foundBlock){
					wpCtr--;
					foundBlock = false;
				}
				xyt = odometer.getXYT();				//get the robots position on the grid
				currentX = xyt[0];						//place data in its respective variables
				currentY = xyt[1];
				currentTheta = xyt[2];
				/*
				 * The following calculations figure out the desired angle that the bot needs to turn.
				 * It uses arctan2 to get the angle in polar coordinates from (0,0) so for every coordinate
				 * it will calculate the angle for the implicit coordinate in relation to (0,0). For example:
				 * if the bot is at (2,2,45) and it needs to turn to (2,0), the logic below will figure out the angle
				 * needed to turn to (0,-2) and then subtract the current angle of the bot.
				 * */
				if(wpCtr > 0){//need to make sure this is not performed for the first coordinate as we assume the bot starts at 0,0
					if(wpCtr == 1) searchThread.start();
					if(waypoints[wpCtr][0] >= 0 && waypoints[wpCtr][1] >= 0){										//angles are from the y-axis instead of x-axis
						path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE)));  		//angles in positive xy-plane
					}else if(waypoints[wpCtr][1] < 0){							
						path_angle = 180+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE)));	//angles in the negative y-plane
					}else{
						path_angle = 360+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE))); 	//angles in negative x-plane and positive y-plane
					}
				}else{
					if(waypoints[wpCtr][0] >= 0 && waypoints[wpCtr][1] >= 0){										//angles are from the y-axis instead of x-axis
						path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);  		//angles in positive xy-plane
					}else if(waypoints[wpCtr][1] < 0){							
						path_angle = 180+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);	//angles in the negative y-plane
					}else{
						path_angle = 360+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]); 	//angles in negative x-plane and positive y-plane
					}
				}

				deltaTheta = path_angle-currentTheta;
				nav.turnTo(deltaTheta);
				nav.travelTo(waypoints[wpCtr][0]*TILE_SIZE, waypoints[wpCtr][1]*TILE_SIZE);
				if(foundBlock){
					wpCtr--;
					foundBlock = false;
				}
				wpCtr++;
			}

		}else if(buttonChoice == Button.ID_RIGHT){
			//DEBUGGING/TESTING OPTION FOR BOT
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			//search for blocks
			Thread searchThread = new Thread(search);
			while(wpCtr < waypoints.length){
				if(foundBlock){
					Lab4.noisemaker.systemSound(2);
					try{
						Thread.sleep(10000);
					}catch (Exception e){}
					wpCtr--;
					foundBlock = false;
				}
				xyt = odometer.getXYT();				//get the robots position on the grid
				currentX = xyt[0];						//place data in its respective variables
				currentY = xyt[1];
				currentTheta = xyt[2];
				/*
				 * The following calculations figure out the desired angle that the bot needs to turn.
				 * It uses arctan2 to get the angle in polar coordinates from (0,0) so for every coordinate
				 * it will calculate the angle for the implicit coordinate in relation to (0,0). For example:
				 * if the bot is at (2,2,45) and it needs to turn to (2,0), the logic below will figure out the angle
				 * needed to turn to (0,-2) and then subtract the current angle of the bot.
				 * */
				if(wpCtr > 0){//need to make sure this is not performed for the first coordinate as we assume the bot starts at 0,0
					if(wpCtr == 1) searchThread.start();
					if(waypoints[wpCtr][0] >= 0 && waypoints[wpCtr][1] >= 0){										//angles are from the y-axis instead of x-axis
						path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE)));  		//angles in positive xy-plane
					}else if(waypoints[wpCtr][1] < 0){							
						path_angle = 180+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE)));	//angles in the negative y-plane
					}else{
						path_angle = 360+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE))); 	//angles in negative x-plane and positive y-plane
					}
				}else{
					if(waypoints[wpCtr][0] >= 0 && waypoints[wpCtr][1] >= 0){										//angles are from the y-axis instead of x-axis
						path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);  		//angles in positive xy-plane
					}else if(waypoints[wpCtr][1] < 0){							
						path_angle = 180+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);	//angles in the negative y-plane
					}else{
						path_angle = 360+(180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]); 	//angles in negative x-plane and positive y-plane
					}
				}
				try{
					odoThread.run();
				}catch(Exception e){}
				
				deltaTheta = path_angle-currentTheta;
				nav.turnTo(deltaTheta);
				nav.travelTo(waypoints[wpCtr][0]*TILE_SIZE, waypoints[wpCtr][1]*TILE_SIZE);
				//				if(foundBlock){
				//					Lab4.noisemaker.systemSound(0);
				//					try{
				//						Thread.sleep(1500);
				//					}catch (Exception e){}
				//					wpCtr--;
				//					foundBlock = false;
				//				}
				wpCtr++;
			}
		}else if(buttonChoice == Button.ID_DOWN){
			//ANOTHER TEST OPTION
			/*
			 * -Assumes bot starts at 0,0,0
			 * - Goes to LL, turns to second waypoint and crawls forward until within certain range of next waypoint
			 * 
			 */
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread searchThread = new Thread(search);
			path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);
			deltaTheta = path_angle-currentTheta;
			nav.turnTo(deltaTheta);
			nav.travelTo(waypoints[wpCtr][0]*TILE_SIZE, waypoints[wpCtr][1]*TILE_SIZE);
			path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0]-(Math.round(currentX/TILE_SIZE)), waypoints[wpCtr][1]-(Math.round(currentY/TILE_SIZE)));
			path_angle = (180/Math.PI)*Math.atan2(waypoints[wpCtr][0], waypoints[wpCtr][1]);
			deltaTheta = path_angle-currentTheta;
			nav.turnTo(deltaTheta);
			try{
				Thread.sleep(5000);
			}catch (Exception e){}
			
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			leftMotor.forward();
			rightMotor.forward();
			wpCtr++;
			searchThread.start();
			
			while(true){
				xyt = odometer.getXYT();
				while(!foundBlock && !(xyt[0]/waypoints[wpCtr][0]*TILE_SIZE <= 1.2 && xyt[0]/waypoints[wpCtr][0]*TILE_SIZE >= 0.8) && !(xyt[1]/waypoints[wpCtr][1]*TILE_SIZE <= 1.2 && xyt[1]/waypoints[wpCtr][1]*TILE_SIZE >= 0.8)){
					xyt = odometer.getXYT();
				}
				if((xyt[0]/waypoints[wpCtr][0]*TILE_SIZE <= 1.2 && xyt[0]/waypoints[wpCtr][0]*TILE_SIZE >= 0.8) && (xyt[1]/waypoints[wpCtr][1]*TILE_SIZE <= 1.2 && xyt[1]/waypoints[wpCtr][1]*TILE_SIZE >= 0.8)){
					wpCtr++;
					nav.turnTo(-90);
				}else{
					try{
						Thread.sleep(5000);
					}catch (Exception e){}
					leftMotor.forward();
					rightMotor.forward();
				}
				noisemaker.systemSound(1);
				foundBlock = false;
			}
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
