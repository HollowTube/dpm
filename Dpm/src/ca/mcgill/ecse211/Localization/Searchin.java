package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.lab5.MotorControl;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class Searchin {
	private static double xyt[] = new double[3];
	private MotorControl motorcontrol;
	private Nav nav;
	private static Odometer odometer;
	private double deltaTheta;
	private double currentX;
	private double currentY;
	private double currentTheta;
	public double[][] waypoints= { {-0.5, 2.5}, {2.5, 2.5}, {2.5, -0.5}, {0, -0.5}, {0, 2}, {2, 2}, {2, 0}, {0.5, 0}, {0.5, 1.5}, {1.5, 1.5}, {1.5, 0.5}, {1, 0.5}, {1,1}};
	public int wpCtr = 0;
	private double TILE_SIZE = 30.48;
	private double path_angle;

	public Searchin(Odometer odometer, Nav nav) throws OdometerExceptions{
		this.odometer = Odometer.getOdometer();
		this.motorcontrol = MotorControl.getMotor();
		this.nav = nav;
	}
	/**
	 * Travels waypoints in global variable waypoints[] creating a spiral-like pattern. Always turns right
	 * within the set search area.
	 * 
	 * */
	public void zamboni(){ //could have param of wpCtr for obstacle avoidance purposes
		/*
		 * Brute force approach:
		 * Zamboni movement, i.e slight overlap with previous movements.
		 * - Start in center of bottom left corner
		 * - Navigate nearly full loop in outermost layer in center of block constant polling light sensor
		 * - Stop at line of x=0, and */
		while(wpCtr < waypoints[0].length){
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
			
			//POTENTIAL OBSTACLE AVOIDANCE
			//while(odo.X != waypoint(x) && odo.X != waypoint(y)){}
			//if the x and y aren't at the desired coordinate, wait to update the wpCtr
			
			wpCtr++;
		}
	}
	
	
	/**
	 * Avoid osbstacle by moving right, frorward, left, forward.
	 * 
	 */
	public void avoid(){
		motorcontrol.dime_turn(90);
		motorcontrol.leftRot(8, true);
		motorcontrol.rightRot(8,false);
		motorcontrol.dime_turn(-90);
		motorcontrol.leftRot(8, true);
		motorcontrol.rightRot(8,false);
		//turn right
		//move straight
		//turn left
		//move straight
		//move to waypoint
	}
}
