package ca.mcgill.ecse211.main_package;

/**
 * This Class takes in the parameters for the game with wifi.
 * It allows the robot to know the layout of the playing field for the run.
 * It also supplies the key coordinates needed for the robot to the waypoint system for navigation.
 * @author Alexandre
 *
 */
//TODO the wifi inputs should be handled in this class

//import java.util.Map;
//import ca.mcgill.ecse211.WiFiClient.WifiConnection;

public class Parameter_intake {
	public int RedTeam; 		//Team starting from red zone
	public int GreenTeam = 13;   	//Team starting from green zone
	public int RedCorner;		//Starting corner for red team
	public int GreenCorner = 1;		//Starting corner for green team
	public int OG;				//color of green opponent flag
	public int OR;				//color of red opponent flag
	public int Red_LL_x = 0;		//x coordinate of lower left hand corner of Red Zone
	public int Red_LL_y = 5;		//y coordinate of lower left hand corner of Red Zone
	public int Red_UR_x = 8;		//x coordinate of upper right hand corner of Red Zone
	public int Red_UR_y = 8;		//y coordinate of upper right hand corner of Red Zone
	public int Green_LL_x = 0;		//x coordinate of lower left hand corner of Green Zone
	public int Green_LL_y = 0;		//y coordinate of lower left hand corner of Green Zone
	public int Green_UR_x = 8;		//x coordinate of upper right hand corner of Green Zone
	public int Green_UR_y = 3;		//y coordinate of upper right hand corner of Green Zone
	public int TN_LL_x = 2;			//x coordinate of lower left corner of the tunnel footprint
	public int TN_LL_y = 3;			//y coordinate of lower left corner of the tunnel footprint
	public int TN_UR_x = 3;			//x coordinate of upper right corner of the tunnel footprint
	public int TN_UR_y = 5;			//y coordinate of upper right corner of the tunnel footprint
	public int BR_LL_x = 5;			//x coordinate of lower left corner of the bridge footprint
	public int BR_LL_y = 3;			//y coordinate of lower left corner of the bridge footprint
	public int BR_UR_x = 6;			//x coordinate of upper right corner of the bridge footprint
	public int BR_UR_y = 5;			//y coordinate of upper right corner of the bridge footprint
	public int SR_LL_x;			//x coordinate of lower left corner of the search region in red player zone
	public int SR_LL_y;			//y coordinate of lower left corner of the search region in red player zone
	public int SR_UR_x;			//x coordinate of upper right corner of the search region in red player zone
	public int SR_UR_y;			//y coordinate of upper right corner of the search region in red player zone
	public int SG_LL_x;			//x coordinate of lower left corner of the search region in green player zone
	public int SG_LL_y;			//y coordinate of lower left corner of the search region in green player zone
	public int SG_UR_x;			//x coordinate of upper right corner of the search region in green player zone
	public int SG_UR_y;			//y coordinate of upper right corner of the search region in green player zone

	// ** Set these as appropriate for your team and current situation **
//	private static final String SERVER_IP = "192.168.2.3"; 
//	private static final int TEAM_NUMBER = 13;
//	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false; // Enable/disable printing of debug info from the WiFi class
//	
	private static Parameter_intake parameters=null;

	/**
	 * Class constructor
	 * @author Alexandre Coulombe
	 */
	public Parameter_intake(){
		
	}

	/**
	 * Method to receive parameters of the play field from the game controller
	 * @author Alexandre Coulombe
	 * @param RedTeam
	 * @param GreenTeam
	 * @param RedCorner
	 * @param GreenCorner
	 * @param OG
	 * @param OR
	 * @param Red_LL_x
	 * @param Red_LL_y
	 * @param Red_UR_x
	 * @param Red_UR_y
	 * @param Green_LL_x
	 * @param Green_LL_y
	 * @param Green_UR_x
	 * @param Green_UR_y
	 * @param TN_LL_x
	 * @param TN_LL_y
	 * @param TN_UR_x
	 * @param TN_UR_y
	 * @param BR_LL_x
	 * @param BR_LL_y
	 * @param BR_UR_x
	 * @param BR_UR_y
	 * @param SR_LL_x
	 * @param SR_LL_y
	 * @param SR_UR_x
	 * @param SR_UR_y
	 * @param SG_LL_x
	 * @param SG_LL_y
	 * @param SG_UR_x
	 * @param SG_UR_y
	 */
//	public Parameter_intake wifiIntake(){
//		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
//
//		try{
//			Map data = conn.getData();
//
//			this.RedTeam = ((Long) data.get("RedTeam")).intValue();
//			this.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
//			this.RedCorner = ((Long) data.get("RedCorner")).intValue();
//			this.GreenCorner = ((Long) data.get("GreenCorner")).intValue();
//			this.OG = ((Long) data.get("OG")).intValue();
//			this.OR = ((Long) data.get("OR")).intValue();
//			this.Red_LL_x = ((Long) data.get("Red_LL_x")).intValue();
//			this.Red_LL_y = ((Long) data.get("Red_LL_y")).intValue();
//			this.Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
//			this.Red_UR_y = ((Long) data.get("Red_UR_y")).intValue();
//			this.Green_LL_x = ((Long) data.get("Green_LL_x")).intValue();
//			this.Green_LL_y = ((Long) data.get("Green_LL_y")).intValue();
//			this.Green_UR_x = ((Long) data.get("Green_UR_x")).intValue();
//			this.Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();
//			this.TN_LL_x = ((Long) data.get("TN_LL_x")).intValue();
//			this.TN_LL_y = ((Long) data.get("TN_LL_y")).intValue();
//			this.TN_UR_x = ((Long) data.get("TN_UR_x")).intValue();
//			this.TN_UR_y = ((Long) data.get("TN_UR_y")).intValue();
//			this.BR_LL_x = ((Long) data.get("BR_LL_x")).intValue();
//			this.BR_LL_y = ((Long) data.get("BR_LL_y")).intValue();
//			this.BR_UR_x = ((Long) data.get("BR_UR_x")).intValue();
//			this.BR_UR_y = ((Long) data.get("BR_UR_y")).intValue();
//			this.SR_LL_x = ((Long) data.get("SR_LL_x")).intValue();
//			this.SR_LL_y = ((Long) data.get("SR_LL_y")).intValue();			
//			this.SR_UR_x = ((Long) data.get("SR_UR_y")).intValue();			
//			this.SR_UR_y = ((Long) data.get("SR_UR_x")).intValue();			
//			this.SG_LL_x = ((Long) data.get("SG_LL_x")).intValue();			
//			this.SG_LL_y = ((Long) data.get("SG_LL_y")).intValue();			
//			this.SG_UR_x = ((Long) data.get("SG_UR_x")).intValue();			
//			this.SG_UR_y = ((Long) data.get("SG_UR_y")).intValue();
//		}catch (Exception e){
//
//		}
//		return parameters;
//	}
	/**
	 * Method to receive parameters of the play field from the game controller
	 * @author Alexandre Coulombe
	 * @param RedTeam
	 * @param GreenTeam
	 * @param RedCorner
	 * @param GreenCorner
	 * @param OG
	 * @param OR
	 * @param Red_LL_x
	 * @param Red_LL_y
	 * @param Red_UR_x
	 * @param Red_UR_y
	 * @param Green_LL_x
	 * @param Green_LL_y
	 * @param Green_UR_x
	 * @param Green_UR_y
	 * @param TN_LL_x
	 * @param TN_LL_y
	 * @param TN_UR_x
	 * @param TN_UR_y
	 * @param BR_LL_x
	 * @param BR_LL_y
	 * @param BR_UR_x
	 * @param BR_UR_y
	 * @param SR_LL_x
	 * @param SR_LL_y
	 * @param SR_UR_x
	 * @param SR_UR_y
	 * @param SG_LL_x
	 * @param SG_LL_y
	 * @param SG_UR_x
	 * @param SG_UR_y
	 */
	public void giveParameters(int RedTeam, int GreenTeam, int RedCorner, int GreenCorner, int OG, int OR, int Red_LL_x, int Red_LL_y,
			int Red_UR_x, int Red_UR_y, int Green_LL_x, int Green_LL_y, int Green_UR_x, int Green_UR_y, int TN_LL_x,
			int TN_LL_y, int TN_UR_x, int TN_UR_y, int BR_LL_x, int BR_LL_y, int BR_UR_x, int BR_UR_y, int SR_LL_x, 
			int SR_LL_y, int SR_UR_x, int SR_UR_y, int SG_LL_x, int SG_LL_y, int SG_UR_x, int SG_UR_y){
		this.RedTeam=RedTeam;
		this.GreenTeam=GreenTeam;
		this.RedCorner=RedCorner;
		this.GreenCorner=GreenCorner;
		this.OG=OG;
		this.OR=OR;
		this.Red_LL_x=Red_LL_x;
		this.Red_LL_y=Red_LL_y;
		this.Red_UR_x=Red_UR_x;
		this.Red_UR_y=Red_UR_y;
		this.Green_LL_x=Green_LL_x;
		this.Green_LL_y=Green_LL_y;
		this.Green_UR_x=Green_UR_x;
		this.Green_UR_y=Green_UR_y;
		this.TN_LL_x=TN_LL_x;
		this.TN_LL_y=TN_LL_y;
		this.TN_UR_x=TN_UR_x;
		this.TN_UR_y=TN_UR_y;
		this.BR_LL_x=BR_LL_x;
		this.BR_LL_y=BR_LL_y;
		this.BR_UR_x=BR_UR_x;
		this.BR_UR_y=BR_UR_y;
		this.SR_LL_x=SR_LL_x;
		this.SR_LL_y=SR_LL_y;
		this.SR_UR_x=SR_UR_x;
		this.SG_UR_y=SG_UR_y;
		this.SG_LL_x=SG_LL_x;
		this.SG_LL_y=SG_LL_y;
		this.SG_UR_x=SG_UR_x;
		this.SG_UR_y=SG_UR_y;
	}
	/**
	 * Method to synchronize with the Parameter_intake use in all classes
	 * @author Alexandre Coulombe
	 * @return the parameter_intake object to be used by all classes
	 */
	public synchronized static Parameter_intake getParameter(){
		if (parameters != null) { // Return existing object
			return parameters;
		} else { // create object and return it
			parameters = new Parameter_intake();
			return parameters;
		}
	}
	/**
	 * Method to return the color of the target block.
	 * The method will check which team the robot belongs to and
	 * return the color associated by the integer value of the variable
	 * representing the color of the target block of the team.
	 * 
	 * @author Alexandre Coulombe
	 * @return color of target block
	 */
	public String Target_color(){
		if(GreenTeam==13){
			switch (OG){
			case 1:
				return "red";
			case 2:
				return "blue";
			case 3:
				return "yellow";
			case 4:
				return "white";
			case 5:
				return "";
			}
		}
		else if(RedTeam==13){
			switch (OR){
			case 1:
				return "red";
			case 2:
				return "blue";
			case 3:
				return "yellow";
			case 4:
				return "white";
			case 5:
				return "";
			}
		}
		return "";
	}
	/**
	 * Method to return the x coordinate of the starting corner of the robot
	 * Will return -1 if a fault occurs for green team
	 * @author Alexandre Coulombe
	 * @return x coordinate of starting corner
	 */
	public int Green_start_coord_x(){
			switch (GreenCorner){
			case 1:
				return 1;
			case 2:
				return 7;
			case 3:
				return 7;
			case 4:
				return 1;
			}
			return -1;
		}
	/**
	 * Method to return the x coordinate of the starting corner of the robot
	 * Will return -1 if a fault occurs for red team
	 * @author Alexandre Coulombe
	 * @return x coordinate of starting corner
	 */
	public int Red_start_coord_x(){
			switch (RedCorner){
			case 1:
				return 1;
			case 2:
				return 7;
			case 3:
				return 7;
			case 4:
				return 1;
			}
		return -1;
	}
	/**
	 * Method to return the x coordinate of the starting corner of the robot
	 * Will return -1 if a fault occurs for green team
	 * @author Alexandre Coulombe
	 * @return y coordinate of starting corner
	 */
	public int Green_start_coord_y(){
			switch (GreenCorner){
			case 1:
				return 1;
			case 2:
				return 1;
			case 3:
				return 7;
			case 4:
				return 7;
			}
			return -1;
		}
	/**
	 * Method to return the x coordinate of the starting corner of the robot
	 * Will return -1 if a fault occurs for red team
	 * @author Alexandre Coulombe
	 * @return y coordinate of starting corner
	 */
	public int Red_start_coord_y(){
			switch (RedCorner){
			case 1:
				return 1;
			case 2:
				return 1;
			case 3:
				return 7;
			case 4:
				return 7;
			}
		return -1;
	}
	/**
	 * Method to return the heading of the robot after it localizes based
	 * on the information of the starting corner for Green team
	 * returns -1 is a fault occurs
	 * @author Alexandre Coulombe
	 * @return heading after localization
	 */
	public int Green_start_heading(){
		switch (GreenCorner){
		case 1:
			return 90;
		case 2:
			return 0;
		case 3:
			return 270;
		case 4:
			return 180;
		}
		return -1;
	}
	/**
	 * Method to return the heading of the robot after it localizes based
	 * on the information of the starting corner for Red team
	 * returns -1 is a fault occurs
	 * @author Alexandre Coulombe
	 * @return heading after localization
	 */
	public int Red_start_heading(){
		switch (RedCorner){
			case 1:
				return 90;
			case 2:
				return 0;
			case 3:
				return 270;
			case 4:
				return 180;
		}
		return -1;
	}
	/**
	 * Method to get the x coordinate needed to place the robot before starting the tunnel crossing
	 * The robot will be placed to the right of the tunnel entrance before it traverses the tunnel.
	 * The method uses the alignment of the tunnel with the edge of the green area to determine the 
	 * location of the tunnel entrance. It is then used to determine the coordinates for the robot.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @return x coordinate to go to before crossing the tunnel
	 */
	public int TN_coord_x(){
		if(Green_UR_y==TN_LL_y){
			return (TN_LL_x+1);
		}
		else if(Green_LL_y==TN_UR_y){
			return (TN_UR_x-1);
		}
		else if(Green_LL_x==TN_UR_x){
			return (TN_UR_x+1);
		}
		else if(Green_UR_x==TN_LL_x){
			return (TN_LL_x-1);
		}
		return -1;
	}
	/**
	 * Method to get the y coordinate needed to place the robot before starting the tunnel crossing
	 * The robot will be placed to the right of the tunnel entrance before it traverses the tunnel.
	 * The method uses the alignment of the tunnel with the edge of the green area to determine the 
	 * location of the tunnel entrance. It is then used to determine the coordinates for the robot.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @return y coordinate to go to before crossing the tunnel
	 */
	public int TN_coord_y(){
		if(Green_LL_y==TN_UR_y){
			return (TN_UR_y+1);  		//if Green zone is "above" tunnel area
		}
		else if(Green_UR_y==TN_LL_y){	//if Green zone is "below" tunnel area
			return (TN_LL_y-1);		
		}
		else if(Green_UR_x==TN_LL_x){	//if Green zone is "left" to tunnel area
			return TN_LL_y;
		}
		else if(Green_LL_x==TN_UR_x){	//if Green zone is "right" to tunnel area
			return TN_UR_y;
		}
		return -1;
	}
	/**
	 * Method to get the expected x coordinate of the robot after crossing the tunnels
	 * The robot will localize to the front right coordinate after the tunnel exit.
	 * The method uses the coordinates of the robot before the tunnel traversal and the direction
	 * of the tunnel based on the longest side. It is then used to determine the coordinates the robot
	 * will have at the end of the traversal.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @param TN_coord_x
	 * @return expected x coordinate after crossing the tunnel
	 */
	public int TN_end_x(int TN_coord_x){
		int TN_length=TN_length();
		if(TN_coord_x==TN_UR_x || TN_coord_x==TN_LL_x){	//if the robot is moving up or down the map
			return TN_coord_x;
		}
		else if(TN_coord_x<TN_LL_x){					//robot is moving to the right
			return (TN_coord_x+2+TN_length);
		}
		else if(TN_coord_x>TN_UR_x){					//robot is moving to the left
			return (TN_coord_x-2-TN_length);
		}
		return -1;
	}
	/**
	 * Method to get the expected y coordinate of the robot after crossing the tunnel
	 * The robot will localize to the front right coordinate after the tunnel exit.
	 * The method uses the coordinates of the robot before the tunnel traversal and the direction
	 * of the tunnel based on the longest side. It is then used to determine the coordinates the robot
	 * will have at the end of the traversal.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @param TN_coord_y
	 * @return expected y coordinate after crossing the tunnel
	 */
	public int TN_end_y(int TN_coord_y){
		int TN_length=TN_length();
		if(TN_coord_y==TN_UR_y){						//if the robot is moving left or right the map
			return (TN_coord_y);
		}
		else if(TN_coord_y==TN_LL_y){
			return (TN_coord_y);
		}
		else if(TN_coord_y<TN_LL_y){					//robot is moving to the up
			return (TN_coord_y+2+TN_length);
		}
		else if(TN_coord_y>TN_UR_y){					//robot is moving to the down
			return (TN_coord_y-2-TN_length);
		}
		return -1;
	}
	/**
	 * Method to determine the length of the tunnel
	 * It uses the coordinates mapping the tunnel and does a difference,
	 * it takes the longest of the two directions 
	 * @author Alexandre Coulombe
	 * @return tunnel length
	 */
	public int TN_length(){
		int TN_length=TN_UR_x-TN_LL_x;
		if(TN_length==1){								//get the length of the tunnel, if the first equation gives 1 it may not be the long side
			TN_length=TN_UR_y-TN_LL_y;
		}
		return TN_length;
	}
	/**
	 * Method to get the y coordinate needed to place the robot before starting the bridge crossing
	 * The robot will be placed to the right of the bridge entrance before it traverses the bridge.
	 * The method uses the alignment of the bridge with the edge of the red area to determine the 
	 * location of the bridge entrance. It is then used to determine the coordinates for the robot.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @return x coordinate to go to before crossing the bridge
	 */
	public int BR_coord_x(){
		if(Red_UR_y==BR_LL_y){
			return (BR_LL_x+1);
		}
		else if(Red_LL_y==BR_UR_y){
			return (BR_UR_x-1);
		}
		else if(Red_LL_x==BR_UR_x){
			return (BR_UR_x+1);
		}
		else if(Red_UR_x==BR_LL_x){
			return (BR_LL_x-1);
		}
		return -1;
	}
	/**
	 * Method to get the y coordinate needed to place the robot before starting the bridge crossing
	 * The robot will be placed to the right of the bridge entrance before it traverses the bridge.
	 * The method uses the alignment of the bridge with the edge of the red area to determine the 
	 * location of the bridge entrance. It is then used to determine the coordinates for the robot.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @return y coordinate to go to before crossing the bridge
	 */
	public int BR_coord_y(){
		if(Red_LL_y==BR_UR_y){
			return (BR_UR_y+1);  		//if Green zone is "above" tunnel area
		}
		else if(Red_UR_y==BR_LL_y){	//if Green zone is "below" tunnel area
			return (BR_LL_y-1);		
		}
		else if(Red_UR_x==BR_LL_x){	//if Green zone is "left" to tunnel area
			return BR_LL_y;
		}
		else if(Red_LL_x==BR_UR_x){	//if Green zone is "right" to tunnel area
			return BR_UR_y;
		}
		return -1;
	}
	/**
	 * Method to get the expected x coordinate of the robot after crossing the bridge
	 * The robot will localize to the front right coordinate after the bridge exit.
	 * The method uses the coordinates of the robot before the bridge traversal and the direction
	 * of the bridge based on the longest side. It is then used to determine the coordinates the robot
	 * will have at the end of the traversal.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @param BR_coord_x
	 * @return expected x coordinate after crossing the bridge
	 */
	public int BR_end_x(int BR_coord_x){
		int BR_length=BR_length();
		if(BR_coord_x==BR_UR_x || BR_coord_x==BR_LL_x){	//if the robot is moving up or down the map
			return BR_coord_x;
		}
		else if(BR_coord_x<BR_LL_x){					//robot is moving to the right
			return (BR_coord_x+2+BR_length);
		}
		else if(BR_coord_x>BR_UR_x){					//robot is moving to the left
			return (BR_coord_x-2-BR_length);
		}
		return -1;
	}
	/**
	 * Method to get the expected y coordinate of the robot after crossing the bridge
	 * The robot will localize to the front right coordinate after the bridge exit.
	 * The method uses the coordinates of the robot before the bridge traversal and the direction
	 * of the bridge based on the longest side. It is then used to determine the coordinates the robot
	 * will have at the end of the traversal.
	 * Will return -1 if a fault occurs
	 * @author Alexandre Coulombe
	 * @param BR_coord_y
	 * @return expected y coordinate after crossing the bridge
	 */
	public int BR_end_y(int BR_coord_y){
		int BR_length=BR_length();
		if(BR_coord_y==BR_UR_y){	//if the robot is moving left or right the map
			return (BR_coord_y);
		}
		else if(BR_coord_y==BR_LL_y){
			return (BR_coord_y);
		}
		else if(BR_coord_y<BR_LL_y){					//robot is moving to the up
			return (BR_coord_y+2+BR_length);
		}
		else if(BR_coord_y>BR_UR_y){					//robot is moving to the down
			return (BR_coord_y-2-BR_length);
		}
		return -1;
	}
	/**
	 * Method to determine the length of the bridge
	 * It uses the coordinates mapping the bridge and does a difference,
	 * it takes the longest of the two directions 
	 * @author Alexandre Coulombe
	 * @return bridge length
	 */
	public int BR_length(){
		int BR_length=BR_UR_x-BR_LL_x;
		if(BR_length==1){								//get the length of the bridge, if the first equation gives 1 it may not be the long side
			BR_length=BR_UR_y-BR_LL_y;
		}
		return BR_length;
	}
}

