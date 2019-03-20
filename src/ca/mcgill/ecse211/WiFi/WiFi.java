package ca.mcgill.ecse211.WiFi;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.entryPoint.Lab5;
import lejos.hardware.Button;

/**
 * Example class using WifiConnection to communicate with a server and receive data concerning the
 * competition such as the starting corner the robot is placed in.
 *
 * Keep in mind that this class is an **example** of how to use the WiFi code; you must use the
 * WifiConnection class yourself in your own code as appropriate. In this example, we simply show
 * how to get and process different types of data.
 *
 * There are two variables you **MUST** set manually before trying to use this code.
 *
 * 1. SERVER_IP: The IP address of the computer running the server application. This will be your
 * own laptop, until the beta beta demo or competition where this is the TA or professor's laptop.
 * In that case, set the IP to 192.168.2.3.
 *
 * 2. TEAM_NUMBER: your project team number
 *
 * Note: We System.out.println() instead of LCD printing so that full debug output (e.g. the very
 * long string containing the transmission) can be read on the screen OR a remote console such as
 * the EV3Control program via Bluetooth or WiFi. You can disable printing from the WiFi code via
 * ENABLE_DEBUG_WIFI_PRINT (below).
 *
 * @author Michael Smith, Tharsan Ponnampalam
 *
 */
public class WiFi {

  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.8";
  private static final int TEAM_NUMBER = 2;
  
  private static Map data;
  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

  @SuppressWarnings("rawtypes")
  public static void getData() {

    System.out.println("Running..");

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
       data = conn.getData();
       
       // Team numbers
       Lab5.RedTeam = ((Long) data.get("RedTeam")).intValue();
       Lab5.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
       
       // Corners
       Lab5.greenCorner= ((Long) data.get("GreenCorner")).intValue();
       
       // Green Zone
       Lab5.Green_UR_x= ((Long) data.get("Green_UR_x")).intValue();
       Lab5.Green_LL_x= ((Long) data.get("Green_LL_x")).intValue();
       Lab5.Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();
       Lab5.Green_LL_y= ((Long) data.get("Green_LL_y")).intValue();
       
       /*Lab5.Island_UR_x = ((Long) data.get("Island_UR_x")).intValue();
       Lab5.Island_LL_x= ((Long) data.get("Island_LL_x")).intValue();
       Lab5.Island_UR_y = ((Long) data.get("Island_UR_y")).intValue();
       Lab5.Island_LL_y = ((Long) data.get("Island_LL_y")).intValue();*/
       
       // Green's Tunnel
       Lab5.TNG_LL_x = ((Long) data.get("TNG_LL_x")).intValue();
       Lab5.TNG_UR_x = ((Long) data.get("TNG_UR_x")).intValue();
       Lab5.TNG_LL_y= ((Long) data.get("TNG_LL_y")).intValue();
       Lab5.TNG_UR_y= ((Long) data.get("TNG_UR_y")).intValue();
       
       // Search Area Red Team
       
       // Upper-right hand corner
       Lab5.SZR_UR_x = ((Long) data.get("SZR_UR_x")).intValue();
       Lab5.SZR_UR_y = ((Long) data.get("SZR_UR_y")).intValue(); 
       
       // Lower-left hand corner
       Lab5.SZR_LL_x = ((Long) data.get("SZR_LL_x")).intValue();
       Lab5.SZR_LL_y = ((Long) data.get("SZR_LL_y")).intValue(); 
       
       // Search Area Green Team

       // Upper-right hand corner
       Lab5.SZG_UR_x = ((Long) data.get("SZG_UR_x")).intValue();
       Lab5.SZG_UR_y = ((Long) data.get("SZG_UR_y")).intValue();
       
       // Lower-left hand corner
       Lab5.SZG_LL_x = ((Long) data.get("SZG_LL_x")).intValue();
       Lab5.SZG_LL_y = ((Long) data.get("SZG_LL_y")).intValue();
       
		if(Lab5.RedTeam == TEAM_NUMBER)
		{
			assignTeamColor(0);
		}
		else if(Lab5.GreenTeam == TEAM_NUMBER)
		{
			assignTeamColor(1);
		}
		else
		{
			System.out.println("Team number error");
		}

	} catch (Exception e) {
		System.err.println("Error: " + e.getMessage());
	}
	System.out.println("Complete!");   
  }
	
	/**
	 * sets the parameters to the correct values depending on team color
	 * 
	 * @param color 0 if red team, 1 if green team
	 */
	private static void assignTeamColor(int color) {
		if(color == 0) {  // RED		
			
			setStartingCoordinates(Lab5.RedTeam);
			
		    Lab5.redCorner= ((Long) data.get("RedCorner")).intValue();
		    // Search area
	       Lab5.Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
	       Lab5.Red_LLx = ((Long) data.get("Red_LL_x")).intValue();
	       Lab5.Red_UR_y= ((Long) data.get("Red_UR_y")).intValue();
	       Lab5.Red_LL_y= ((Long) data.get("Red_LL_y")).intValue();
	       
	       // Tunnel
	       Lab5.TNR_LL_x = ((Long) data.get("TNR_LL_x")).intValue();
	       Lab5.TNR_UR_x = ((Long) data.get("TNR_UR_x")).intValue();
	       Lab5.TNR_LL_y = ((Long) data.get("TNR_LL_y")).intValue();
	       Lab5.TNR_UR_y = ((Long) data.get("TNR_UR_y")).intValue();
	       
	       // Search Area
	       
	       // Upper-right hand corner
	       Lab5.SZR_UR_x = ((Long) data.get("SZR_UR_x")).intValue();
	       Lab5.SZR_UR_y = ((Long) data.get("SZR_UR_y")).intValue(); 
	       
	       // Lower-left hand corner
	       Lab5.SZR_LL_x = ((Long) data.get("SZR_LL_x")).intValue();
	       Lab5.SZR_LL_y = ((Long) data.get("SZR_LL_y")).intValue(); 
	       
		}
		else if(color == 1) //green
		{
			corner = G;
			LL_x = Green_LL_x;
			LL_y = Green_LL_y;
			UR_x = Green_UR_x;
			UR_y = Green_UR_y;
			TunLL_x = BRG_LL_x;
			TunLL_y = BRG_LL_y;
			TunUR_x = BRG_UR_x;
			TunUR_y = BRG_UR_y;
			Tr_x = TG_x;
			Tr_y = TG_y;
			setStartingCoordinates(G);
			
			opp_corner = R;
			opp_LL_x = Red_LL_x;
			opp_LL_y = Red_LL_y;
			opp_UR_x = Red_UR_x;
			opp_UR_y = Red_UR_y;
			opp_TunLL_x = BRR_LL_x;
			opp_TunLL_y = BRR_LL_y;
			opp_TunUR_x = BRR_UR_x;
			opp_TunUR_y = BRR_UR_y;
			opp_Tr_x = TR_x;
			opp_Tr_y = TR_y;
		}
		else
		{
			System.out.println("Wrong color input for parameter assignment");
		}
	}

	/**
	 * finds the first set of coordinates that the robot will localize to depending on the starting zone and 
	 * the size of the game grid
	 * 
	 * @param zone 0, 1, 2, or 3
	 */
	private static void setStartingCoordinates(int zone)
	{
		if(zone == 0)
		{
			localizeX = 1;
			localizeY = 1;
		}
		else if(zone == 1)
		{
			localizeX = RingChallenge.GAME_GRID_X - 1;
			localizeY = 1;	
		}
		else if(zone == 2)
		{
			localizeX = RingChallenge.GAME_GRID_X - 1;
			localizeY = RingChallenge.GAME_GRID_Y - 1;	
		}
		else if(zone == 3)
		{
			localizeX = 1;
			localizeY = RingChallenge.GAME_GRID_Y - 1;	
		}
		else
		{
			System.out.println("Zone number error");
		}
	}
	
    // Wait until user decides to end program
    Button.waitForAnyPress();
  }

