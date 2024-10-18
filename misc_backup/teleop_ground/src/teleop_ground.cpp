#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include <rclcpp/rclcpp.hpp>

// map for movement keys
// no change along z-axis
// NED frame, where x-axis oriented in direction of tracking camera
std::map<char, std::vector<float>> moveBindings 
{
	{'w',{1}},
	{'a',{4}},
	{'s',{2}},
	{'d',{3}}
};

const char* msg_bootup = R"(
Reading from the keyboard and publishing to ground locomotion!
----
Movement:
  w
a   d
  s
----
w: move along +x-axis
s: move along -x-axis
d: turn right (positive rotation about z-axis)
a: turn left (negative rotation about z-axis)
----
To stop motion, press 'x'.
----
)";

// initialize variables
char key = ' ';
__uint16_t TX = 0;
char TX_out[4];

// serial information
const char* port = "/dev/ttyHS1";
int teensy = open(port, O_RDWR);

class TeleopGround : public rclcpp::Node
{
	public:
		TeleopGround() : Node("teleop_ground")
		{
			
			printf("%s",msg_bootup);
			printf("Opening serial port %s.\n",port);
			usb_port_0();
			printf("----\n");
			control_loop();
		}
	private:
		void control_loop();
		int getch();
		void update_position(char key);
		void usb_port_0(void);
};

// retrieving keyboard inputs
// SOURCE: https://github.com/aarsht7/teleop_cpp_ros2/blob/main/src/teleop_cpp_ros2.cpp
int TeleopGround::getch(void)
{
	int ch;
	struct termios oldt;
	struct termios newt;

	// Store old settings, and copy to new settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	// Make required changes and apply the settings
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_iflag |= IGNBRK;
	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	newt.c_cc[VMIN] = 1;
	newt.c_cc[VTIME] = 0;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// Get the current character
	ch = getchar();

	// Reapply old settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

// convert keyboard input to new value for Servos
// KEY:
// 0 = stop vehicle
// 1 = move along +x-axis
// 2 = move along -x-axis
// 3 = turn right
// 4 = turn left
void TeleopGround::update_position(char key)
{
	if (moveBindings.count(key) == 1)
	{
		TX = moveBindings[key][0];
		if (key == 'w')
		{
			printf("\nVehicle moving forward.\n");
		}
		else if (key == 's')
		{
			printf("\nVehicle moving backward.\n");
		}
		else if (key == 'd')
		{
			printf("\nVehicle turning right.\n");
		}
		else if (key == 'a')
		{
			printf("\nVehicle turning left.\n");
		}
	}
	else if (key=='x') 
	{
		TX = 0;
		printf("\nVehicle stopped.\n");
	}
	else if (key == '\x03')
	{
		TX = 0;
		printf("\nNode terminated.\n");
		rclcpp::shutdown();
	}
	else 
	{
		TX = 0;
		printf("\nInvalid command. Stopping vehicle.\n");
	}
	
	// send to Teensy	
	TX_out[0] = char(14); // initial value of data packet ALWAYS 14
	TX_out[1] = char((TX >> 8) & 0xFF);
	TX_out[2] = char(TX & 0xFF);
	TX_out[3] = char(22); // final value of data packet ALWAYS 22
	write(teensy, &TX_out, sizeof(TX_out));
}

// opening USB port to Teensy
void TeleopGround::usb_port_0(void)
{
	if (teensy < 0) 
	{
	    printf("Error %i from open: %s\n", errno, strerror(errno)); //check for errors
	}
	else 
	{
		printf("Serial port %s opened successfully!\n",port);
	}

	struct termios tty; //create termios structure named tty
	memset(&tty, 0, sizeof tty); 

	if(tcgetattr(teensy, &tty) != 0)  //read in existing settings and handle any error
	{
	    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB; //Disable parity bit
	tty.c_cflag &= ~CSTOPB; //One stop bit
	tty.c_cflag |= CS8;	//8 bits per byte
	tty.c_cflag &= ~CRTSCTS; //Disable flow control
	tty.c_cflag |= CREAD | CLOCAL; //Turn on Read & ignore ctrl lines
	tty.c_lflag &= ~ICANON;	//disable canonical mode
	tty.c_lflag &= ~ECHO; //Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL;	// Disable new-line echo
	tty.c_lflag &= ~ISIG; //Disable signals
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);	// Turn off software flow control
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	tty.c_cc[VTIME] = 1;	//VMIN = 0, VTIME = 0:	No blocking, return immediately with what is available
	tty.c_cc[VMIN] = 0;		//VMIN > 0, VTIME = 0:	read() will always wait for bytes (as deterined by VMIN); could block indefinitely
							//VMIN = 0, VTIME > 0;	read() will block until either any amount of data is available, or timeout occurs (given by VTIME)
							//VMIN > 0, VTIME > 0;  block until either VMIN characters have been recieved, or VTIME after first character has elapsed
							//VMIN + VTIME is capped at 255. VTIME is in deciseconds, so 255 = 25.5 seconds

	cfsetispeed(&tty, B57600); //set in baud rate to 115200
	cfsetospeed(&tty, B57600); //set out baud rate to 115200

	if (tcsetattr(teensy, TCSANOW, &tty) != 0) //change and save attributes
		{
		    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		}
}

// main control loop
void TeleopGround::control_loop(void)
{
	while (rclcpp::ok()) 
	{
		// obtain character key pressed
		key = getch();
		printf("\nCharacter pressed: %c",key);
		update_position(key);
	}
}

int main(int argc, char *argv[]) {	
	// node initialization
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TeleopGround>());
	return 0;
}
