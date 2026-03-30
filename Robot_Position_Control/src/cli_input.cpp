#include "cli_input.h"

float orientation_reference = 0;
float right_distance_reference = 0;
float left_distance_reference = 0;
float angular_velocity_reference = 0;
float linear_velocity_reference = 0;

bool manual_commands = false;
bool automatic_commands = true;
bool orientation_command = false;
bool forward_command = false; 
bool backward_command = false; 
bool rightMotorCommand = false;
bool leftMotorCommand  = false;
bool print_command = false; 

volatile bool right_motor_enabled = false;
volatile bool left_motor_enabled = false;

DataFromPi commands;

void command_handler(String command) {
    command.trim();  
	float val = 0;

	if (command == "MANUAL") {
		Serial.println("\033[2J");
		Serial.println("\033[H"); 
		Serial.println("Manual Mode");
		motors_control_state = MANUAL;
		right_motor_enabled = false;
		left_motor_enabled = false;
		automatic_commands = false;
		manual_commands = true;
		return;
	}

	if (command == "AUTO"){
		Serial.println("\033[2J");
		Serial.println("\033[H"); 
		Serial.println("Automatic Mode");
		motors_control_state = AUTOMATIC;
		manual_commands = false;
		automatic_commands = true;
		return;
	}

	if (manual_commands){
		if (command == "help") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Available commands");
			Serial.println("  help        - Display the list of available commands");
			Serial.println("  left        - Rotate left (-90°)");
			Serial.println("  right       - Rotate right (+90°)");
			Serial.println("  turn        - Rotate by a specified azimuth");
			Serial.println("  forward     - Move forward for a specified distance");
			Serial.println("  rightMotor  - Start the right motor");
			Serial.println("  leftMotor   - Start the left motor");
			Serial.println("  stop        - Stop both motors");
			Serial.println("  status      - Display the current robot status");
			Serial.println("  print       - Print real-time robot status");
		}
		else if (command == "turn") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Turns the structure");
			orientation_reference = current_position.robot_orientation_deg;
			orientation_command = true;
		}
		else if (orientation_command){
			float delta_angle = command.toFloat();

			if (delta_angle > 180.0f)
				delta_angle -= 360.0f;
			else if (delta_angle < -180.0f)
				delta_angle += 360.0f;

			rotate(delta_angle);
			orientation_command = false;
		}
		else if (command == "left") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Turns left (-90°)");
			orientation_reference = orientation_reference - 90;
			rotate(orientation_reference);
		}
		else if (command == "right") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Turns right (90°)");
			orientation_reference = orientation_reference + 90;
			rotate(orientation_reference);
		}
		else if (command == "rightMotor") {
			rightMotorCommand = true;
			return;
		}
		else if (rightMotorCommand) {
			val = command.toFloat();
			right_distance_reference = val;
			right_motor_enabled = true;
			rightMotorCommand = false;
		}
		else if (command == "leftMotor") {
			leftMotorCommand = true;
			return;
		}
		else if (leftMotorCommand) {
			val = command.toFloat();
			left_distance_reference = val;
			left_motor_enabled = true;
			leftMotorCommand = false;
		}
		else if (command == "forward") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Both Motors started !");
			forward_command = true;
		} 
		else if (forward_command){
			val = command.toFloat();
			if (val != 0) {
				forward_command = false;
				move_forward(val);
			}
			else {
				Serial.println("\033[2J");
				Serial.println("\033[H"); 
				Serial.println("Motor stopped!");
				forward_command = false;
			}
		}	
		else if (command == "backward") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Both Motors started !");
			backward_command = true;
		} 
		else if (backward_command){
			val = command.toFloat();
			if (val != 0) {
				backward_command = false;
				move_backward(val);
			}
			else {
				Serial.println("\033[2J");
				Serial.println("\033[H"); 
				Serial.println("Motor stopped!");
				backward_command = false;
			}
		}
		else if (command == "stop") {
			left_motor_enabled = false;
			right_motor_enabled = false;
			left_motor_state = DONE;
			right_motor_state = DONE;
			left_encoder.initialized = false;
			right_encoder.initialized = false;
			orientation_command = false;
			forward_command = false;
			backward_command = false;
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Motor stopped!");
		} 
		else if (command == "status") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.printf("current position - x : %8.2f y : %8.2f theta : %8.2f ang_vel : %8.2f °/s lin_vel : %8.2f m/s",robot_state.x, robot_state.y, robot_state.theta, robot_state.current_angular_velocity, robot_state.current_linear_velocity);
		}
		else if (command == "print") {
			Serial.println("\033[2J");
			Serial.println("\033[H");
			print_command = !print_command;
		}
		else {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.print("❌ Unknown command");
			Serial.println(command);
			Serial.println("Tapez 'help' pour voir la liste des commandes.");
		}
	}
	else if (automatic_commands){
		if (command == "help") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.println("Available commands");
			Serial.println("  help        - Display the list of available commands");
			Serial.println("  status      - Display the current robot status");
			Serial.println("  print       - Print real-time robot status");
		}
		else if (command == "status") {
			Serial.println("\033[2J");
			Serial.println("\033[H"); 
			Serial.printf("current position - x : %8.2f y : %8.2f theta : %8.2f ang_vel : %8.2f °/s lin_vel : %8.2f m/s",robot_state.x, robot_state.y, robot_state.theta, robot_state.current_angular_velocity, robot_state.current_linear_velocity);
		}
		else if (command == "print") {
			Serial.println("\033[2J");
			Serial.println("\033[H");
			print_command = !print_command;
		}
		else {
			Serial.println("\033[2J");
			Serial.println("\033[H");
			Serial.print("❌ Unknown command");
			Serial.println(command);
			Serial.println("Tapez 'help' pour voir la liste des commandes.");
		}
	}
	else {
			Serial.print("❌ Unknown command");
			Serial.println(command);
			Serial.println("Tapez 'help' pour voir la liste des commandes.");
	}

}

void read_write_HSPI_task(void *parameter) {

	static constexpr size_t BUFFER_SIZE = 20;

	uint8_t tx_buf[BUFFER_SIZE] {0};
	uint8_t rx_buf[BUFFER_SIZE] {0};

    while (1) {
    	memcpy(tx_buf, &robot_state, sizeof(Odometry_t));

    	//SPI COMMUNICATION
    	HSPI_queue(tx_buf, rx_buf, BUFFER_SIZE);

    	memcpy(&commands, rx_buf, sizeof(DataFromPi));
		angular_velocity_reference = commands.target_angular_velocity;
		linear_velocity_reference = commands.target_linear_velocity;
    }
}

void read_serial_input_task(void *parameter) {
	String inputBuffer = "";  // Stocke la chaîne reçue
	while (1) {
		
		while (Serial.available()) {  // Vérifie si des données sont disponibles
			char c = Serial.read();
			if (c == '\n') {  // Si on appuie sur "Enter"
				if (inputBuffer.length() > 0) {
					command_handler(inputBuffer);  // Exécuter la commande
					inputBuffer = "";  // Réinitialiser le buffer
				}
			} 
			else if (c != '\r') {  // Ignorer le retour chariot (Windows)
				inputBuffer += c;  // Ajouter le caractère à la commande
			}
		}
	vTaskDelay(10 / portTICK_PERIOD_MS);  // Évite un CPU bloqué	
	}
}

void read_wifi_input_task(void *parameter) {
	while (1){
		if (page_loaded == true){
			command_handler(input_message);
			page_loaded = false;
			vTaskDelay(1 / portTICK_PERIOD_MS);  // Évite un CPU bloqué
		}

	vTaskDelay(10 / portTICK_PERIOD_MS);  // Évite un CPU bloqué	
	}
}

