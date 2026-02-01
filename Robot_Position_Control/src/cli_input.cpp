#include "cli_input.h"

float orientation_reference = 0;
float right_distance_reference = 0;
float left_distance_reference = 0;

bool orientation_command = false;
bool forward_command = false; 
bool backward_command = false; 
bool rightMotorCommand = false;
bool leftMotorCommand  = false;

volatile bool right_motor_enabled = false;
volatile bool left_motor_enabled = false;

void command_handler(String command) {
    command.trim();  
	float val = 0;

    if (command == "help") {
        Serial.println("Available commands");
		Serial.println("  help        - Display the list of available commands");
		Serial.println("  left        - Rotate left (-90°)");
		Serial.println("  right       - Rotate right (+90°)");
		Serial.println("  turn        - Rotate by a specified azimuth");
		Serial.println("  forward     - Move forward for a specified distance");
		Serial.println("  rightMotor  - Start the right motor");
		Serial.println("  leftMotor   - Start the left motor");
		Serial.println("  stop        - Stop both motors");
		Serial.println("  status      - Display the current motor status");
    }
	else if (command == "turn") {
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
		Serial.println("Turns left (-90°)");
		orientation_reference = orientation_reference - 90;
		rotate(orientation_reference);
	}
	else if (command == "right") {
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
			Serial.println("Motor stopped!");
			forward_command = false;
		}
	}	
	else if (command == "backward") {
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
		Serial.println("Motor stopped!");
	} 
	else if (command == "status") {
			Serial.print("current orientation : ");
			Serial.print(current_position.robot_orientation_deg);
			Serial.print("\tdistance travelled : ");
			Serial.println(current_position.robot_distance_travelled);
	}
	else {
		Serial.print("❌ Unknown command");
		Serial.println(command);
		Serial.println("Tapez 'help' pour voir la liste des commandes.");
	}
}

void HSPIInitialisation(){
	spi_bus_config_t buscfg;
	memset(&buscfg, 0, sizeof(buscfg));

	buscfg.mosi_io_num = HSPI_MOSI;
	buscfg.miso_io_num = HSPI_MISO;
	buscfg.sclk_io_num = HSPI_SCLK;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 32;

	spi_slave_interface_config_t slvcfg;
	memset(&slvcfg, 0, sizeof(slvcfg));


	slvcfg.spics_io_num = HSPI_CS;
	slvcfg.queue_size = 1;
	slvcfg.mode = 0;

	spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, 0);
}

void read_HSPI_input_task(void *parameter) {
    char rxBuffer[5] = {0};

    while (1) {
        spi_slave_transaction_t t = {};
        t.length = 32;
        t.rx_buffer = rxBuffer;

        spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);

        String cmd = String(rxBuffer); 
        command_handler(cmd);
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

