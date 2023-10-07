/*
     Basic example code for controlling a stepper without library

     by Dejan, https://howtomechatronics.com
*/

// defining variables
float feed_rate;
int motor_direction;
int motor_status;
int i;
int k;
unsigned long t0;
int time_delay_ms_int;
String Prompt;
float retraction_distance;

// current sensor variables
float CurrentSensorOutput;
float CurrentReading;
float CurrentThreshold = .15;
const int CURRENT_SENSOR_PIN = A0; // Input pin for measuring Vout of current sensor

// monitor time of current run (mode 1)
long t0_full_run;
long tf_full_run;
// monitor time of current machining step
long t0_current_step;
long tf_current_step;
// temporary variable to store time reading
long t_tmp;

// monitoring time of current run (mode 2)
float experiment_runtime; // user inputed duration of experiment
long experiment_runtime_ms; // same value in ms
// variables for monitoring start and end time of experiment
long t_experiment_start;
long t_experiment_end;
// temp variable to store current time reading
long t_current;


// time variable for printout
  float time_global_printout_s;

// global variables related to machining distance
// global means that any function can access them
  // values related to total distance
    float motor_distance_total_mm;
    float motor_distance_total_um;
    float motor_distance_total_steps;
    int   motor_distance_total_steps_int;
  // values related to tool-substrate approach
    int   motor_distance_approach_steps_int;
    float motor_distance_approach_um;
    float motor_distance_approach_mm;
  // values related to moved distance so far during machining
    float motor_distance_machined_mm;
  // global printout value
    float motor_distance_global_printout_mm;

  //current
    float current_global_printout_mA;

// temperature sensor
  float TemperatureReading;    


// action_string
  String ActionString;

    
// defines pins
#define dirPin  6
#define stepPin 5
#define ECMPin 2
#define PumpPin 3



//MLX90614 Library
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();



//custom functions
  void serial_print_line();

void serial_print_line(){
          // time
            Serial.print("Time(s): "); 
            Serial.print(time_global_printout_s); 
            Serial.print("\t"); 
          // action string
            Serial.print("Action: ");
            Serial.print(ActionString);
            Serial.print("\t"); 
          // distance
            Serial.print("Distance(mm): "); 
            Serial.print(motor_distance_global_printout_mm);
            Serial.print("\t");
          //current sensor
            Serial.print("Current(mA): "); 
            Serial.print(current_global_printout_mA); 
            Serial.print("\t");
          // temperature sensor
            Serial.print("Tempature(C): "); 
            Serial.print(TemperatureReading); 
            Serial.print("\t");
          // line break
            Serial.println("  "); // current

}
  


//***************************************
//   FUNCTIONS CALLED IN THE MAIN CODE
//***************************************

// User input function, string
String UserInputString(String UserPrompt) {
  Serial.println(UserPrompt);
  while (Serial.available() > 0) { // clears earlier data
    Serial.read();
  }

  while (Serial.available() == 0) { // waits for data to be entered
  }

  String UserInput = Serial.readString(); // read until timeout
  UserInput.trim(); // remove \r \n (whitespace) at end of string
  Serial.println(UserInput);
  return UserInput;
}
 

// User input function, integer
int UserInputInt(String UserPrompt) {
  Serial.println(UserPrompt); // asks for user prompt
  // This code will clear the earlier input value
  while (Serial.available() >0) {
    Serial.read ();
  } 
  
  // code pauses while waiting for an input
  while (Serial.available() == 0) {
  }

  int UserInput = Serial.parseInt();
  Serial.println(UserInput);
  return UserInput;
}


// User input function, float
float UserInputFloat(String UserPrompt) {
  Serial.println(UserPrompt);
  while (Serial.available() > 0) {
    Serial.read();
  }

  while (Serial.available() == 0) {
  }

  float UserInput = Serial.parseFloat();
  Serial.println(UserInput);
  return UserInput;
}


// Read current sensor function
float ReadCurrentSensor() {
  // Current Sensor Reading
  float CurrentSensorOutput = analogRead(CURRENT_SENSOR_PIN);
  // analog pin maps 0 to 5v value to a value ranging from 0 to 1024. 5v /1024 units = 0.049mV/unit
  float CurrentSensorVoltage = CurrentSensorOutput * 0.0049; // converts from (0 to 1024) value to voltage
  float Current = ( 2.5 - CurrentSensorVoltage ) / 0.185;
  return Current;
  // Power source is 5v. Base voltage is 2.5 volts when current is zero. Any decrease in this value represents a change in current.
  // Current increases 1A for every 185mV change from the base voltage.
}


//position workpiece function
void PositionWorkpiece(float retraction_distance) {
  // CALCULATE STEPS FOR DESIRED RETRACTION DISTANCE
  float um_per_step = 35;
  float retraction_steps = retraction_distance / um_per_step;
  int retraction_steps_int = round(retraction_steps);

  ActionString   = "position_workpiece";
  CurrentReading = ReadCurrentSensor(); // initial current reading


  motor_distance_approach_steps_int = 0;
  
  while (CurrentReading < CurrentThreshold) {


    motor_distance_approach_steps_int++;
    motor_distance_approach_um = motor_distance_approach_steps_int * um_per_step;
    motor_distance_approach_mm = motor_distance_approach_um * .001;
    
    digitalWrite(stepPin, HIGH); // move motor forward
    digitalWrite(stepPin, LOW);

      //update time
      time_global_printout_s  = 0. * .001;
      
      // update distance
      motor_distance_global_printout_mm  = motor_distance_approach_mm;

      // update current
      CurrentReading                  = ReadCurrentSensor();
      current_global_printout_mA      = CurrentReading *1000.;

      //update temperature
      TemperatureReading              = mlx.readObjectTempC();//-999.;

      // line printout
      serial_print_line();


    
    delay(75);
  }

  // moves workpiece back user-inputted distance
  for (k=1; k<=retraction_steps_int; k++) {
    digitalWrite(dirPin, LOW); // change motor direction to backwards
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    delay(50); // ms, delay controlls speed of retraction (higher delay = slower speed)
  }
  digitalWrite(dirPin, HIGH); // change the motor direction back to forward
}


//*******************************
// *** MAIN CODE STARTS HERE ****
//*******************************

// setup loop runs 1 time
void setup() {

  Serial.begin(115200); // starts serial communication between arduino and computer



  // temperature sensor setup
  mlx.begin(); // start temperature sensor communication
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());




  // SETTING PIN MODES
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(ECMPin, OUTPUT);
  pinMode(PumpPin, OUTPUT);
  
  // Relay 1, controlled by ECMPin
  // When on LOW, + power supply connected to resistor(R_series), R_series connected to substrate
  // If tool and substrate are shorted, and power supply is off, resistance should be equal to R_series
  digitalWrite(ECMPin, LOW); 
  // Relay 2, controlled by PumpPin
  digitalWrite(PumpPin, LOW);
  // ECM VOLTAGE AND PUMP AUTOMATICALLY SET TO LOW (OFF)


//  ***** CURRENT TESTING CODE *****
  Prompt = "Would you like to test the current for 15 seconds? (Enter 1 for yes and 2 for no): ";
  int current_test = UserInputInt(Prompt);
    
  if (current_test == 1) {

    ActionString   = "current_test";
    
    // record run start time
    int t_current_run_start = millis();
    int t = 0;

    while (t < 15000) {
      
      //update time
      t                       = millis() - t_current_run_start;
      time_global_printout_s  = t * .001;
      
      // update distance
      motor_distance_global_printout_mm     = 0.;

      // update current
      CurrentReading                  = ReadCurrentSensor();
      current_global_printout_mA      = CurrentReading *1000.;

      //update temperature
      TemperatureReading              = mlx.readObjectTempC();//-999.;

      // line printout
      serial_print_line();
      

      // Time delay between readings
      delay(100);
    }
  }

  // ***** DATA INPUTS *****
  String material = UserInputString("Enter substrate material: ");
  float density = UserInputFloat("Enter substrate density: ");
  float thickness = UserInputFloat("Enter substrate thickness: ");
  float initial_mass = UserInputFloat("Enter initial substrate mass: ");
  String electrolyte_compisition = UserInputString("Enter electrolyte compisition: ");
  float voltage = UserInputFloat("Enter the voltage magnitude of the power supply: ");
  String pulse = UserInputString("Is the current pulsed? (Y/N): ");
  if (pulse == "Y") {
    String frequency = UserInputString("Enter the frequency: ");
    String duty_cycle = UserInputString("Enter duty cycle: ");
    String feeding_capacitor_value = UserInputString("Enter feeding capacitor value: ");
  }

  // ******* EXPERIMENT MODES *******
  delay(500); // delay gives time for previous lines to load before the next one is sent
  Serial.println("Mode 1 moves the workpiece during machining and stops after a user-inputed distance.");
  delay(500);
  Serial.println("Mode 2 keeps the workpiece in the same spot and stops after a user-inputed time.");
  delay(500);

  Prompt = "Enter the experiment mode. (Enter 1 for 'Mode 1' and 2 for 'Mode 2'): ";
  int experiment_mode = UserInputInt(Prompt);

  Prompt = "Enter the retraction distance in microns (Must be >= 35 microns, 35 microns = 1 step): ";
  retraction_distance = UserInputFloat(Prompt);


  // ******** CODE FOR MODE 1 **********
  if (experiment_mode == 1) {
    
    Prompt = "Enter the feed rate in microns per second: ";
    feed_rate = UserInputFloat(Prompt);

    Prompt = "Enter the machining distance in mm: ";
    motor_distance_total_mm = UserInputFloat(Prompt);

//
//    // SETTING PIN MODES
//    pinMode(stepPin, OUTPUT);
//    pinMode(dirPin, OUTPUT);
//    pinMode(ECMPin, OUTPUT);
//    pinMode(PumpPin, OUTPUT);

  
    // SETTING MOTOR DIRECTION
    digitalWrite(dirPin, HIGH); // LOW means backwards direction, HIGH means forward direction.

  
    // CALCULATING TIME DELAY BASED ON GIVEN FEED RATE
    // 2500 steps = 10cm
    // 45 steps = 3mm 
    // below is the fixed distance that workpiece travels per step of the motor)
    float um_per_step = 35; // microns per step // original was 200/3 microns/step
    //calculating time delay
    float time_delay_s = um_per_step/feed_rate; // seconds
    float time_delay_ms = time_delay_s * 1000.; // milliseconds
    int time_delay_ms_int = (int)round(time_delay_ms); // milliseconds, converted to integer form

  
    // CALCULATE STEPS FOR DESIRED MACHINING DISTANCE
    motor_distance_total_um             = motor_distance_total_mm * 1000;
    motor_distance_total_steps          = motor_distance_total_um / um_per_step;
    motor_distance_total_steps_int      = round(motor_distance_total_steps);

    
    // POSITIONING THE WORKPIECE
    Serial.println("Positioning workpiece...");
    delay(2000); // delay so user can read print statement
    PositionWorkpiece(retraction_distance); // calls positioning function
    Serial.println("Initial position reached.");
    delay(500);
    Serial.println("Starting machining...");


    // START MACHINING

    ActionString = "machining_mode_1";
    
    digitalWrite(PumpPin, HIGH); // LOW = pump off, HIGH = pump on
    digitalWrite(ECMPin, HIGH); // turns on ECM voltage for machining
    delay(5000); // waits 5 seconds for pumps to start working

    // track time since experiment start
    t0_full_run = millis();
    tf_full_run = millis() - t0_full_run;
    
    for(i=1;i<=motor_distance_total_steps_int;i++) {  
       digitalWrite(stepPin, HIGH);
       digitalWrite(stepPin, LOW);   
    
       // initial time readings to start the while loop
       t0_current_step = millis();
       tf_current_step = millis() - t0_current_step;
  

       motor_distance_machined_mm   = i * um_per_step; // mm
    
       while (tf_current_step < time_delay_ms_int) {
          // update time readings
          t_tmp = millis();
          tf_full_run     = t_tmp - t0_full_run;
          tf_current_step = t_tmp - t0_current_step;

               //update time
                  time_global_printout_s  = tf_current_step * .001;
                  
                  // update distance
                  motor_distance_global_printout_mm = motor_distance_machined_mm;
            
                  // update current
                  CurrentReading                  = ReadCurrentSensor();
                  current_global_printout_mA      = CurrentReading *1000.;
            
                  //update temperature
                  TemperatureReading              = mlx.readObjectTempC();//-999.;
            
                  // line printout
                  serial_print_line();

          
       }
    }
    
    // turns off pump and ECM voltage when machining is finished
    digitalWrite(PumpPin, LOW); 
    digitalWrite(ECMPin, LOW); 

    Serial.println("Machining complete.");
  }



  // ******* CODE FOR MODE 2 ********
  if (experiment_mode == 2) {

    

    // USER INPUT
    Prompt = "Enter the desired experiment runtime in minutes: ";
    experiment_runtime = UserInputFloat(Prompt);

    digitalWrite(dirPin, HIGH); // sets motor direction to forwards

    // POSITION WORKPIECE
    Serial.println("Positioning workpiece...");
      delay(2000);
    PositionWorkpiece(retraction_distance); // calls positioning function
    Serial.println("Initial position reached.");
    delay(500);
    Serial.println("Starting machining...");
    delay(1000);

//
//    // setting pin status
//    pinMode(stepPin, OUTPUT);
//    pinMode(dirPin, OUTPUT);
//    pinMode(ECMPin, OUTPUT);
//    pinMode(PumpPin, OUTPUT);

    // TURNING ON PUMP AND ECM VOLTAGE
    digitalWrite(PumpPin, HIGH); // LOW = Pump off, HIGH = Pump on
    digitalWrite(ECMPin, HIGH); //LOW = ECM voltage off, HIGH = ECM voltage on
    delay(5000); // 5 second pause to let pumps start
  
    ActionString = "machining_mode_2";
    // STARTS MACHINING FOR GIVEN TIME
    t_experiment_start = millis();
    t_current = millis() - t_experiment_start;
    experiment_runtime_ms = experiment_runtime * 60000;

    // runs until user-inputted time duration has passed
    while (t_current < experiment_runtime_ms) {
      //update time
      t_current = millis() - t_experiment_start;

      //update time
      time_global_printout_s  = t_current * .001;
      
      // update distance
      motor_distance_global_printout_mm     = 0.;

      // update current
      CurrentReading                  = ReadCurrentSensor();
      current_global_printout_mA      = CurrentReading *1000.;

      //update temperature
      TemperatureReading              = mlx.readObjectTempC();//-999.;

      // line printout
      serial_print_line();
    }
    digitalWrite(PumpPin, LOW);
    digitalWrite(ECMPin, LOW); // turn off pump and ECM voltage when machining is finished

    Serial.println("Machining complete.");
  }
  

  // calculate material removal rate (modes 1 and 2)
  float final_mass = UserInputFloat("Enter the final mass of the substrate: ");
  float volume_machined = (initial_mass - final_mass) / density;
  float MRR = volume_machined / t_current; 
}

void loop() {
  delay(500);
}
