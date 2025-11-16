idf_component_register(
    SRCS 
        "application.cpp"
        
        "TickTimer.cpp" 
        "SerialComm.cpp" 
        "Motor.cpp" 
        "MainApp.cpp" 
        "Logger.cpp" 
        "Commands.cpp" 
        "Comm.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        esp_timer 
        driver 
        lwip
)