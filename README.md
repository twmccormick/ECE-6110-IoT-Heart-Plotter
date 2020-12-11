# ECE-6110-IoT-Heart-Plotter
 An IoT heart-rate plotter for the STM32 platform that uses the MAX30102 optical sensor.
 
 After configuring and connecting to a WIFI network, the embedded HTTP server begins listening for requests made by clients. In between requests, the connected sensor collects data for about one second, processing the data to extract a heart rate, which only occurs if a person’s finger is placed onto the sensor, otherwise the measured heart-rate becomes zero. Triggered by a timer, an interrupt first shifts the heart-rate storage array to the right, effectively deleting the oldest measurement, and then storing the newest sample in the open space in the array. Because this interrupt is executed every one second, the heart-rate storage array always contains the previous ten seconds of heart-rate measurements. If the HTTP server detects a request, it then constructs a dynamic plot using a JavaScript plotting API, which access the previous ten seconds of heart-rate samples from the array and encodes the results in the page being constructed, which is then sent to the client once construction completes. Once this page is loaded by the client, an auto-refresh line within the page will automatically request another page every one second, and thus the page is now a ‘live’ view of the heart-rate data from whomever places their finger onto the sensor.   
