const speedSlider = document.querySelector("#speedSlider");
const metalDetectorSensorValue = document.querySelector("#metal-detector-sensor-value");
const metalDetectorAlertText = document.querySelector("#metal-detector-alert-text");
const metalDetectorDetectedText = document.querySelector("#metal-detector-detected-text");
const gpsLatitudeValueText = document.querySelector("#gps-latitude-value-text");
const gpsLongitudeValueText = document.querySelector("#gps-longitude-value-text");
const gpsSpeedValueText = document.querySelector("#gps-speed-value-text");
const gpsAltitudeValueText = document.querySelector("#gps-altitude-value-text");
const gasSensorValueText = document.querySelector("#gas-sensor-value-text");
const gasSensorDetectedText = document.querySelector("#gas-sensor-detected-text");
const gasSensorAlertText = document.querySelector("#gas-sensor-alert-text");

speedSlider.addEventListener("input", function () {
    const sliderValue = speedSlider.value;
    console.log("Speed Slider Value: " + sliderValue);
    const sendSpeed = `speed-${sliderValue}`;
    sendCommand(sendSpeed);
});

speedSlider.addEventListener("mouseout", () => {
    speedSlider.blur();
});


const video = document.querySelector("#espCamStream");
video.src = `http://${prompt("Enter ESP32 CAM IP :")}/1600x1200.mjpeg`;
// http://192.168.137.17/1600x1200.mjpeg (Example Link)

const esp32ip = prompt("Enter ESP32 RC Control IP :");
const websocketvalue = `ws://${esp32ip}:81`;
// ws://192.168.29.1:81 (Example websocketvalue)
let socket = new WebSocket(websocketvalue);

socket.onmessage = function (event) {
    let data = JSON.parse(event.data);
    let metalDetectorValue = data.metalDetector;
    let gpsLatitudeValue = data.gpsLatitude;
    let gpsLongitudeValue = data.gpsLongitude;
    let gpsSpeedValue = data.gpsSpeed;
    let gpsAltitudeValue = data.gpsAltitude;
    let gasSensorValue = data.gasSensor;

    console.log("GPS Latitude Value: " + gpsLatitudeValue);
    console.log("GPS Longitude Value: " + gpsLongitudeValue);
    console.log("GPS Speed Value: " + gpsSpeedValue);
    console.log("GPS Altitude Value: " + gpsAltitudeValue);
    console.log("Gas Sensor Value: " + gasSensorValue);

    metalDetectorSensorValue.textContent = metalDetectorValue.toString();
    if (metalDetectorValue > 400) {
        metalDetectorAlertText.textContent = "Metal Detected!";
        metalDetectorAlertText.style.color = "#ff4c76";
        metalDetectorDetectedText.textContent = "Yes";
    } else {
        metalDetectorAlertText.textContent = "No Metal Detected!";
        metalDetectorAlertText.style.color = "#e3ad19";
        metalDetectorDetectedText.textContent = "No";
    }

    gpsLatitudeValueText.textContent = gpsLatitudeValue || gpsLatitudeValueText.textContent;
    gpsLongitudeValueText.textContent = gpsLongitudeValue || gpsLongitudeValueText.textContent;
    gpsSpeedValueText.textContent = gpsSpeedValue || gpsSpeedValueText.textContent;
    gpsAltitudeValueText.textContent = gpsAltitudeValue || gpsAltitudeValueText.textContent;

    gasSensorValueText.textContent = gasSensorValue.toString();
    if (gasSensorValue > 300) {
        gasSensorAlertText.textContent = "Gas Detected!";
        gasSensorAlertText.style.color = "#ff4c76";
        gasSensorDetectedText.textContent = "Yes";
    } else {
        gasSensorAlertText.textContent = "No Gas Detected!";
        gasSensorAlertText.style.color = "#e3ad19";
        gasSensorDetectedText.textContent = "No";
    }
};

function sendCommand(command) {
    socket.send(command);
}

document.addEventListener('keydown', function (event) {
    switch (event.key) {
        case 'ArrowUp':
            sendCommand('forward');
            break;
        case 'ArrowLeft':
            sendCommand('left');
            break;
        case 'ArrowRight':
            sendCommand('right');
            break;
        case 'ArrowDown':
            sendCommand('backward');
            break;
        case ' ':
            sendCommand('stop');
            break;
    }
});