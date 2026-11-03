/*
  ============================================================
  ESP32-S3 WROOM CAM + OV3660
  Fast Browser Streaming + RTSP Priority + OTA + Web Serial
  ============================================================

  Wi-Fi AP
  ------------------------------------------------------------
  SSID       : ESP32-CAM
  Password   : 12345678
  IP         : 192.168.4.1

  Portal
  ------------------------------------------------------------
  http://192.168.4.1/

  RTSP
  ------------------------------------------------------------
  rtsp://192.168.4.1:554/mjpeg/1

  JPEG
  ------------------------------------------------------------
  http://192.168.4.1/jpg

  Serial
  ------------------------------------------------------------
  http://192.168.4.1/serial

  OTA
  ------------------------------------------------------------
  http://192.168.4.1/update


  STREAMING LOGIC
  ------------------------------------------------------------

  Normal:
      Browser Online Streaming = ON

  RTSP client connects:
      Browser Streaming = OFF
      RTSP = ON

  RTSP client disconnects:
      RTSP = OFF
      Browser Streaming automatically resumes


  IMPORTANT ARDUINO IDE SETTING
  ------------------------------------------------------------

  Tools -> PSRAM -> OPI PSRAM


  CAMERA
  ------------------------------------------------------------

  Physical sensor : OV3660
  Camera object   : OV2640 abstraction used by Micro-RTSP


  IMPORTANT
  ------------------------------------------------------------

  PWDN_GPIO_NUM = -1

  Therefore CAMERA OFF is a software shutdown.
  It does not physically disconnect power from the OV3660.
*/


// ============================================================
// 1. LIBRARIES
// ============================================================

#include "OV2640.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include "SimStreamer.h"
#include "OV2640Streamer.h"
#include "CRtspSession.h"

#include <ElegantOTA.h>


// ============================================================
// 2. WIFI ACCESS POINT
// ============================================================

const char* AP_SSID = "ESP32-CAM";
const char* AP_PASSWORD = "12345678";

IPAddress AP_IP(
    192, 168, 4, 1
);

IPAddress AP_GATEWAY(
    192, 168, 4, 1
);

IPAddress AP_SUBNET(
    255, 255, 255, 0
);


// ============================================================
// 3. HTTP SERVER
// ============================================================

WebServer server(80);


// ============================================================
// 4. RTSP SERVER
// ============================================================

WiFiServer rtspServer(554);


// ============================================================
// 5. CAMERA
// ============================================================

OV2640 cam;

camera_config_t cameraConfig;


// ============================================================
// 6. MICRO-RTSP STREAMER
// ============================================================

CStreamer* streamer = NULL;


// ============================================================
// 7. RTSP CLIENT
// ============================================================

WiFiClient rtspClient;


// ============================================================
// 8. BROWSER STREAM CLIENT
// ============================================================

WiFiClient webStreamClient;

bool webStreamActive = false;


// ============================================================
// 9. CAMERA STATE
// ============================================================

bool cameraEnabled = true;
bool cameraReady = false;


// ============================================================
// 10. STREAMING STATE
// ============================================================

/*
  RTSP has priority.

  If an RTSP client is connected:

      RTSP_ACTIVE = true

  Browser streaming is automatically stopped.
*/

bool rtspActive = false;


// ============================================================
// 11. CAMERA PIN CONFIGURATION
// ============================================================

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1

#define XCLK_GPIO_NUM    15

#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5

#define Y2_GPIO_NUM      11
#define Y3_GPIO_NUM      9
#define Y4_GPIO_NUM      8
#define Y5_GPIO_NUM      10
#define Y6_GPIO_NUM      12
#define Y7_GPIO_NUM      18
#define Y8_GPIO_NUM      17
#define Y9_GPIO_NUM      16

#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13


// ============================================================
// 12. STREAM TIMING
// ============================================================

/*
  Browser:

      50 ms = approximately 20 FPS

  This is a good target for VGA MJPEG.

  The actual FPS can be lower if the camera or Wi-Fi
  cannot keep up.
*/

const uint32_t ONLINE_FRAME_INTERVAL = 50;


/*
  RTSP:

      66 ms = approximately 15 FPS

  SVGA at JPEG quality 12 produces a larger image, so
  15 FPS is a safer target for the ESP32.
*/

const uint32_t RTSP_FRAME_INTERVAL = 66;


uint32_t lastOnlineFrame = 0;
uint32_t lastRTSPFrame = 0;


// ============================================================
// 13. RTSP DISCONNECT HANDLING
// ============================================================

/*
  Small delay before returning to browser streaming.

  This prevents rapid:

      RTSP disconnect
          ->
      browser start
          ->
      RTSP reconnect
          ->
      browser stop

  from happening too aggressively.

  500 ms is enough for normal cleanup.
*/

const uint32_t RTSP_RESUME_DELAY = 500;

uint32_t rtspDisconnectedAt = 0;


// ============================================================
// 14. WEB SERIAL LOG
// ============================================================

String webLog = "";

const size_t MAX_LOG_LENGTH = 6000;


// ============================================================
// 15. LOGGING
// ============================================================

void logMessage(const String& message)
{
    Serial.println(message);

    webLog += message;
    webLog += "\n";


    /*
      Prevent the log from consuming memory forever.
    */

    if (webLog.length() > MAX_LOG_LENGTH)
    {
        webLog.remove(
            0,
            3000
        );
    }
}


// ============================================================
// 16. PREPARE CAMERA CONFIGURATION
// ============================================================

void prepareCameraConfig()
{
    /*
      Clear the structure first.

      This prevents unused fields from containing
      random values.
    */

    memset(
        &cameraConfig,
        0,
        sizeof(cameraConfig)
    );


    // --------------------------------------------------------
    // Camera clock
    // --------------------------------------------------------

    cameraConfig.ledc_channel =
        LEDC_CHANNEL_0;

    cameraConfig.ledc_timer =
        LEDC_TIMER_0;


    // --------------------------------------------------------
    // Camera data pins
    // --------------------------------------------------------

    cameraConfig.pin_d0 = Y2_GPIO_NUM;
    cameraConfig.pin_d1 = Y3_GPIO_NUM;
    cameraConfig.pin_d2 = Y4_GPIO_NUM;
    cameraConfig.pin_d3 = Y5_GPIO_NUM;
    cameraConfig.pin_d4 = Y6_GPIO_NUM;
    cameraConfig.pin_d5 = Y7_GPIO_NUM;
    cameraConfig.pin_d6 = Y8_GPIO_NUM;
    cameraConfig.pin_d7 = Y9_GPIO_NUM;


    // --------------------------------------------------------
    // Camera control pins
    // --------------------------------------------------------

    cameraConfig.pin_xclk =
        XCLK_GPIO_NUM;

    cameraConfig.pin_pclk =
        PCLK_GPIO_NUM;

    cameraConfig.pin_vsync =
        VSYNC_GPIO_NUM;

    cameraConfig.pin_href =
        HREF_GPIO_NUM;


    // --------------------------------------------------------
    // SCCB / I2C
    // --------------------------------------------------------

    cameraConfig.pin_sccb_sda =
        SIOD_GPIO_NUM;

    cameraConfig.pin_sccb_scl =
        SIOC_GPIO_NUM;


    // --------------------------------------------------------
    // Power and reset
    // --------------------------------------------------------

    cameraConfig.pin_pwdn =
        PWDN_GPIO_NUM;

    cameraConfig.pin_reset =
        RESET_GPIO_NUM;


    // --------------------------------------------------------
    // Camera clock
    // --------------------------------------------------------

    cameraConfig.xclk_freq_hz =
        20000000;


    // --------------------------------------------------------
    // JPEG output
    // --------------------------------------------------------

    cameraConfig.pixel_format =
        PIXFORMAT_JPEG;


    /*
      Start at VGA.

      Browser streaming uses VGA because it provides a
      much better latency/FPS balance than SVGA.
    */

    cameraConfig.frame_size =
        FRAMESIZE_VGA;


    /*
      JPEG quality:

          Lower number = higher quality
          Higher number = smaller image

      14 is a good starting point for fast browser feed.
    */

    cameraConfig.jpeg_quality =
        14;


    // --------------------------------------------------------
    // PSRAM frame buffers
    // --------------------------------------------------------

    cameraConfig.fb_count = 2;


    /*
      IMPORTANT FOR LOW LATENCY

      CAMERA_GRAB_LATEST means the camera tries to keep
      the newest frame available.

      We do not want the ESP32 to send an old frame after
      the network has slowed down.
    */

    cameraConfig.grab_mode =
        CAMERA_GRAB_LATEST;


    /*
      Store frame buffers in PSRAM.

      Requires:

          Tools
          -> PSRAM
          -> OPI PSRAM
    */

    cameraConfig.fb_location =
        CAMERA_FB_IN_PSRAM;
}


// ============================================================
// 17. INITIALIZE CAMERA
// ============================================================

bool initializeCamera()
{
    logMessage("");
    logMessage(
        "[CAMERA] Initializing OV3660..."
    );


    prepareCameraConfig();


    /*
      Initialize camera.

      We initialize only once during normal operation.

      This is important for stability.

      We do NOT continuously deinitialize/reinitialize
      the camera when RTSP changes.
    */

    esp_err_t result =
        cam.init(cameraConfig);


    if (result != ESP_OK)
    {
        logMessage(
            "[CAMERA] ERROR: Initialization failed."
        );

        return false;
    }


    /*
      Give the OV3660 a short amount of time to stabilize.

      This delay is only during startup.
      It does NOT run during streaming.
    */

    delay(100);


    sensor_t* sensor =
        esp_camera_sensor_get();


    if (sensor == NULL)
    {
        logMessage(
            "[CAMERA] ERROR: Sensor unavailable."
        );

        return false;
    }


    // --------------------------------------------------------
    // Sensor detection
    // --------------------------------------------------------

    Serial.printf(
        "[CAMERA] Sensor PID: 0x%02X\n",
        sensor->id.PID
    );


    if (sensor->id.PID == OV3660_PID)
    {
        logMessage(
            "[CAMERA] OV3660 detected."
        );
    }
    else
    {
        logMessage(
            "[CAMERA] WARNING: Sensor is not OV3660."
        );
    }


    // --------------------------------------------------------
    // Image orientation
    // --------------------------------------------------------

    sensor->set_vflip(
        sensor,
        1
    );

    sensor->set_hmirror(
        sensor,
        1
    );


    // --------------------------------------------------------
    // Brightness
    // --------------------------------------------------------

    sensor->set_brightness(
        sensor,
        1
    );


    /*
      Disable Wi-Fi power saving.

      This is one of the most important settings for
      reducing streaming latency in AP mode.
    */

    WiFi.setSleep(false);


    cameraReady = true;


    logMessage(
        "[CAMERA] Camera ready."
    );


    return true;
}


// ============================================================
// 18. CHANGE CAMERA STREAM SETTINGS
// ============================================================

void setCameraMode(bool rtspMode)
{
    if (!cameraReady)
    {
        return;
    }


    sensor_t* sensor =
        esp_camera_sensor_get();


    if (sensor == NULL)
    {
        return;
    }


    // --------------------------------------------------------
    // RTSP MODE
    // --------------------------------------------------------

    if (rtspMode)
    {
        /*
          SVGA:

              800 x 600

          Better image quality for RTSP.
        */

        sensor->set_framesize(
            sensor,
            FRAMESIZE_SVGA
        );


        /*
          Quality 12:

          Larger JPEG
          Better image quality
          More network traffic
        */

        sensor->set_quality(
            sensor,
            12
        );


        logMessage(
            "[CAMERA] RTSP settings active."
        );
    }


    // --------------------------------------------------------
    // BROWSER MODE
    // --------------------------------------------------------

    else
    {
        /*
          VGA:

              640 x 480

          Smaller frames mean:
              - lower latency
              - faster transmission
              - less Wi-Fi traffic
              - less browser processing
        */

        sensor->set_framesize(
            sensor,
            FRAMESIZE_VGA
        );


        /*
          Quality 14 is intentionally slightly lower
          than RTSP to keep browser streaming fast.
        */

        sensor->set_quality(
            sensor,
            14
        );


        logMessage(
            "[CAMERA] Browser settings active."
        );
    }
}


// ============================================================
// 19. STOP BROWSER STREAM
// ============================================================

void stopBrowserStream()
{
    if (webStreamActive)
    {
        logMessage(
            "[WEB] Browser stream stopped."
        );
    }


    if (webStreamClient)
    {
        webStreamClient.stop();
    }


    webStreamClient =
        WiFiClient();


    webStreamActive = false;

    lastOnlineFrame = 0;
}


// ============================================================
// 20. STOP RTSP
// ============================================================

void stopRTSP()
{
    if (rtspClient)
    {
        rtspClient.stop();
    }


    rtspClient =
        WiFiClient();


    rtspActive = false;


    lastRTSPFrame = 0;


    rtspDisconnectedAt =
        millis();


    logMessage(
        "[RTSP] Client disconnected."
    );
}


// ============================================================
// 21. HANDLE ROOT PORTAL
// ============================================================

void handleRoot()
{
    String page = R"rawliteral(
<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
      content="width=device-width,initial-scale=1">

<title>ESP32-CAM</title>


<style>

/* =========================================================
   BASIC PAGE
   ========================================================= */

* {
    box-sizing: border-box;
}

body {
    margin: 0;

    font-family:
        Arial,
        Helvetica,
        sans-serif;

    background: #0f172a;

    color: #e5e7eb;
}


/* =========================================================
   TOP HEADER
   ========================================================= */

.header {
    padding: 22px 18px;

    background: #020617;

    border-bottom:
        1px solid #1e293b;
}

.header h1 {
    margin: 0;

    font-size: 24px;
}

.header p {
    margin: 6px 0 0;

    color: #94a3b8;

    font-size: 13px;
}


/* =========================================================
   MAIN CONTAINER
   ========================================================= */

.container {
    width: 100%;

    max-width: 1000px;

    margin: auto;

    padding: 16px;
}


/* =========================================================
   CARD
   ========================================================= */

.card {
    background: #1e293b;

    border:
        1px solid #334155;

    border-radius: 14px;

    padding: 18px;

    margin-bottom: 16px;
}

.card h2 {
    margin-top: 0;

    font-size: 17px;
}


/* =========================================================
   STATUS
   ========================================================= */

.status {
    display: grid;

    grid-template-columns:
        repeat(
            auto-fit,
            minmax(150px, 1fr)
        );

    gap: 10px;
}

.statusBox {
    background: #0f172a;

    border:
        1px solid #334155;

    border-radius: 10px;

    padding: 13px;
}

.statusTitle {
    color: #94a3b8;

    font-size: 11px;

    margin-bottom: 5px;
}

.statusValue {
    font-size: 16px;

    font-weight: bold;
}


/* =========================================================
   CAMERA VIEW
   ========================================================= */

.cameraBox {
    width: 100%;

    background: #000;

    border-radius: 12px;

    overflow: hidden;

    min-height: 250px;

    display: flex;

    align-items: center;

    justify-content: center;
}

#camera {
    display: block;

    width: 100%;

    height: auto;

    max-height: 650px;

    object-fit: contain;

    background: #000;
}

.offMessage {
    padding: 60px 20px;

    text-align: center;

    color: #64748b;
}


/* =========================================================
   BUTTONS
   ========================================================= */

button,
a.button {
    display: inline-block;

    border: none;

    border-radius: 9px;

    padding: 11px 15px;

    margin: 4px;

    font-size: 14px;

    text-decoration: none;

    cursor: pointer;

    background: #334155;

    color: white;
}

button:hover,
a.button:hover {
    opacity: 0.85;
}

.blue {
    background: #2563eb !important;
}

.green {
    background: #16a34a !important;
}

.red {
    background: #dc2626 !important;
}


/* =========================================================
   INFORMATION
   ========================================================= */

.info {
    margin-top: 12px;

    padding: 11px;

    border-radius: 8px;

    background: #0f172a;

    color: #94a3b8;

    font-size: 13px;
}


/* =========================================================
   RTSP ADDRESS
   ========================================================= */

.rtsp {
    padding: 12px;

    background: #020617;

    border-radius: 8px;

    overflow-x: auto;

    font-family: monospace;

    font-size: 13px;
}


/* =========================================================
   FOOTER
   ========================================================= */

.footer {
    text-align: center;

    padding: 10px;

    color: #64748b;

    font-size: 11px;
}

</style>

</head>


<body>


<div class="header">

    <h1>ESP32-CAM</h1>

    <p>
        ESP32-S3 • OV3660 • Local Camera Server
    </p>

</div>


<div class="container">


<!-- =====================================================
     STATUS
     ====================================================== -->

<div class="card">

    <h2>System Status</h2>

    <div class="status">

        <div class="statusBox">

            <div class="statusTitle">
                CAMERA
            </div>

            <div
                class="statusValue"
                id="cameraStatus">
                ON
            </div>

        </div>


        <div class="statusBox">

            <div class="statusTitle">
                STREAM
            </div>

            <div
                class="statusValue"
                id="streamStatus">
                ONLINE
            </div>

        </div>


        <div class="statusBox">

            <div class="statusTitle">
                IP ADDRESS
            </div>

            <div class="statusValue">
                192.168.4.1
            </div>

        </div>

    </div>

</div>


<!-- =====================================================
     CAMERA VIEW
     ====================================================== -->

<div class="card">

    <h2>Live Camera</h2>


    <div class="cameraBox">

        <img
            id="camera"
            src="/stream"
            alt="Camera Feed"
        >

    </div>


    <div class="info"
         id="feedInfo">

        Browser streaming is active.

    </div>

</div>


<!-- =====================================================
     CAMERA CONTROL
     ====================================================== -->

<div class="card">

    <h2>Camera Control</h2>


    <button
        class="green"
        onclick="cameraOn()">

        Camera ON

    </button>


    <button
        class="red"
        onclick="cameraOff()">

        Camera OFF

    </button>


    <div class="info">

        Camera starts in browser streaming mode.
        Turning the camera OFF releases the camera
        driver and PSRAM frame buffers.

    </div>

</div>


<!-- =====================================================
     RTSP
     ====================================================== -->

<div class="card">

    <h2>RTSP Stream</h2>


    <div class="rtsp">

        rtsp://192.168.4.1:554/mjpeg/1

    </div>


    <div class="info">

        RTSP automatically gets priority when an RTSP
        client connects. The browser feed will stop
        automatically. When the RTSP client disconnects,
        the browser feed will automatically resume.

    </div>

</div>


<!-- =====================================================
     TOOLS
     ====================================================== -->

<div class="card">

    <h2>Tools</h2>


    <a
        class="button blue"
        href="/jpg"
        target="_blank">

        Snapshot

    </a>


    <a
        class="button"
        href="/serial">

        Serial Monitor

    </a>


    <a
        class="button"
        href="/update">

        OTA Update

    </a>

</div>


<div class="footer">

    ESP32-S3 CAM • OV3660 • Local Wi-Fi

</div>


</div>


<script>

/* =========================================================
   CAMERA ON
   ========================================================= */

function cameraOn()
{
    fetch('/api/camera?set=on')
        .then(() => {
            location.reload();
        });
}


/* =========================================================
   CAMERA OFF
   ========================================================= */

function cameraOff()
{
    fetch('/api/camera?set=off')
        .then(() => {
            location.reload();
        });
}


/* =========================================================
   STREAM STATUS
   ========================================================= */

function updateStatus()
{
    fetch('/api/status')
        .then(response => response.json())
        .then(data => {

            document.getElementById(
                'cameraStatus'
            ).textContent =
                data.camera ? 'ON' : 'OFF';


            document.getElementById(
                'streamStatus'
            ).textContent =
                data.rtsp ? 'RTSP' : 'ONLINE';


            let info =
                document.getElementById(
                    'feedInfo'
                );


            if (!data.camera)
            {
                info.textContent =
                    'Camera is OFF.';
            }

            else if (data.rtsp)
            {
                info.textContent =
                    'RTSP client is using the camera. Browser streaming is temporarily OFF.';
            }

            else
            {
                info.textContent =
                    'Browser streaming is active.';
            }

        });
}


/* =========================================================
   AUTOMATIC STATUS UPDATE
   ========================================================= */

setInterval(
    updateStatus,
    1000
);


/* =========================================================
   AUTOMATIC BROWSER FEED RECONNECT
   =========================================================

   When RTSP starts, ESP32 closes /stream.

   The browser will therefore lose the image.

   When RTSP ends, this function reloads the image
   connection so the portal feed comes back automatically.
   ========================================================= */

let lastRTSPState = false;


setInterval(function()
{
    fetch('/api/status')
        .then(response => response.json())
        .then(data => {

            if (
                lastRTSPState === true &&
                data.rtsp === false &&
                data.camera === true
            )
            {
                let image =
                    document.getElementById(
                        'camera'
                    );

                image.src =
                    '/stream?t=' +
                    Date.now();
            }


            lastRTSPState =
                data.rtsp;

        });

}, 1000);


/* =========================================================
   IMAGE ERROR RECOVERY
   =========================================================

   If the browser loses the MJPEG connection for any reason,
   periodically try to reconnect.

   This is especially useful after RTSP finishes.
   ========================================================= */

document
    .getElementById('camera')
    .onerror = function()
    {
        setTimeout(function()
        {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {

                    if (
                        data.camera &&
                        !data.rtsp
                    )
                    {
                        document
                            .getElementById('camera')
                            .src =
                                '/stream?t=' +
                                Date.now();
                    }

                });

        }, 1000);
};


</script>


</body>

</html>
)rawliteral";


    server.send(
        200,
        "text/html",
        page
    );
}


// ============================================================
// 22. BROWSER MJPEG STREAM
// ============================================================

void handleStream()
{
    /*
      RTSP has priority.

      If RTSP is currently being used, do not allow a browser
      stream to take the camera.
    */

    if (rtspActive)
    {
        server.send(
            503,
            "text/plain",
            "RTSP is currently using the camera."
        );

        return;
    }


    if (!cameraEnabled || !cameraReady)
    {
        server.send(
            503,
            "text/plain",
            "Camera is OFF."
        );

        return;
    }


    /*
      Close an old browser stream if one exists.
    */

    stopBrowserStream();


    /*
      Take the HTTP client.

      From this point the browser connection stays open
      and receives JPEG frames from processOnlineStream().
    */

    webStreamClient =
        server.client();


    webStreamClient.setNoDelay(
        true
    );


    /*
      MJPEG multipart HTTP header.
    */

    webStreamClient.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Expires: 0\r\n"
        "Connection: close\r\n\r\n"
    );


    webStreamActive = true;

    lastOnlineFrame = 0;


    logMessage(
        "[WEB] Browser camera connected."
    );
}


// ============================================================
// 23. PROCESS BROWSER STREAM
// ============================================================

void processOnlineStream()
{
    /*
      Never run browser streaming while RTSP is active.
    */

    if (rtspActive)
    {
        stopBrowserStream();

        return;
    }


    if (!cameraEnabled || !cameraReady)
    {
        stopBrowserStream();

        return;
    }


    if (!webStreamActive)
    {
        return;
    }


    if (
        !webStreamClient ||
        !webStreamClient.connected()
    )
    {
        stopBrowserStream();

        return;
    }


    uint32_t now =
        millis();


    /*
      Maintain approximately 20 FPS.

      Importantly, this does not block the CPU.
    */

    if (
        now - lastOnlineFrame <
        ONLINE_FRAME_INTERVAL
    )
    {
        return;
    }


    lastOnlineFrame =
        now;


    // --------------------------------------------------------
    // Capture newest frame
    // --------------------------------------------------------

    cam.run();


    uint8_t* frame =
        cam.getfb();


    size_t frameSize =
        cam.getSize();


    if (
        frame == NULL ||
        frameSize == 0
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Prepare multipart header
    // --------------------------------------------------------

    char header[96];


    int headerLength =
        snprintf(
            header,
            sizeof(header),

            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n",

            (unsigned int)frameSize
        );


    // --------------------------------------------------------
    // Send header
    // --------------------------------------------------------

    if (
        webStreamClient.write(
            (uint8_t*)header,
            headerLength
        )
        !=
        (size_t)headerLength
    )
    {
        stopBrowserStream();

        return;
    }


    // --------------------------------------------------------
    // Send JPEG
    // --------------------------------------------------------

    size_t written =
        webStreamClient.write(
            frame,
            frameSize
        );


    if (
        written != frameSize
    )
    {
        stopBrowserStream();

        return;
    }


    // --------------------------------------------------------
    // End frame
    // --------------------------------------------------------

    static const char frameEnd[] =
        "\r\n";


    if (
        webStreamClient.write(
            (const uint8_t*)frameEnd,
            2
        )
        != 2
    )
    {
        stopBrowserStream();

        return;
    }
}


// ============================================================
// 24. PROCESS RTSP
// ============================================================

void processRTSP()
{
    if (!cameraEnabled || !cameraReady)
    {
        return;
    }


    if (streamer == NULL)
    {
        return;
    }


    // --------------------------------------------------------
    // Handle RTSP protocol requests
    // --------------------------------------------------------

    streamer->handleRequests(0);


    // --------------------------------------------------------
    // Check existing RTSP connection
    // --------------------------------------------------------

    if (rtspActive)
    {
        /*
          Client disappeared.

          Stop RTSP and allow the browser to return after
          the short cleanup period.
        */

        if (
            !rtspClient ||
            !rtspClient.connected()
        )
        {
            stopRTSP();

            return;
        }


        // ----------------------------------------------------
        // Send RTSP camera frame
        // ----------------------------------------------------

        uint32_t now =
            millis();


        if (
            now - lastRTSPFrame >=
            RTSP_FRAME_INTERVAL
        )
        {
            streamer->streamImage(
                now
            );

            lastRTSPFrame =
                now;
        }


        return;
    }


    // --------------------------------------------------------
    // No RTSP client currently active
    // --------------------------------------------------------

    WiFiClient newClient =
        rtspServer.accept();


    if (newClient)
    {
        /*
          RTSP has priority immediately.

          Stop browser streaming BEFORE giving the camera
          to RTSP.
        */

        stopBrowserStream();


        /*
          Change camera to RTSP settings.

          This only changes sensor settings.
          The camera driver remains initialized.
        */

        setCameraMode(true);


        /*
          Disable TCP buffering where supported.

          This helps lower RTSP latency.
        */

        newClient.setNoDelay(
            true
        );


        rtspClient =
            newClient;


        rtspActive =
            true;


        lastRTSPFrame =
            millis();


        logMessage(
            "[RTSP] Client connected: " +
            rtspClient.remoteIP().toString()
        );


        /*
          Give the client to Micro-RTSP.
        */

        streamer->addSession(
            &rtspClient
        );
    }
}


// ============================================================
// 25. JPEG SNAPSHOT
// ============================================================

void handleJPG()
{
    if (!cameraEnabled || !cameraReady)
    {
        server.send(
            503,
            "text/plain",
            "Camera is OFF."
        );

        return;
    }


    /*
      Do not allow snapshot capture to interfere with RTSP.
    */

    if (rtspActive)
    {
        server.send(
            503,
            "text/plain",
            "RTSP is currently using the camera."
        );

        return;
    }


    /*
      Capture latest frame.
    */

    cam.run();


    uint8_t* frame =
        cam.getfb();


    size_t frameSize =
        cam.getSize();


    if (
        frame == NULL ||
        frameSize == 0
    )
    {
        server.send(
            500,
            "text/plain",
            "Camera frame failed."
        );

        return;
    }


    WiFiClient client =
        server.client();


    client.println(
        "HTTP/1.1 200 OK"
    );

    client.println(
        "Content-Type: image/jpeg"
    );

    client.println(
        "Content-Length: " +
        String(frameSize)
    );

    client.println(
        "Cache-Control: no-cache"
    );

    client.println(
        "Connection: close"
    );

    client.println();


    client.write(
        frame,
        frameSize
    );


    client.stop();
}


// ============================================================
// 26. CAMERA API
// ============================================================

void handleCameraAPI()
{
    if (!server.hasArg("set"))
    {
        server.send(
            400,
            "text/plain",
            "Missing command."
        );

        return;
    }


    String command =
        server.arg("set");


    command.toLowerCase();


    // --------------------------------------------------------
    // CAMERA ON
    // --------------------------------------------------------

    if (command == "on")
    {
        if (cameraReady)
        {
            cameraEnabled = true;

            setCameraMode(
                false
            );


            server.send(
                200,
                "text/plain",
                "Camera already ON."
            );

            return;
        }


        /*
          Reinitialize camera if it was previously disabled.
        */

        cameraEnabled = true;


        if (initializeCamera())
        {
            /*
              Return to normal browser streaming mode.
            */

            setCameraMode(
                false
            );


            server.send(
                200,
                "text/plain",
                "Camera ON."
            );
        }
        else
        {
            cameraEnabled = false;


            server.send(
                500,
                "text/plain",
                "Camera initialization failed."
            );
        }


        return;
    }


    // --------------------------------------------------------
    // CAMERA OFF
    // --------------------------------------------------------

    if (command == "off")
    {
        /*
          Stop all network camera usage first.
        */

        stopBrowserStream();

        stopRTSP();


        /*
          Deinitialize camera driver.

          This releases camera resources and PSRAM buffers.
        */

        if (cameraReady)
        {
            esp_camera_deinit();
        }


        cameraReady =
            false;

        cameraEnabled =
            false;


        logMessage(
            "[CAMERA] Camera OFF."
        );


        server.send(
            200,
            "text/plain",
            "Camera OFF."
        );


        return;
    }


    server.send(
        400,
        "text/plain",
        "Invalid command."
    );
}


// ============================================================
// 27. STATUS API
// ============================================================

void handleStatusAPI()
{
    String json;

    json.reserve(180);


    json += "{";


    json +=
        "\"camera\":";

    json +=
        cameraEnabled
        ? "true"
        : "false";


    json += ",";


    json +=
        "\"ready\":";

    json +=
        cameraReady
        ? "true"
        : "false";


    json += ",";


    json +=
        "\"rtsp\":";

    json +=
        rtspActive
        ? "true"
        : "false";


    json += ",";


    json +=
        "\"online\":";

    json +=
        (
            cameraEnabled &&
            cameraReady &&
            !rtspActive
        )
        ? "true"
        : "false";


    json += ",";


    json +=
        "\"ip\":\"192.168.4.1\"";


    json += "}";


    server.send(
        200,
        "application/json",
        json
    );
}


// ============================================================
// 28. SERIAL MONITOR PAGE
// ============================================================

void handleSerial()
{
    String page = R"rawliteral(
<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
      content="width=device-width,initial-scale=1">

<title>ESP32 Serial Monitor</title>


<style>

body {
    margin: 0;

    background: #020617;

    color: #d1fae5;

    font-family: monospace;
}

.header {
    padding: 16px;

    background: #0f172a;

    border-bottom:
        1px solid #334155;
}

h2 {
    margin: 0;
}

#log {
    height: calc(100vh - 70px);

    margin: 0;

    padding: 15px;

    overflow: auto;

    white-space: pre-wrap;

    font-size: 13px;
}

</style>

</head>


<body>


<div class="header">

<h2>
    ESP32 Serial Monitor
</h2>

</div>


<pre id="log">Loading...</pre>


<script>

function updateLog()
{
    fetch('/logs')

    .then(response =>
        response.text()
    )

    .then(data =>
    {
        let log =
            document.getElementById(
                'log'
            );

        log.textContent =
            data;

        log.scrollTop =
            log.scrollHeight;
    });
}


updateLog();


setInterval(
    updateLog,
    1000
);

</script>


</body>

</html>
)rawliteral";


    server.send(
        200,
        "text/html",
        page
    );
}


// ============================================================
// 29. LOG API
// ============================================================

void handleLogs()
{
    server.send(
        200,
        "text/plain",
        webLog
    );
}


// ============================================================
// 30. 404 PAGE
// ============================================================

void handleNotFound()
{
    server.send(
        404,
        "text/plain",
        "ESP32-CAM: Page not found."
    );
}


// ============================================================
// 31. SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial
    // --------------------------------------------------------

    Serial.begin(
        115200
    );


    /*
      Only a startup delay.

      No long delays will be used in the streaming loop.
    */

    delay(500);


    logMessage("");
    logMessage(
        "========================================"
    );

    logMessage(
        " ESP32-S3 OV3660 CAMERA SERVER"
    );

    logMessage(
        "========================================"
    );


    // --------------------------------------------------------
    // Wi-Fi AP
    // --------------------------------------------------------

    WiFi.mode(
        WIFI_AP
    );


    WiFi.softAPConfig(
        AP_IP,
        AP_GATEWAY,
        AP_SUBNET
    );


    bool apOK =
        WiFi.softAP(
            AP_SSID,
            AP_PASSWORD
        );


    if (!apOK)
    {
        logMessage(
            "[WIFI] ERROR: AP failed."
        );

        return;
    }


    /*
      Disable Wi-Fi sleep.

      Important for a continuous low-latency video stream.
    */

    WiFi.setSleep(
        false
    );


    delay(200);


    logMessage(
        "[WIFI] AP started."
    );


    logMessage(
        "[WIFI] SSID: ESP32-CAM"
    );


    logMessage(
        "[WIFI] IP: " +
        WiFi.softAPIP().toString()
    );


    // --------------------------------------------------------
    // Camera
    // --------------------------------------------------------

    cameraEnabled =
        true;


    if (
        !initializeCamera()
    )
    {
        cameraEnabled =
            false;


        logMessage(
            "[CAMERA] Camera failed to initialize."
        );
    }


    // --------------------------------------------------------
    // RTSP streamer
    // --------------------------------------------------------

    if (cameraReady)
    {
        streamer =
            new OV2640Streamer(
                &cam
            );
    }


    // --------------------------------------------------------
    // RTSP server
    // --------------------------------------------------------

    rtspServer.begin();


    logMessage(
        "[RTSP] Server started on port 554."
    );


    // --------------------------------------------------------
    // HTTP routes
    // --------------------------------------------------------

    server.on(
        "/",
        HTTP_GET,
        handleRoot
    );


    server.on(
        "/stream",
        HTTP_GET,
        handleStream
    );


    server.on(
        "/jpg",
        HTTP_GET,
        handleJPG
    );


    server.on(
        "/serial",
        HTTP_GET,
        handleSerial
    );


    server.on(
        "/logs",
        HTTP_GET,
        handleLogs
    );


    server.on(
        "/api/camera",
        HTTP_GET,
        handleCameraAPI
    );


    server.on(
        "/api/status",
        HTTP_GET,
        handleStatusAPI
    );


    server.onNotFound(
        handleNotFound
    );


    // --------------------------------------------------------
    // OTA
    // --------------------------------------------------------

    ElegantOTA.begin(
        &server
    );


    // --------------------------------------------------------
    // HTTP server
    // --------------------------------------------------------

    server.begin();


    // --------------------------------------------------------
    // Final status
    // --------------------------------------------------------

    logMessage(
        "[HTTP] Web server started."
    );

    logMessage(
        "[SYSTEM] Browser streaming is default."
    );

    logMessage(
        "[SYSTEM] RTSP has streaming priority."
    );

    logMessage("");

    logMessage(
        "Dashboard:"
    );

    logMessage(
        "http://192.168.4.1/" 
    ); 
 
    logMessage(""); 
 
    logMessage( 
        "RTSP:" 
    ); 
 
    logMessage( 
        "rtsp://192.168.4.1:554/mjpeg/1" 
    ); 
 
    logMessage(""); 
 
    logMessage( 
        "System ready." 
    ); 
 
    logMessage( 
        "========================================" 
    ); 
} 
 
 
// ============================================================ 
// 32. MAIN LOOP 
// ============================================================ 
 
void loop() 
{ 
    // ======================================================== 
    // HANDLE WEB REQUESTS 
    // ======================================================== 
 
    server.handleClient(); 
 
 
    // ======================================================== 
    // OTA 
    // ======================================================== 
 
    ElegantOTA.loop(); 
 
 
    // ======================================================== 
    // CAMERA CHECK 
    // ======================================================== 
 
    if (!cameraEnabled || !cameraReady) 
    { 
        /* 
          Nothing camera-related should run while the camera 
          is disabled. 
 
          Keep the CPU available for: 
              - web server 
              - OTA 
              - other system tasks 
        */ 
 
        delay(1); 
 
        return; 
    } 
 
 
    // ======================================================== 
    // RTSP PROCESSING 
    // ======================================================== 
 
    processRTSP(); 
 
 
    // ======================================================== 
    // BROWSER STREAMING 
    // ======================================================== 
 
    /* 
      Browser streaming only runs when RTSP is NOT active. 
    */ 
 
    if (!rtspActive) 
    { 
        processOnlineStream(); 
    } 
 
 
    // ======================================================== 
    // RESTORE BROWSER CAMERA MODE 
    // ======================================================== 
 
    /* 
      When RTSP disconnects, switch the sensor back to VGA. 
 
      We don't do this immediately in the same moment as 
      disconnecting RTSP. 
 
      A short 500 ms gap gives the RTSP session time to 
      completely disappear before the browser resumes. 
    */ 
 
    if ( 
        !rtspActive && 
        rtspDisconnectedAt != 0 
    ) 
    { 
        if ( 
            millis() - rtspDisconnectedAt 
            >= RTSP_RESUME_DELAY 
        ) 
        { 
            /* 
              Change back to browser-optimized settings. 
            */ 
 
            setCameraMode( 
                false 
            ); 
 
 
            /* 
              Clear the flag. 
 
              This prevents repeatedly changing the sensor 
              settings on every loop. 
            */ 
 
            rtspDisconnectedAt = 
                0; 
 
 
            logMessage( 
                "[WEB] Browser streaming resumed." 
            ); 
        } 
    } 
 
 
    // ======================================================== 
    // VERY SMALL IDLE DELAY 
    // ======================================================== 
 
    /* 
      Do NOT use delay(50), delay(100), etc. here. 
 
      The ESP32 needs to continuously process: 
 
          - RTSP packets 
          - browser packets 
          - HTTP requests 
          - OTA 
          - Wi-Fi 
 
      A 1 ms yield is enough to prevent unnecessary 
      CPU spinning while keeping the loop responsive. 
    */ 
 
    delay(1); 
}
//- this one worked
