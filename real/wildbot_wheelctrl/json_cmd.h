#define FB_MOTOR 20010
#define FB_INFO	 20011

// all wheels in one line, sent every poll cycle. See poll_send_frame().
#define FB_MOTOR_ALL 20012

// {"T":10000,"id":1}
// wheel_stop(id)
#define CMD_WHEEL_STOP	10000

// {"T":10010,"id":1,"cmd":50,"act":3}
// wheel_ctrl(id, cmd, act)
#define CMD_WHEEL_CTRL 10010

// set the target of every wheel and start polling mode.
// "cmd" is indexed by wheel id - 1.
// {"T":10011,"cmd":[50,50,-50,-50],"act":1}
#define CMD_WHEEL_CTRL_ALL 10011

// 1: current loop
// 2: speed loop
// 3: position loop
// {"T":10012,"id":1,"mode":2}
// wheel_change_mode(id, mode)
#define CMD_WHEEL_CHANGE_MODE	10012

// leave ("en":0) or enter ("en":1) polling mode. Leaving stops the wheels.
// {"T":10013,"en":0}
#define CMD_WHEEL_POLL	10013


// get other info
// {"T":10032,"id":1}
// wheel_get_info(id)
#define CMD_WHEEL_INFO	10032

// {"T":11001,"time":2000}
// {"T":11001,"time":-1}
// set_heartbeat_time(time_ms)
#define CMD_HEARTBEAT_TIME	11001



// === === === esp32 settings. === === ===

// reboot device.
// {"T":600}
#define CMD_REBOOT 600

