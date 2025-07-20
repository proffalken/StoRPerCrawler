# StoRPer micro-ROS Firmware

This repository contains the firmware for controlling a [StoRPer](https://github.com/concretedog/stoRPer) tracked robot using [micro-ROS](https://micro.ros.org/) on a Raspberry Pi Pico W. It receives ROS 2 `Twist` commands over Wi-Fi and drives the rear motors via PWM.

---

## 🔗 Related Projects

- 🤖 **StoRPer Robot Hardware**: https://github.com/concretedog/stoRPer
- 🧠 **ROS 2 micro-ROS Agent**: https://github.com/micro-ROS/micro-ros-agent
- 📦 **PlatformIO**: https://platformio.org/

---

## 📦 Requirements

- [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html)
- Raspberry Pi Pico W with StoRPer firmware wiring
- micro-ROS Agent running (Docker or native)

---

## 🔐 Environment Variables

This project **does not hardcode any credentials**. Instead, create a `.env` file in the project root:

```env
WIFI_SSID="your_wifi_name"
WIFI_PASS="your_wifi_password"
AGENT_IP="192.168.8.5"
AGENT_PORT=8888
````

Then load the environment:

```bash
source .env
```

Or use the provided helper:

```bash
source load_env.sh
```

---

## 🚀 Flashing the Firmware

1. Clone the repo and enter the directory:

   ```bash
   git clone https://github.com/yourname/storper-microros-firmware.git
   cd storper-microros-firmware
   ```

2. Install dependencies and load your `.env`:

   ```bash
   platformio run
   platformio run -t upload
   ```

---

## 🔄 Updating Code

To pull updates and rebuild:

```bash
git pull
source .env
platformio run -t upload
```

---

## 📋 Testing Movement

Use the included bash script to send test `Twist` commands to `/rt/cmd_vel`:

```bash
./test_rt_cmd_sequence.sh
```

This will:

1. Drive forward at 50% for 5s
2. Reverse at 20% for 5s
3. Turn right while driving forward
4. Turn left while reversing

---

## 🧠 Notes

* Rear motor pins match the StoRPer board:

  * Right motor: GPIO 12 (forward), 13 (reverse)
  * Left motor: GPIO 8 (forward), 9 (reverse)
* ROS 2 expects values in the range `-1.0` to `1.0`
* The agent remaps `/cmd_vel` → `/rt/cmd_vel`

---

## 🛠️ Troubleshooting

* If the build fails with `expected primary-expression`, ensure you exported all variables in `.env`
* Use `pio run -t clean` to clear intermediate files
* To debug Wi-Fi/PWM output, open serial monitor:

  ```bash
  platformio device monitor
  ```

---

## 📜 License

MIT – see [LICENSE](./LICENSE)

---

Made with ❤️ for embedded ROS adventure.
