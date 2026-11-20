# CANBUS Stepper: Getting Started

Just received your CANBUS Stepper? This guide walks you through assembling it, spinning a motor, and configuring the driver using the web GUI.

> **Web GUI:** [thingsbyjosh.com/canbus_gui](https://thingsbyjosh.com/canbus_gui)

---

## Contents

1. [Assembling the Kit onto a NEMA 17](#1-assembling-the-kit-onto-a-nema-17)
2. [Powering the Board](#2-powering-the-board)
3. [Daisy Chaining Boards](#3-daisy-chaining-boards)
4. [Using the Web GUI](#4-using-the-web-gui)
5. [Closed Loop Control](#5-closed-loop-control)
6. [Sensorless Homing](#6-sensorless-homing)
7. [Serial Control](#7-serial-control)
8. [Custom CAN Messages](#8-custom-can-messages)

---

## 1. Assembling the Kit onto a NEMA 17

### What you'll need

| Item | Notes |
|---|---|
|CANBUS Stepper Kit| Available at [thingsbyjosh.com](https://thingsbyjosh.com/products/canbus-stepper)|
| NEMA 17 stepper motor | Ideally **34 mm long** with a standard **6-pin JST PH** connector |
| Superglue | Only needed if you want encoder position readings / closed loop control |

**Example motor:** Hanpose 17HS3401S, available on [Amazon](https://www.amazon.com/Captive-Stepper-17HS3401S-4-Lead-Printer/dp/B0G9KRJFTY) and [AliExpress](https://www.aliexpress.com/item/1005006577156384.html).

---

### Step 1: Install the magnet *(encoder only)*

Superglue the included diametrically magnetised magnet onto the **centre of the motor shaft**.

> [!CAUTION]
> Use only a very small amount of glue, and keep it out of the motor bearing.

<img src="https://github.com/user-attachments/assets/c32ad5e4-c292-4e67-a0ee-864bf49cce15" width="40%" />

---

### Step 2: Remove the original motor screws

<img src="https://github.com/user-attachments/assets/aff3b972-af76-4eab-b2e3-df1b70109dc4" width="40%" />

---

### Step 3: Apply a heat pad to the heat spreader

Peel off the protective film and stick one of the included heat pads onto the heat spreader.

> [!IMPORTANT]
> The heat pad goes on the **recessed side** of the heat spreader.

<p float="left">
  <img src="https://github.com/user-attachments/assets/619f5212-02ab-4623-b3c7-b2ffd994ca20" width="40%" />
  <img src="https://github.com/user-attachments/assets/c62a9630-885b-43b8-9ef4-58c4579a1af6" width="40%" />
</p>

---

### Step 4: Fit the spacer

Place the spacer onto the motor in the orientation shown, relative to the motor connector.

<img src="https://github.com/user-attachments/assets/975f0007-0633-43b5-bc02-90deb9b1e985" width="40%" />

---

### Step 5: Add the PCB

Install the PCB in the orientation shown so that both the **motor connectors** and the **heat pad** line up.

<img src="https://github.com/user-attachments/assets/a98cf78d-519c-4a04-afc4-1bb1bafb2742" width="40%" />

---

### Step 6: Add the cover and countersunk screws

Fit the cover and secure it with the included M3 × 40 mm countersunk screws.

> [!NOTE]
> These screws suit a 34 mm NEMA 17. Other motor lengths may need longer or shorter screws.

<p float="left">
  <img src="https://github.com/user-attachments/assets/860b4957-be2f-4ece-8546-8102fa0447d0" width="40%" />
  <img src="https://github.com/user-attachments/assets/90658808-802d-4ca8-b0fa-40012e529a51" width="40%" />
</p>

---

### Step 7: Add the heatsink

Use the **larger heatsink** if you plan to run the motor for long periods and/or at high currents.

<p float="left">
  <img src="https://github.com/user-attachments/assets/281de296-c433-4fc9-ba4a-39a087a0c895" width="40%" />
  <img src="https://github.com/user-attachments/assets/5bc483ff-dd3c-4c62-a775-2643c6d06de9" width="40%" />
</p>

---

### Step 8: Plug in the motor loom

The kit includes **two looms** to suit different motors. Try one first. If the motor stutters or doesn't spin in the later steps, power off the board and try the other.

> [!WARNING]
> Never connect or disconnect the motor while the board is powered.

<img src="https://github.com/user-attachments/assets/fc9cb31f-0f76-4702-9303-2ef88845c0f4" width="40%" />

---

## 2. Powering the Board

Connect a **12 V or 24 V** supply to the **PWR IN** connector. A standard **XT30** connector can be used.

<img src="https://github.com/user-attachments/assets/b363f0bc-ffd2-418a-9c67-79c06eeb4b52" width="50%" />

| Power source | Configure & monitor | Drive the motor |
|---|:---:|:---:|
| 12 V / 24 V via PWR IN | ✅ | ✅ |
| USB Type-C only | ✅ | ❌ |

> [!TIP]
> Want a stepper driver powered entirely over USB? See the [PD Stepper](https://thingsbyjosh.com/products/pd-stepper).

---

## 3. Daisy Chaining Boards

Pay attention to the **IN** vs **OUT** labels on the power connectors.

<img src="https://github.com/user-attachments/assets/8caea5b8-1b53-4eff-b528-4ec1e6a78cea" width="50%" />

> [!WARNING]
> **CAN termination:** Set the **CAN TERM** DIP switch **ON** for the two boards at each end of the chain, and **OFF** for every board in between.

> [!WARNING]
> **Unique node IDs:** Every board on the bus needs its own node ID (1–31).
> To set it, **hold SW1 while powering on** the board. The LED flashes once per count. Release the button when it reaches the ID you want.

---

## 4. Using the Web GUI

The [Web GUI](https://thingsbyjosh.com/canbus_gui) is used to control, configure, and update the CANBUS Stepper.

### Connecting

1. Connect the board to your PC using the **USB Type-C** port on the PCB.
2. Open the Web GUI, click **Connect via USB**, and select your board.
   - On Windows it appears as something like **"USB JTAG/serial debug unit (COMxx)"**.
   - You can confirm the COM port number in **Device Manager**.

<p float="left">
  <img src="https://github.com/user-attachments/assets/fc696aa7-0247-400b-b9af-5dba61f54305" width="53%" />
  <img src="https://github.com/user-attachments/assets/e7d89f7a-5c0b-406d-8234-d92349b7a5e2" width="43%" />
</p>

### Selecting a node

3. Click **"x Nodes Found"** (bottom right) to see every discovered node. Click one to view it, or pick its ID from **TARGET NODE** (top right).

> [!TIP]
> Setting **TARGET NODE** to `0` broadcasts the command to **all** nodes on the bus.

### Changing settings

4. Double-click any value under **Node Device Settings** to edit it. Once you've confirmed the change works, click **SAVE TO FLASH** so it persists across power cycles.

### Updating firmware

5. Click **UPDATE FIRMWARE** to automatically flash the latest firmware straight from this GitHub repo.

<p float="left">
  <img src="https://github.com/user-attachments/assets/fc04c014-c239-455f-8515-ca8a926522e6" width="60%" />
  <img src="https://github.com/user-attachments/assets/ceb72661-ec54-4ed3-aaed-dc5fe9b0db12" width="28%" />
</p>

### Connecting over WiFi *(optional)*

6. While connected via USB, enter your WiFi details to enable wireless control over WebSockets. You can then unplug the USB and connect using either:
   - the hostname `canbus-stepper-[NODE ID].local`, or
   - the node's IP address.

<p float="left">
  <img src="https://github.com/user-attachments/assets/96b8514c-f808-4762-acfe-8a27aed66d46" width="55%" />
  <img src="https://github.com/user-attachments/assets/71e49f1b-d34d-458a-89e9-64133c448618" width="43%" />
</p>

> [!NOTE]
> The **analog input** on the AUX connector can't be used while WiFi is enabled, due to an IO clash.

---

## 5. Closed Loop Control

To enable closed loop control from the Web GUI:

1. Under **LIVE TELEMETRY**, note the current **Encoder Angle (deg)**.
2. Double-click **Set Position (deg)** and set it to `360`.
3. Check how the **Encoder Angle** changed:

   | Encoder Angle result | Action |
   |---|---|
   | Increased by 360° (± 1°) | ✅ All good, continue |
   | Decreased by 360° | Set **Map Direction** to `1` |
   | Increased by 180° | Set **Steps per Rev** to `400` |

4. Set **Closed Loop Type** to `1`.
5. Once it's working, click **SAVE TO FLASH**.

---

## 6. Sensorless Homing

Sensorless homing uses the **StallGuard** feature of the TMC2209 driver. The motor drives until it hits a hard stop, detects the increase in load, then stops and sets that point as zero.

### Procedure

1. Set **Stallguard Behaviour** (MsgType `7`) to `4`.
2. Set **Set Velocity (deg/sec)** (MsgType `3`) to your desired speed and direction.

When StallGuard triggers, the motor stops and the zero position is reset.

### Tuning

| Setting | MsgType | Purpose |
|---|:---:|---|
| Stallguard Threshold | `12` | Adjusts the motor load at which a stall is detected |
| Stallguard Value | N/A | Live readout of the current load (useful for tuning the threshold) |
| Stallguard Behaviour | `7` | Choose what happens on a stall (`4` = stop and re-zero) |

All of these are available in the Web GUI.

---

## 7. Serial Control

The **AUX connector** can be configured to control a node (or a whole system of nodes) over hardware serial.

1. Double-click the **AUX Connector** setting.
2. Select **Serial Control**.

<img width="606" alt="AUX connector set to Serial Control" src="https://github.com/user-attachments/assets/4003c5b0-0bea-4c77-a773-f1d0a084e380" />

| Parameter | Value |
|---|---|
| Baud rate | `115200` |
| Format | `8N1` |
| Message format | Same as CAN packets (see the **CAN Protocol** document) |

> [!NOTE]
> The USB serial connection remains available while serial control is enabled.

---

## 8. Custom CAN Messages

The **CAN message builder** (top right of the Web GUI) lets you build and send CAN messages manually. This is handy for debugging when you're writing your own control interface.

See the **CAN Protocol** document for details on the packet structure.
