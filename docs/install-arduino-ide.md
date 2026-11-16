# Installing Emu32 with Arduino IDE

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software).
2. Open Arduino IDE and select **File → Preferences**.
3. Add this URL under **Additional Boards Manager URLs**:

   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

4. Open **Tools → Board → Boards Manager**.
5. Search for **esp32** and install **esp32 by Espressif Systems**.
6. Download the Emu32 ZIP package for your device and extract it.
7. Open the matching `.ino` file from the extracted device folder:
   - ESP32-C6 1.47-inch: `esp32-c6-1.47/esp32-c6-1.47.ino`
   - ESP32-S3 1.47-inch: `esp32-s3-1.47/esp32-s3-1.47.ino`
   - ESP32-S3 1.54-inch: `esp32-s3-1.54/esp32-s3-1.54.ino`
8. Connect your ESP32 device to your computer with a USB data cable.
9. In Arduino IDE, select the matching ESP32 board under **Tools → Board** and select the device's port under **Tools → Port**.
10. Click **Upload** and wait for the upload to finish.
11. Restart the device. Emu32 should start.

If the upload does not start, put the board into download mode using its BOOT button, then try uploading again.
