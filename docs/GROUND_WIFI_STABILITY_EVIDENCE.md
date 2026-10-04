# Ground Wi-Fi Stability: Evidence, Fix, and Verification

Date: 2026-10-02

## Observed failure

- `PHOENIX-GROUND` associates, the dashboard opens, then reports `DASHBOARD ERROR`
  and the client disconnects after roughly 5–10 seconds.
- The ESP32-S3 remains alive on USB serial and the SSID can remain visible. This makes
  a whole-board reset or immediate power loss less likely than an AP/client or HTTP
  traffic failure for the observed USB-powered test.
- The dashboard previously started `/api/status` fetches every 500 ms with
  `setInterval`. A slow request did not prevent another request from starting. The
  firmware uses Arduino's synchronous `WebServer`, so overlapping browser requests can
  build a backlog and cause timeouts even though the Wi-Fi radio is still operating.
- A controlled connection test with USB serial attached produced a repeatable ESP32-S3
  stack-canary panic immediately after `/api/status`. Address-to-line decoding placed
  the crash in `statusJson()` during `snprintf` floating-point conversion. That
  function placed a 5,200-byte buffer on Arduino `loopTask`'s limited stack. The board
  then rebooted, which explains both the dashboard failure and apparent Wi-Fi dropout.
  Reset reasons after the panic were software CPU reset and panic reset, not brownout.

## Published and primary evidence

1. Espressif documents a specific ESP32/ESP32-S2/ESP32-S3 SoftAP interoperability
   problem in which phones and PCs disconnect while traffic is flowing. Espressif's
   recommended mitigation is to disable Wi-Fi A-MPDU RX and TX and use packet capture
   for deeper confirmation:
   <https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/wifi.html>
2. Espressif identifies A-MPDU as an IEEE 802.11n aggregation feature. This firmware's
   installed Arduino/ESP-IDF libraries are precompiled, so their menuconfig switches
   cannot be changed per project. The runtime mitigation therefore removes 802.11n
   from the SoftAP protocol bitmap and runs 802.11b/g at HT20. That prevents this AP
   from negotiating the 802.11n path associated with A-MPDU, trading unneeded
   throughput for compatibility.
3. Apple documents that choosing **Without Internet** keeps an iPhone/iPad associated
   with a captive network, while cancelling the captive page disassociates it:
   <https://support.apple.com/en-gb/102554>
4. RFC 8952 explains that an operating system can avoid making a captive Wi-Fi network
   its default route while access remains restricted. `PHOENIX-GROUND` is intentionally
   local-only, so the OS can label it as having no Internet even when the dashboard is
   healthy: <https://www.rfc-editor.org/rfc/rfc8952.html>
5. A peer-reviewed laboratory study found that non-Wi-Fi interference in the 2.4 GHz
   ISM band reduced mean 802.11 throughput by 85%. This supports testing away from
   2.4 GHz interferers before concluding that a remaining failure is firmware-only:
   <https://pmc.ncbi.nlm.nih.gov/articles/PMC8068345/>
6. A published ESP32 system design reports that wireless startup current peaks require
   an adequate regulator and capacitors to avoid resets from supply-voltage drop. This
   is relevant to later battery tests, but the current USB observation does not point
   to it as the leading cause:
   <https://doi.org/10.3390/en15228707>

## Implemented controls

- The 6,000-byte JSON formatting buffer is now static storage instead of loop-task
  stack storage. This directly fixes the reproduced crash mechanism.
- Only one dashboard refresh may be in flight at a time.
- Refresh period changed from 500 ms to 1000 ms.
- Each fetch aborts after 3 seconds instead of accumulating indefinitely.
- JSON responses use `Cache-Control: no-store` and close each HTTP connection.
- SoftAP operates in conservative 802.11b/g, 20 MHz compatibility mode.
- WPA2-PSK, a standard 100-TU beacon interval, disabled Wi-Fi sleep, and the existing
  maximum configured transmit power are explicit.
- Firmware records AP client connect/disconnect counts, AP-stop count, uptime, reset
  reason, and free heap in `/api/status` and logs client events over USB serial.

## Verification procedure

1. Flash the ground board and press RST once after flashing.
2. Forget the old `PHOENIX-GROUND` network on the phone/computer, then rejoin it with
   password `phoenixground`.
3. If the OS warns that there is no Internet, choose **Without Internet** or its
   equivalent. Do not press Cancel on Apple's captive-network page.
4. Open `http://192.168.8.1/` explicitly in a full browser.
5. Leave the dashboard open for at least 10 minutes. A successful test requires:
   no Wi-Fi disconnect, no `DASHBOARD ERROR`, and continuously changing uptime.
6. If it fails, capture USB serial through the failure. Compare the last uptime/reset
   reason and the Wi-Fi connect/disconnect/AP-stop counters:
   - uptime restarts: board reset/power problem;
   - disconnect count rises without restart: radio/client interoperability or RF issue;
   - association stays up but fetch times out: HTTP scheduling/application issue;
   - AP-stop count rises: Wi-Fi driver/AP lifecycle fault.

## Verification result (2026-10-02)

- Build: successful with PlatformIO `espressif32@7.0.1`.
- Flash: successful to the confirmed ground-board MAC `A4:CB:8F:A7:03:6C`.
- Reproduction before the stack fix: requesting `/api/status` caused a stack-canary
  panic and repeated software resets within seconds.
- After the fix: 75 successful live API responses; uptime advanced from 14,789 ms to
  113,540 ms with no board reset, no AP stop, and no client disconnect during that
  measurement window. Fifteen later requests failed only after the Mac selected its
  known Internet-connected home network.
- Reconnection validation: `/` returned HTTP 200 with 23,803 bytes and `/api/status`
  returned HTTP 200 with 1,728 bytes at uptime 186,901 ms. The API reported reset
  reason 0 and AP-stop count 0.

## Limits of the evidence

The stack-buffer change addresses the reproduced crash mechanism; the traffic and
compatibility controls address documented secondary mechanisms. They do not prove the
physical RF environment, power rail, or client behavior is healthy. A
10-minute connected test plus the new event log is required before calling the defect
fixed. If the compatibility build still disconnects, the next rigorous step is an
802.11 packet capture and a simultaneous 3.3 V rail measurement, as Espressif advises.
