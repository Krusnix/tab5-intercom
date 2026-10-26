# tab5-intercom

Add-on code for Trent's two M5Stack Tab5 intercoms (ESPHome + VoIP Stack
through Home Assistant).

The Tab5 firmware YAML lives in Home Assistant's ESPHome Builder. It pulls
the components in this repository with:

```yaml
external_components:
  - source: github://Krusnix/tab5-intercom@<commit>
    components: [tab5_video, tab5_broadcast, tab5_sdcard]
```

Always pin a commit (or tag), so a later change here never alters a build
by surprise.

## Components

### `tab5_video`

Video calls for a VoIP Stack (`n-IA-hane/esphome-voip-stack`, JPEG video
profile) ESP32-P4 device whose screen is drawn with a normal ESPHome display
lambda. VoIP Stack's own `p4_video_renderer` needs LVGL, and ESPHome does not
allow LVGL next to a display lambda, so this component does the receive side
without LVGL:

- Each received JPEG picture is decoded by the P4's hardware JPEG decoder on
  a small task of its own (stack in PSRAM).
- The ESPHome main loop scales it with the PPA to fill a rectangle of the
  screen (cutting off at most `max_crop`, otherwise with black bars), turns it
  to match the display's rotation, and writes it straight to the panel. All
  panel writes stay on the main loop, like the display lambda's.
- `has_frame()` is true while a picture is on the screen, so the lambda can
  draw only what is outside the picture. `on_first_frame` and
  `on_video_ended` tell it when to change layout. The picture goes off when
  the call's video ends or when no picture arrived for `no_video_timeout`.

The optional `camera:` block adds a gate in front of the camera's
`esp_jpeg_video_source`. Give the gate to VoIP Stack as `video.source`.
VoIP Stack can only change video direction during a call if video was agreed
when the call started, and it agrees to send only while sending is wanted.
With the gate, VoIP Stack always agrees on video both ways, while the camera
itself stays off at the start of every call. `set_enabled(true)` starts it
during a call; it stops again by itself when the call's video ends.

The gate also hands every picture it sends to the view's **self view**: the
device's own picture, scaled down and mirrored (like a mirror; the other side
sees it the right way round). The display lambda says where it goes with
`id(remote_video).set_self_view(x, y, w, h)` on every drawing (0 width hides
it); it is drawn again right after each screen update, and over the other
side's picture it is put into that picture before it reaches the panel, so
it never flickers. Turn it off with `self_view: false`.

```yaml
esp_jpeg_video_source:
  id: camera_video
  camera_id: tab5_camera
  width: 640
  height: 360
  framerate: 10

tab5_video:
  id: remote_video
  display_id: main_display
  display_rotation: 270        # same as the display's rotation:
  area: {x: 0, y: 0, width: 1280, height: 620}
  width: 640                   # picture size asked of the other side
  height: 360
  framerate: 10
  # max_decode_width: 1280     # largest picture accepted from any caller
  # max_decode_height: 720
  # max_crop: 15%
  # no_video_timeout: 3s
  # swap_red_blue: false       # if red and blue come out swapped
  camera:
    id: camera_gate
    source_id: camera_video
    # self_view: true          # own picture where set_self_view() says
    # self_view_mirror: true
    # self_view_framerate: 10
    # width: 640               # the camera's call picture size
    # height: 360
  on_first_frame:
    - component.update: main_display
  on_video_ended:
    - component.update: main_display

voip_stack:
  video:
    codec: jpeg
    source: camera_gate
    sink: remote_video
    width: 640
    height: 360
    framerate: 10
    rtp_port: 40002
    offer_payload_type: 26
    max_rtp_payload: 1400
```

In lambdas: `id(remote_video).has_frame()`, `is_receiving()`,
`frames_shown()`, `decode_errors()`, `set_self_view(x, y, w, h)`,
`get_self_view(&x, &y, &w, &h)`, and `id(camera_gate).set_enabled(bool)`,
`is_enabled()`, `is_available()`.

#### Still pictures (doorbell camera)

Outside calls, the same rectangle can show a still JPEG picture fetched over
plain HTTP, for example a Home Assistant camera's latest picture:

```yaml
tab5_video:
  # ...
  on_picture:              # picture changed, went away, or fetching failed
    - component.update: main_display
```

```cpp
// From a lambda on the main loop; call again every few seconds to pick up
// a newer picture (the screen only changes when the picture did).
id(remote_video).show_picture("http://192.168.4.114/api/camera_proxy/camera.front_door?token=...&width=960&height=540");
id(remote_video).hide_picture();
```

- A small task of its own (stack in PSRAM) fetches the picture with a plain
  HTTP/1.0 GET (no https), into a 1 MB PSRAM buffer made on first use. The
  same hardware decoder and PPA path as call video put it on the screen.
- An unchanged picture (same checksum) is not decoded or drawn again.
- A call's video always wins: `show_picture()` refuses while one is set up,
  and a call's video starting turns the still picture off.
- `has_picture()`, `picture_mode()`, `picture_error()` (a short sentence, or
  `""`), `picture_changed_ms()`, `picture_width()`, `picture_height()`.
- Pictures bigger than `max_decode_width` x `max_decode_height` are refused
  (Home Assistant's camera proxy shrinks them when given `width` and
  `height`). Baseline JPEG only (the P4 hardware decoder).

Note for the display lambda (built-in `mipi_dsi`): that driver only sends
pixels that changed in its own buffer. After the picture goes away, mark the
whole screen as changed before drawing (for example by drawing two corner
pixels in two different colours), or parts of the last picture stay. The
same goes for the self view's old rectangle when it moves or is hidden.

Built against ESPHome 2026.10.0, VoIP Stack v2026.11.0-dev and
esphome-esp-video-camera v2026.10.0-dev for the M5Stack Tab5.

### `tab5_broadcast`

Broadcasts between the two Tab5s: one Tab5 records a few seconds of voice,
the other one plays it right away. Two parts:

**The clip server** (`tab5_broadcast:`) keeps this Tab5's newest recording
and serves it as a WAV file on a small HTTP server of its own (ESP-IDF
`esp_http_server`, plain http, home network only). The other Tab5 plays it
straight from there with its `speaker_source` media player and an
`audio_http` media source, so the audio never goes through Home Assistant;
Home Assistant only passes the address on (an ESPHome API action).

```yaml
tab5_broadcast:
  id: bc_server
  port: 8099               # default
```

```cpp
// 16-bit mono samples (for example the microphone's, collected in PSRAM).
// Returns "http://<ip>:8099/broadcast/<id>.wav", or "" (no IPv4 address,
// no memory). Only the newest clip is kept; an older address answers 404.
std::string url = id(bc_server).publish_clip(samples, count, 16000);
```

- The clip is copied into PSRAM with a 44-byte WAV header, and a 10 ms fade
  in and out (no click at either end).
- Each clip gets a random id in its address, so an old address never plays
  a newer recording.
- At most two clients at a time; the server's task stack is in PSRAM. If
  the network isn't up yet it tries again every 10 s.
- `get_clip_url()`, `clear_clip()`, `get_fetch_count()`, `is_running()`.

**The pass-through speaker** (`speaker: - platform: tab5_broadcast`) goes
between a media player and the rest of its speaker chain. A media player sets
the volume and mute of its speaker, and the mixer and resampler speakers pass
those down to the hardware speaker, where they would change the volume of
calls and the ring too. This speaker passes the audio through unchanged and
keeps volume and mute to itself; set how loud the media player plays some
other way, for example with the mixer source's ducking
(`id(mixer_input)->apply_ducking(db, 0)`).

```yaml
speaker:
  - platform: tab5_broadcast
    id: bcast_out
    output_speaker: bcast_resampler   # resampler -> mixer source -> hardware

media_player:
  - platform: speaker_source
    media_pipeline:
      speaker: bcast_out
      # ...
```

`id(bcast_out).get_played_bytes()` / `reset_played_bytes()` count the audio
that went through, so a lambda can tell whether a playback made any sound.

### `tab5_sdcard`

The Tab5's microSD card. Its slot is on the ESP32-P4's SD host, slot 0
(4-bit: CLK 43, CMD 44, D0-D3 39-42), with the pins powered by the P4's
on-chip LDO 4. The Wi-Fi chip (`esp32_hosted`) sits on slot 1 of the same SD
host, so this component:

- powers the slot with ESP-IDF's on-chip LDO driver, starts only slot 0 (the
  host already runs for the Wi-Fi chip) and never shuts the host or the slot
  down again, so Wi-Fi is never cut off;
- mounts the card once at start-up, before Wi-Fi connects (setting up a slot
  changes the shared host's clock), and again only when asked with
  `check()`. Don't call `check()` during a call;
- runs at 40 MHz like M5Stack's own Tab5 code (the same host clock as the
  Wi-Fi chip);
- does its slow work (free space, the read/write test) on a task of its own,
  with its stack in PSRAM.

The card must be FAT32 (cards up to 32 GB come that way; exFAT isn't
supported). Its files are then plain C/POSIX files under the mount point.

```yaml
tab5_sdcard:
  id: sd_card
  mount_point: /sdcard     # default
  ldo_channel: 4           # default (the Tab5's)
  frequency: 40MHz         # default; 20MHz for slow cards
  max_files: 4             # files open at once
  on_change:               # mounted or not, free space, test result changed
    - lambda: id(sd_status).publish_state(id(sd_card).get_status());
```

```cpp
id(sd_card).is_mounted();      // the files can be used
id(sd_card).path("a/b.wav");   // "/sdcard/a/b.wav"
id(sd_card).get_status();      // "Ready", "No card found", "Not FAT32: ..."
id(sd_card).get_card_info();   // "SDHC 31.9 GB (SD32G), 40 MHz"
id(sd_card).get_size_gb();     // NAN until known
id(sd_card).get_free_gb();
id(sd_card).check();           // mount again if needed, then a 4 MB
                               // write/read test on the card's task
id(sd_card).get_test_result(); // "", "Testing...", "OK: writes 9.1 MB/s,
                               // reads 11.8 MB/s" or "Failed: ..."
```

`run_on_card_task(job)` queues other card work on the card's task,
`refresh_space()` measures the free space again, and
`report_card_error(what)` marks the card as gone until the next `check()`.

ESPHome 2026.10 leaves ESP-IDF's FAT and SD card parts out of builds; this
component puts them back (`fatfs`, `sdmmc`, `esp_driver_sdmmc`,
`wear_levelling`) with long file names and FAT's buffers in PSRAM.

Built against ESPHome 2026.10.0 (ESP-IDF 5.5) for the M5Stack Tab5.
